#include "MPU9250.h"
#include "sd_lib.hpp"
#include "cansat_gps.hpp"
#include "azimuthLibrary.hpp"
#include "yaw_offset.hpp"
#include <stdlib.h>

// 設定変数
const bool USE_YAW_OFFSET = false; // 計算したyawのオフセット値を使うかどうか
const int MAG_DECLINATION = 9.1; // 地磁気偏角

MPU9250 mpu;
MySD mysd = MySD(); // SDカードへの書き込み処理を行う
Cansat_gps gps = Cansat_gps(); // GPSの値を読み込む
Azimuth azimuth = Azimuth(); // 方位角の計算を行う
YawOffset yawOffset = YawOffset(); // yawの補正値を計算する

HardwareSerial gps_serial(2); // GPSの通信

// 目的地の座標
double goal_lat = 0;
double goal_lng = 0;

const int motorA[3] = {4, 13, 25};  // AIN1, AIN2, PWMA
const int motorB[3] = {14, 27, 26}; // BIN1, BIN2, PWMB
const int CHANNEL_A = 0;
const int CHANNEL_B = 1;
const int LEDC_TIMER_BIT = 8;
const int LEDC_BASE_FREQ = 490;

double current_yaw = 0;
double yaw_offset = 0;
double yaw_sum = 0;
double yaw_count = 0;
double direction = 0;
unsigned long start_time = 0;
unsigned long close_to_goal_start_time = 0;
double current_lat = 0;
double current_lng = 0;
char log_buf[300];
String change_direct_log = "";
char change_direct_log_buf[4096];

// 100kinsatの状態の定義
enum State {
  Start,
  CalcYawOffset,
  YawOffsetSet,
  GetGPS,
  ChangeDirection,
  MoveTo,
  CloseToGoalGetGPS,
  CloseToGoalMove,
  End
};
State state = Start;

enum ChangeDirectState {
  CD_Start,
  CD_Right,
  CD_Left
};
ChangeDirectState cd_state = CD_Start;

enum DistanceState {
  GreaterThan5m,
  Between1mAnd5m,
  LessThan1m
};
DistanceState distance_state = GreaterThan5m;


// モータを動作・停止させる
void moveMoter(int left_moter, int right_moter) {
  String msg = String("moter,");
  if (left_moter == 0) {
    digitalWrite(motorA[0], LOW);
    digitalWrite(motorA[1], LOW);
    ledcWrite(CHANNEL_A, 0);
    msg += String("A,LOW,LOW,0");
  } else if (left_moter > 0) {
    digitalWrite(motorA[0], LOW);
    digitalWrite(motorA[1], HIGH);
    ledcWrite(CHANNEL_A, left_moter);
    msg += String("A,LOW,HIGH,") + String(left_moter);
  } else {
    digitalWrite(motorA[0], HIGH);
    digitalWrite(motorA[1], LOW);
    ledcWrite(CHANNEL_A, -1 * left_moter);
    msg += String("A,HIGH,LOW,") + String(-1 * left_moter);
  }
  if (right_moter == 0) {
    digitalWrite(motorB[0], LOW);
    digitalWrite(motorB[1], LOW);
    ledcWrite(CHANNEL_B, 0);
    msg += String(",B,LOW,LOW,0");
  } else if (right_moter > 0) {
    digitalWrite(motorB[0], HIGH);
    digitalWrite(motorB[1], LOW);
    ledcWrite(CHANNEL_B, right_moter);
    msg += String(",B,HIGH,LOW,") + String(right_moter);
  } else {
    digitalWrite(motorB[0], LOW);
    digitalWrite(motorB[1], HIGH);
    ledcWrite(CHANNEL_B, -1 * right_moter);
    msg += String(",B,LOW,HIGH,") + String(-1 * right_moter);
  }
  msg.toCharArray(log_buf, 300);
  mysd.appendLog(log_buf);
}

void mpuAppendLog(float accX, float accY, float accZ, float gyroX, float gyroY, float gyroZ, float magX, float magY, float magZ, float yaw) {
  unsigned char n = 3;
  change_direct_log += String("sensor,mpu,") + String(accX, n) + "," + String(accY, n) + "," + String(accZ, n) + ",";
  change_direct_log += String(gyroX, n) + "," + String(gyroY, n) + "," + String(gyroZ, n) + ",";
  change_direct_log += String(magX, n) + "," + String(magY, n) + "," + String(magZ, n) + "," + String(yaw, n) + "\n";
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  gps_serial.begin(9600);
  delay(2000);

  Serial.println("start program");

  Serial.println("set up MPU");
  if (!mpu.setup(0x68)) {
    while (true) {
      Serial.println("MPU connection failed. Please restart 100kinsat.");
    }
  }
  // GPSの値が正常に取れるまで待機
  start_time = millis();
  while(true) {
    if (millis() - start_time > 5 * 60 * 1000) {
      Serial.println("Unable to read GPS value.");
      mysd.appendLog("Unable to read GPS value.");
      state = End;
      return;
    }
    if (gps_serial.available() > 0) {
      char c = gps_serial.read();
      if(gps.getValues(c, &current_lng, &current_lat)) {
        break;
      }
    }
  }

  // キャリブレーションで求めた補正値を設定する

  // == EXAMPLE (You need to calcurate these values on your device) ==
  // < calibration parameters >
  // accel bias [g]: 
  //-309.97, 12.99, -20.75
  //gyro bias [deg/s]: 
  //-0.15, 1.17, 0.39
  //mag bias [mG]: 
  //-500.46, 370.35, 425.59
  //mag scale []: 
  //0.98, 1.04, 0.98
  //   < calibration parameters >
  // accel bias [g]: 
  // -369.58, -15.69, -48.74
  // gyro bias [deg/s]: 
  // -0.06, 1.11, 0.25
  // mag bias [mG]: 
  // -496.91, 398.84, 403.28
  // mag scale []: 
  // 0.98, 1.01, 1.01

  mpu.setAccBias(-369.97, -15.69, -48.74);
  mpu.setGyroBias(-0.06, 1.11, 0.25);
  mpu.setMagBias(-496.46, 398.84, 403.28);
  mpu.setMagScale(1,1,1);
  mpu.selectFilter(QuatFilterSel::MADGWICK);

  // MPUのフィルターが上手く機能するまで待機
  start_time = millis();
  while(true) {
    if (millis() - start_time > 2000) {
      break;
    }
    mpu.update();
  }

  // 方位角の計算のため、目的地を設定
  azimuth.set_goal(goal_lng, goal_lat);

  //モーターのセットアップ
  for (int i = 0; i < 3; i++) {
    pinMode(motorA[i], OUTPUT);
    pinMode(motorB[i], OUTPUT);
  }
  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(motorA[2], CHANNEL_A);
  ledcAttachPin(motorB[2], CHANNEL_B);

  // 開始時刻を記録
  start_time = millis();
  // 停止状態から開始
  moveMoter(0, 0);
  String start_msg = String("Start\ngoal,gps,") + String(goal_lat, 8) + "," + String(goal_lng, 8);
  start_msg.toCharArray(log_buf, 300);
  mysd.appendLog(log_buf);
}

void loop() {
  switch(state) {
    case Start:
      Serial.println("Start moving");
      yaw_offset = 0;
      if (USE_YAW_OFFSET) {
        state = CalcYawOffset;
        mysd.appendLog("transition,CalcYawOffset");
      } else {
        state = GetGPS;
        mysd.appendLog("transition,GetGPS");
      }
      break;

    case CalcYawOffset:
      if (mpu.update()) {
        float magY = mpu.getMagY();
        float yaw = mpu.getYaw();
        yawOffset.update(magY, yaw);
        mpuAppendLog(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(), mpu.getMagX(), magY, mpu.getMagZ(), yaw);
      }
      if (millis() - start_time > 30 * 1000) {
        state = YawOffsetSet;
        mysd.appendLog("transition,YawOffsetSet");
      }
      break;

    case YawOffsetSet:
      yaw_offset = yawOffset.getOffset();
      char buf[50];
      (String("yawOffset,") + yaw_offset).toCharArray(buf, 50);
      Serial.println(buf);
      mysd.appendLog(buf);
      moveMoter(0, 0);
      state = GetGPS;
      mysd.appendLog("transition,GetGPS");
      break;

    // GPSで現在地を取得し、目的地までの距離・方位を求める
    case GetGPS:
      if (mpu.update()) {
        mpu.getYaw();
      }
      if (gps_serial.available() > 0) {
        char c = gps_serial.read();
        if(gps.getValues(c, &current_lng, &current_lat)) {
          double distance = -1;
          azimuth.distance_to_goal(current_lng, current_lat, &distance);
          if (0 <= distance && distance < 0.001) {
            distance_state = LessThan1m;
          } else if (0.001 <= distance && distance < 0.005) {
            if (distance_state == GreaterThan5m) {
              close_to_goal_start_time = millis();
            }
            distance_state = Between1mAnd5m;
          } else {
            distance_state = GreaterThan5m;
          }
          String msg = "";
          switch (distance_state) {
            case LessThan1m:
              // 1m以内ので停止する
              moveMoter(0,0);
              msg = String("azimuth,distance,") + String(distance, 5) + "\n";
              msg += String("transition,End");
              msg.toCharArray(log_buf, 300);
              Serial.println(msg);
              mysd.appendLog(log_buf);
              state = End;
              break;
            case Between1mAnd5m:
              if (millis() - close_to_goal_start_time > 60 * 1000) {
                // 5m以内に入ってから60秒が経過しているので停止
                moveMoter(0,0);
                msg = String("azimuth,distance,") + String(distance, 5) + "\n";
                msg += String("control,CloseToGoalTimeout\ntransition,End");
                msg.toCharArray(log_buf, 300);
                Serial.println(msg);
                mysd.appendLog(log_buf);
                state = End;
              } else {
                azimuth.azimuth_to_goal(current_lng, current_lat, &direction);
                msg = String("sensor,gps,") + String(current_lat, 14) + "," + String(current_lng, 14) + String("\n");
                msg += String("azimuth,direction,") + String(direction, 5) + String("\n");
                msg += String("azimuth,distance,") + String(distance, 5) + String("\n");
                msg += "transition,ChangeDirection";
                msg.toCharArray(log_buf, 300);
                Serial.println(msg);
                mysd.appendLog(log_buf);
                state = ChangeDirection;
              }
              break;
            case GreaterThan5m:
              azimuth.azimuth_to_goal(current_lng, current_lat, &direction);
              msg = String("sensor,gps,") + String(current_lat, 14) + "," + String(current_lng, 14) + String("\n");
              msg += String("azimuth,direction,") + String(direction, 5) + String("\n");
              msg += String("azimuth,distance,") + String(distance, 5) + String("\n");
              msg += "transition,ChangeDirection";
              msg.toCharArray(log_buf, 300);
              Serial.println(msg);
              mysd.appendLog(log_buf);
              state = ChangeDirection;
              break;
          }
        }
      }
      break;

    case ChangeDirection:
      if (mpu.update()) {
        double yaw = mpu.getYaw();
        mpuAppendLog(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(), mpu.getMagX(), mpu.getMagY(), mpu.getMagZ(), yaw);
        if (90 < direction && direction < 270) {
          if (yaw < 0) { yaw = yaw + 360; }
        }
        yaw_sum += yaw;
        yaw_count++;
        if (yaw_count >= 20) {
          current_yaw = (yaw_sum / yaw_count) + 180.0;
          double original_yaw = (yaw_sum / yaw_count) > 180 ? (yaw_sum / yaw_count) - 360 : (yaw_sum / yaw_count);
          if (current_yaw > 360) { current_yaw = current_yaw - 360; }
          if (current_yaw < 0) { current_yaw = current_yaw + 360; }
          double diff = current_yaw - direction;
          if (diff > 180) {
            diff = diff - 360;
          } else if (diff < -180) {
            diff = diff + 360;
          }
          yaw_sum = 0;
          yaw_count = 0;
          Serial.println(String("ChangeDirection,") + String(current_yaw, 5) + "," + String(direction, 5) + "," + String(diff, 5));
          if (abs(diff) < 3) {
            Serial.println("Finish changing direction");
            cd_state = CD_Start;
            start_time = millis();
            moveMoter(245, 245);
            state = MoveTo;
            mysd.appendLog("transition,MoveTo");
          } else if (diff < 0) {
            if (cd_state == CD_Start || cd_state == CD_Left) {
              cd_state = CD_Right;
              moveMoter(80, 0);
            }
          } else {
            if (cd_state == CD_Start || cd_state == CD_Right) {
              cd_state = CD_Left;
              moveMoter(0,80);
            }
          }

          change_direct_log += String("control,current_yaw,") + String(current_yaw, 4);
          change_direct_log.toCharArray(change_direct_log_buf, 4096);
          mysd.appendLog(change_direct_log_buf);
          change_direct_log = "";
        }
      }
      break;

    case MoveTo:
      if(mpu.update()) {
        mpu.getYaw();
      }
      switch (distance_state) {
        case LessThan1m:
          // never happens
          state = End;
          break;
        case Between1mAnd5m:
          if (millis() - start_time > 3 * 1000) {
            moveMoter(0, 0);
            state = GetGPS;
            mysd.appendLog("transition,GetGPS");
          }
          break;
        case GreaterThan5m:
          if (millis() - start_time > 10 * 1000) {
            moveMoter(0, 0);
            state = GetGPS;
            mysd.appendLog("transition,GetGPS");
          }
          break;
      }
      break;

    case End:
      // if (mpu.update()) {
      //   float yaw = mpu.getYaw();
      //   mpuAppendLog(mpu.getAccX(), mpu.getAccY(), mpu.getAccZ(), mpu.getGyroX(), mpu.getGyroY(), mpu.getGyroZ(), mpu.getMagX(), mpu.getMagY(), mpu.getMagZ(), yaw);
      //   current_yaw = yaw + 180;
      //   Serial.println(current_yaw);
      // }
      if (gps_serial.available() > 0) {
        char c = gps_serial.read();
        if(gps.getValues(c, &current_lng, &current_lat)) {
          String msg = String("sensor,gps,") + String(current_lat, 14) + "," + String(current_lng, 14);
          msg.toCharArray(log_buf, 300);
          mysd.appendLog(log_buf);
          Serial.println(msg);
        }
      }
      break;
  }
}
