#include "MPU9250.h"
#include "sd_lib.hpp"
#include "cansat_gps.hpp"
#include "azimuthLibrary.hpp"
#include "yaw_offset.hpp"
#include <stdlib.h>

MPU9250 mpu;
MySD mysd = MySD(); // SDカードへの書き込み処理を行う
Cansat_gps gps = Cansat_gps(); // GPSの値を読み込む
Azimuth azimuth = Azimuth(); // 方位角の計算を行う
YawOffset yawOffset = YawOffset(); // yawの補正値を計算する

HardwareSerial gps_serial(2); // GPSの通信

// 目的地の座標
// 公園の角 43.018823502674586, 141.4370544820975
double goal_lat = 43.018823502674586;
double goal_lng = 141.4370544820975;

const char* LOG_FILE_NAME = "/mag_log/2021-09-02-8.log";

const int motorA[3] = {4, 13, 25};  // AIN1, AIN2, PWMA
const int motorB[3] = {14, 27, 26}; // BIN1, BIN2, PWMB
const int CHANNEL_A = 0;
const int CHANNEL_B = 1;
const int LEDC_TIMER_BIT = 8;
const int LEDC_BASE_FREQ = 490;

double current_yaw = 0;
double yaw_offset = 0;
double direction = 0;
unsigned long start_time = 0;
double current_lat = 0;
double current_lng = 0;
char log_buf[300];

// 100kinsatの状態の定義
enum State {
  Start,
  CalcYawOffset,
  YawOffsetSet,
  GetGPS,
  ChangeDirection,
  MoveTo,
  ReadyStop,
  End
};
State state = Start;

// mpuのyawの値を100kinsatの前方向に対して北から右回りで0 ~ 360の値にする
// double current_yaw() {
//   double ret = mpu.getYaw();
//   return ret + 180;
// }

void setup() {
  Wire.begin();
  Serial.begin(115200);
  gps_serial.begin(9600);
  delay(4000);
  mysd.createDir(SD, "/mag_log");
  mysd.writeFile(SD, LOG_FILE_NAME, "");

  Serial.println("start program");

  if (!mpu.setup(0x68)) {
    while (true) {
      Serial.println("MPU connection failed. Please restart 100kinsat.");
    }
  }

  // set the calibration values.

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

  mpu.setAccBias(-309.97, 12.99, -20.75);
  mpu.setGyroBias(-0.15, 1.17, 0.39);
  mpu.setMagBias(-500.46, 370.35, 425.59);
  mpu.setMagScale(1,1,1);
  mpu.selectFilter(QuatFilterSel::MADGWICK);

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

  digitalWrite(motorA[0], LOW);
  digitalWrite(motorA[1], HIGH);
  ledcWrite(CHANNEL_A, 80);
  digitalWrite(motorB[0], HIGH);
  digitalWrite(motorB[1], LOW);
  ledcWrite(CHANNEL_B, 0);
}

void loop() {
  switch(state) {
    case Start:
      state = GetGPS;
      break;
    case CalcYawOffset:
      if (mpu.update()) {
        yawOffset.update(mpu.getMagY(), mpu.getYaw());
      }
      if (millis() - start_time > 30 * 1000) {
        state = YawOffsetSet;
      }
      break;
    case YawOffsetSet:
      // 典型的には -5 とかの値が出る
      yaw_offset = yawOffset.getOffset();
      char buf[50];
      (String("yaw offset = ") + yaw_offset + "\n").toCharArray(buf, 50);
      mysd.appendFile(SD, LOG_FILE_NAME, buf);
      digitalWrite(motorA[0], LOW);
      digitalWrite(motorA[1], LOW);
      ledcWrite(CHANNEL_A, 0);
      digitalWrite(motorB[0], LOW);
      digitalWrite(motorB[1], LOW);
      ledcWrite(CHANNEL_B, 0);
      state = GetGPS;
      break;
    case GetGPS:
      if (mpu.update()) {
        current_yaw = mpu.getYaw() + 180;
      }
      if (gps_serial.available() > 0) {
        char c = gps_serial.read();
        if(gps.getValues(c, &current_lng, &current_lat)) {
          azimuth.azimuth_to_goal(current_lng, current_lat, &direction);
          String msg = String("current position:,") + String(current_lat, 14) + "," + String(current_lng, 14);
          msg.toCharArray(log_buf, 300);
          Serial.println(msg);
          mysd.appendFile(SD, LOG_FILE_NAME, log_buf);
          Serial.println(String("direction: ") + direction);

          digitalWrite(motorA[0], LOW);
          digitalWrite(motorA[1], HIGH);
          ledcWrite(CHANNEL_A, 80);
          digitalWrite(motorB[0], HIGH);
          digitalWrite(motorB[1], LOW);
          ledcWrite(CHANNEL_B, 0);
          state = ChangeDirection;
        }
      }
      break;
    case ChangeDirection:
      if (mpu.update()) {
        current_yaw = mpu.getYaw() + 180;
        double diff = current_yaw - direction;
        if (diff > 180) {
          diff = diff - 360;
        } else if (diff < -180) {
          diff = diff + 360;
        }
        Serial.println(String(current_yaw, 5) + "," + String(direction, 5) + "," + String(diff, 5));
        if (abs(diff) < 3) {
          Serial.println("Finish changing direction");
          start_time = millis();
          digitalWrite(motorA[0], LOW);
          digitalWrite(motorA[1], HIGH);
          ledcWrite(CHANNEL_A, 180);
          digitalWrite(motorB[0], HIGH);
          digitalWrite(motorB[1], LOW);
          ledcWrite(CHANNEL_B, 180);
          state = MoveTo;
        } else if (diff < 0) {
          digitalWrite(motorA[0], LOW);
          digitalWrite(motorA[1], HIGH);
          ledcWrite(CHANNEL_A, 80);
          digitalWrite(motorB[0], HIGH);
          digitalWrite(motorB[1], LOW);
          ledcWrite(CHANNEL_B, 0);
        } else {
          digitalWrite(motorA[0], LOW);
          digitalWrite(motorA[1], HIGH);
          ledcWrite(CHANNEL_A, 0);
          digitalWrite(motorB[0], HIGH);
          digitalWrite(motorB[1], LOW);
          ledcWrite(CHANNEL_B, 80);
        }
      }
      break;
    case MoveTo:
      if (millis() - start_time > 10 * 1000) {
        digitalWrite(motorA[0], LOW);
        digitalWrite(motorA[1], HIGH);
        ledcWrite(CHANNEL_A, 0);
        digitalWrite(motorB[0], HIGH);
        digitalWrite(motorB[1], LOW);
        ledcWrite(CHANNEL_B, 80);
        state = End;
      }
      break;
    case End:
      if (mpu.update()) {
        current_yaw = mpu.getYaw() + 180;
        Serial.println(current_yaw);
      }
      digitalWrite(motorA[0], LOW);
      digitalWrite(motorA[1], LOW);
      ledcWrite(CHANNEL_A, 0);
      digitalWrite(motorB[0], LOW);
      digitalWrite(motorB[1], LOW);
      ledcWrite(CHANNEL_B, 0);
      // do nothing
      break;
  }


}
