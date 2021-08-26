#include "MPU9250.h"
#include "sd_lib.hpp"

MPU9250 mpu;
MySD mysd = MySD();

const int motorA[3] = {4, 13, 25};  // AIN1, AIN2, PWMA
const int motorB[3] = {14, 27, 26}; // BIN1, BIN2, PWMB
const int CHANNEL_A = 0;
const int CHANNEL_B = 1;
const int LEDC_TIMER_BIT = 8;
const int LEDC_BASE_FREQ = 490;


int count = 0;
double magX = 0;
double magY = 0;
double magZ = 0;

unsigned long start_time = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mysd.createDir(SD, "/mag_log");
  mysd.writeFile(SD, "/mag_log/2021-08-26.log", "");
  delay(2000);
  Serial.println("start program");

  if (!mpu.setup(0x68)) {
    while (true) {
      Serial.println("MPU connection failed. Please restart 100kinsat.");
    }
  }

  // set the calibration values.

  // == EXAMPLE (You need to calcurate these values on your device) ==
  //  < calibration parameters >
  //  accel bias [g]: 
  //  -312.82, 9.55, -16.91
  //  gyro bias [deg/s]: 
  //  -0.34, 1.07, 0.42
  //  mag bias [mG]: ßß
  //  -427.70, 275.98, 303.75ß


  mpu.setAccBias(-312.82, 9.55, -16.91);
  mpu.setGyroBias(-0.34, 1.07, 0.42);
  mpu.setMagBias(-427.70, 275.98, 303.75);
  mpu.setMagScale(1,1,1);
  
  mpu.selectFilter(QuatFilterSel::MADGWICK);

  //set up mortors
  for (int i = 0; i < 3; i++) {
    pinMode(motorA[i], OUTPUT);
    pinMode(motorB[i], OUTPUT);
  }

  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ, LEDC_TIMER_BIT);

  ledcAttachPin(motorA[2], CHANNEL_A);
  ledcAttachPin(motorB[2], CHANNEL_B);

  start_time = millis();


  digitalWrite(motorA[0], LOW);
  digitalWrite(motorA[1], HIGH);
  ledcWrite(CHANNEL_A, 100);
  digitalWrite(motorB[0], HIGH);
  digitalWrite(motorB[1], LOW);
  ledcWrite(CHANNEL_B, 40);

}

void loop() {
  if (mpu.update()) {
    magX += mpu.getMagX();
    magY += mpu.getMagY();
    magZ += mpu.getMagZ();
    count++;
    if (count > 100) {
      String msg = "";
      msg += magX / count;
      msg += ",";
      msg += magY / count;
      msg += ",";
      msg += magZ / count;
      Serial.println(msg);
      char buf[50];
      (msg + "\n").toCharArray(buf, 50);
      mysd.appendFile(SD, "/mag_log/2021-08-26.log", buf);
      magX = 0; magY = 0; magZ = 0;
      count = 0;
    }
  }

  if (millis() - start_time > 90 * 1000) {
    digitalWrite(motorA[0], LOW);
    digitalWrite(motorA[1], HIGH);
    ledcWrite(CHANNEL_A, 40);
    digitalWrite(motorB[0], HIGH);
    digitalWrite(motorB[1], LOW);
    ledcWrite(CHANNEL_B, 100);
  }
}
