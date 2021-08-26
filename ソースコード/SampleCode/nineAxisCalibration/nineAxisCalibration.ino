#include "MPU9250.h"

MPU9250 mpu;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(2000);
  Serial.println("start");

  // initialize nine axis sensor
  if (!mpu.setup(0x68)) {
    while(true) {
      Serial.println("MPU connection failed. Please restart 100kinsat.");
    }
  }

  // Calibration of Accel and Gyro
  Serial.println("Start calibration of Accel and Gyro in 3 sec.");
  Serial.println("Please leave 100kinsat stable.");
  mpu.verbose(true);
  delay(3000);
  mpu.calibrateAccelGyro();
  Serial.println("Finish calibration of Accel and Gyro");

  // Calibration of Mag()
  Serial.println("Start calibration of Mag in 3sec.");
  Serial.println("Please turn 100kinsat into a figure eight");
  delay(3000);
  mpu.calibrateMag();
  mpu.verbose(false);

  // output the result of calibration
  print_calibration();
}

void loop() {
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}
