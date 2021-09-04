#include "motor.hpp"

Motor::Motor() {
  for (int i = 0; i < 3; i++) {
    pinMode(motorA[i], OUTPUT);
    pinMode(motorB[i], OUTPUT);
  }

  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ, LEDC_TIMER_BIT);

  ledcAttachPin(motorA[2], CHANNEL_A);
  ledcAttachPin(motorB[2], CHANNEL_B);
}

/**
 * 前進
 */
void Motor::forward(int pwm) {
  // 左モータ（CCW，反時計回り）
  digitalWrite(motorA[1], LOW);
  digitalWrite(motorA[0], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CW，時計回り）
  digitalWrite(motorB[1], LOW);
  digitalWrite(motorB[0], HIGH);
  ledcWrite(CHANNEL_B, pwm-10);
}

/**
 * 後退
 */
void Motor::back(int pwm) {
  // 左モータ（CW，時計回り）
  digitalWrite(motorA[0], LOW);
  digitalWrite(motorA[1], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CCW，反時計回り）
  digitalWrite(motorB[0], LOW);
  digitalWrite(motorB[1], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

/**
 * 右修正
 */
void Motor::tiltr(int pwm, int pwnr) {
  // 左モータ（CW，時計回り）
  digitalWrite(motorA[1], LOW);
  digitalWrite(motorA[0], HIGH);
  ledcWrite(CHANNEL_A, pwm);

  // 右モータ（CCW，反時計回り）
  digitalWrite(motorB[1], LOW);
  digitalWrite(motorB[0], HIGH);
  ledcWrite(CHANNEL_B, pwm-pwnr);
}

/**
 * 左修正
 */
void Motor::tiltl(int pwm, int pwnl) {
  // 左モータ（CW，時計回り）
  digitalWrite(motorA[1], LOW);
  digitalWrite(motorA[0], HIGH);
  ledcWrite(CHANNEL_A, pwm-pwnl);

  // 右モータ（CCW，反時計回り）
  digitalWrite(motorB[1], LOW);
  digitalWrite(motorB[0], HIGH);
  ledcWrite(CHANNEL_B, pwm);
}

 

/**
 * 停止
 */
void Motor::stop() {
  // 左モータ停止
  digitalWrite(motorA[0], LOW);
  digitalWrite(motorA[1], LOW);
  ledcWrite(CHANNEL_A, HIGH);

  // 右モータ停止
  digitalWrite(motorB[0], LOW);
  digitalWrite(motorB[1], LOW);
  ledcWrite(CHANNEL_B, HIGH);
}
