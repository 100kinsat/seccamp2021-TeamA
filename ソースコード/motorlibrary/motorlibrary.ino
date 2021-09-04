#include "motor.hpp"

Motor motor = Motor();

void setup() {
}

void loop() {
  motor.forward(100);
  delay(1000);

//  motor.stop();
//  delay(1000);

  motor.tiltr(100, 50);
  delay(1000);

//  motor.stop();
//  delay(1000);

  motor.tiltl(100, 50);
  delay(1000);
  
//  motor.stop();
//  delay(1000);
}
