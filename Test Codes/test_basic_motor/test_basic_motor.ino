
/*
Test for motors in chassis
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor1 = AFMS.getMotor(2);
// You can also make another motor on port M2
Adafruit_DCMotor *Motor2 = AFMS.getMotor(3);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  Motor1->setSpeed(150);
  Motor2->setSpeed(150);

  Motor1->run(FORWARD);
  Motor2->run(FORWARD);

  // turn on motor
  Motor1->run(RELEASE);
  Motor2->run(RELEASE);
}

void loop() {
  uint8_t i;

  Serial.print("moving forward");

  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
  for (i=0; i<255; i++) {
    Motor1->setSpeed(i);
    Motor2->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    Motor1->setSpeed(i);
    Motor2->setSpeed(i);
    delay(10);
  }

  Serial.print("moving backward");

  Motor1->run(BACKWARD);
  Motor2->run(BACKWARD);
  for (i=0; i<255; i++) {
    Motor1->setSpeed(i);
    Motor2->setSpeed(i);
    delay(10);
  }
  for (i=255; i!=0; i--) {
    Motor1->setSpeed(i);
    Motor2->setSpeed(i);
    delay(10);
  }

  Serial.print("tech");
  Motor1->run(RELEASE);
  Motor2->run(RELEASE);
  delay(1000);
}