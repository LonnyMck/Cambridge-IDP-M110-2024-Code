/*
Test for motors in chassis
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor = AFMS.getMotor(2);
// You can also make another motor on port M2
//Adafruit_DCMotor *Motor2 = AFMS.getMotor(3);


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor func test");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Motor Shield found.");

}

void loop() {
  uint8_t i;

  void runMotor(150,FORWARD, Motor, 0)


}




void runMotor(int speed, char direction, Adafruit_DCMotor motorObject, int duration){ // direction should be FORWARD, BACKWARD, or STOP. Speed should be an int between 0 and 255.

  if (direction == "FORWARD"){
    motorObject->setSpeed(speed)
    motorObject->run(FORWARD)
    Serial.print("Moving forward at \n");
  } 

  else if (direction == "BACKWARD"){
    motorObject->setSpeed(speed)
    motorObject->run(BACKWARD)
    Serial.print("Moving backward at \n");
  }

  else if (direction == "STOP"){
    motorObject->run(RELEASE);
    Serial.print("Stopping \n");
  }

  else {
    Serial.println("Incorrect direction argument received");
  }

  motorObject->run(RELEASE);


}