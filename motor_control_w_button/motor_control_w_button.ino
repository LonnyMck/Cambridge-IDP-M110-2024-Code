/*
THIS ISN"T ANYWHERE CLOSE TO DONE
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor = AFMS.getMotor(2);
// You can also make another motor on port M2
//Adafruit_DCMotor *Motor2 = AFMS.getMotor(3);


#define PIN_BUTTON                     2 // Button pin


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

  runMotor(150, 1, Motor);
  delay(5000);
  runMotor(0, 1, Motor);
  delay(5000);

}



///////////////////////////
//Declaring functions
///////////////////////////



int runMotor(int speed, bool direction, Adafruit_DCMotor *motorObject){ // direction should be 1 for Forward, 0 for Backward. Speed should be an int between 0 and 255.
  
  Serial.println("Running Motor.");

  if (speed == 0){
    motorObject->run(RELEASE);
    Serial.println("Stopping");
  }

  else if (direction == 1){
    motorObject->setSpeed(speed);
    motorObject->run(FORWARD);
    Serial.println("Moving forward at " + String(speed));
  } 

  else if (direction == 0){
    motorObject->setSpeed(speed);
    motorObject->run(BACKWARD);
    Serial.println("Moving backward at " + String(speed));
  }


  else {
    Serial.println("Incorrect direction argument received");
  }




}


