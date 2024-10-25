
#include <Adafruit_MotorShield.h>


// Define sensor pins
const int sensorPinLL = 13;
const int sensorPinL = 12;
const int sensorPinR = 11;
const int sensorPinRR = 10;

// Sensor states
int sensorStateLL = 0;
int sensorStateL = 0;
int sensorStateR = 0;
int sensorStateRR = 0;

// Motor and servo control
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorR = AFMS.getMotor(2);   // Right motor
Adafruit_DCMotor *MotorL = AFMS.getMotor(3);  // Left motor

double speed;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor and sensor func test");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Motor Shield found.");

  // Initialize sensors as inputs
  pinMode(sensorPinLL, INPUT);
  pinMode(sensorPinL, INPUT);
  pinMode(sensorPinR, INPUT);
  pinMode(sensorPinRR, INPUT);

}

void loop() {
  
  AdjustR();
  delay(10000);

}


void AdjustR() {
  Serial.println("Adjusting to the right");
  runMotor(120, 1, MotorR);
  runMotor(100, 1, MotorL);
  delay(500);
  
  runMotor(100, 1, MotorR);
  runMotor(120, 1, MotorL);
  delay(500);

  stopMotors();
}

void runMotor(int speed, bool direction, Adafruit_DCMotor *motorObject) {
  if (speed == 0) {
    motorObject->run(RELEASE);
    // Serial.println("Stopping motor");
  } else if (direction == 1) {
    motorObject->setSpeed(speed);
    motorObject->run(FORWARD);
    // Serial.println("Moving forward at speed " + String(speed));
  } else if (direction == 0) {
    motorObject->setSpeed(speed);
    motorObject->run(BACKWARD);
    // Serial.println("Moving backward at speed " + String(speed));
  } else {
    Serial.println("Incorrect direction argument received");
  }
}

void stopMotors() {
  runMotor(0, 1, MotorR);
  runMotor(0, 1, MotorL);
}