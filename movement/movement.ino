#include <Adafruit_MotorShield.h>

// Define sensor pins
const int sensorPinLL = 13;
const int sensorPinL = 12;
const int sensorPinR = 11;
const int sensorPinRR = 10;
const int sensorPinB = 2;

// Motor and servo control
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorR = AFMS.getMotor(3);   // Right motor
Adafruit_DCMotor *MotorL = AFMS.getMotor(2);   // Left motor

bool isStopped = false;         // Flag to track if the robot is stopped
bool hasTurned = false;         // Flag to ensure the turn happens only once
bool lineFollowExecuted = false; // Flag to check if lineFollow has run at least once

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
  pinMode(sensorPinB, INPUT);
}

void loop() {
  // Only run lineFollow if the robot is not stopped
  if (!isStopped) {
    lineFollow();
    delay(300);
  }
  // Run makeTurnR once after stopping, but only if lineFollow has run
  else if (lineFollowExecuted && !hasTurned) {
    makeTurnR();
    hasTurned = true; // Ensure makeTurnR is called only once
  }
}

void lineFollow() {

  int speed = 200;

  bool LL = digitalRead(sensorPinLL);
  bool L = digitalRead(sensorPinL);
  bool R = digitalRead(sensorPinR);
  bool RR = digitalRead(sensorPinRR);
  bool B = digitalRead(sensorPinB);
  

  // Check conditions with binary literals
  if (L==HIGH && B==HIGH && R==HIGH) {
    runMotor(speed, 1, MotorR);
    runMotor(speed, 1, MotorL);
    lineFollowExecuted = true;
  }
  else if (L==HIGH && B==HIGH && R==LOW) {
    runMotor(speed + 50, 1, MotorR);
    runMotor(speed, 1, MotorL);
    lineFollowExecuted = true;
  }
  else if (L==LOW && B==HIGH && R==HIGH) {
    runMotor(speed, 1, MotorR);
    runMotor(speed + 50, 1, MotorL);
    lineFollowExecuted = true;
  }
  else if (L==HIGH && B==LOW && R==LOW) {
    runMotor(speed + 100, 1, MotorR);
    runMotor(speed, 1, MotorL);
    lineFollowExecuted = true;
  }
  else if (L==LOW && B==LOW && R==HIGH) {
    runMotor(speed, 1, MotorR);
    runMotor(speed + 100, 1, MotorL);
    lineFollowExecuted = true;
  }
  if (RR==HIGH || LL==HIGH) {
    runMotor(0, 1, MotorR);
    runMotor(0, 1, MotorL);
    isStopped = true; // Set the stop flag to true
  }
}

void runMotor(int speed, bool direction, Adafruit_DCMotor *motorObject) {
  if (speed == 0) {
    motorObject->run(RELEASE);
  } else if (direction == 1) {
    motorObject->setSpeed(speed);
    motorObject->run(FORWARD);
  } else if (direction == 0) {
    motorObject->setSpeed(speed);
    motorObject->run(BACKWARD);
  } else {
    Serial.println("Incorrect direction argument received");
  }
}

void makeTurnR() {
  // Stop both motors initially
  runMotor(0, 0, MotorL);
  runMotor(0, 0, MotorR);
  delay(500);

  runMotor(200, 0, MotorL);
  runMotor(200, 0, MotorR);
  delay(500);


  // Begin turning right
  runMotor(200, 1, MotorL); // Left motor runs backward for right turn
  runMotor(0, 1, MotorR);   // Right motor stopped

  delay(3000);

  // Continue turning until sensorPinL goes HIGH again, aligning with the line
  while (digitalRead(sensorPinL) == LOW) {
    runMotor(200, 1, MotorL);
    runMotor(0, 1, MotorR);
  }

  // Stop both motors after alignment
  runMotor(0, 0, MotorL);
  runMotor(0, 0, MotorR);
  
  // Reset flags to resume line following
  isStopped = false;
  hasTurned = false;
  lineFollowExecuted = false;
}




