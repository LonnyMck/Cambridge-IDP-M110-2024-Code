#include <Adafruit_MotorShield.h>

// Define sensor pins
const int sensorPinLL = 10;
const int sensorPinL = 11;
const int sensorPinR = 12;
const int sensorPinRR = 13;
const int sensorPinB = 2;

// Motor and servo control
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorR = AFMS.getMotor(2);   // Right motor
Adafruit_DCMotor *MotorL = AFMS.getMotor(3);   // Left motor

bool isStopped = false; // Flag to track if the robot is stopped
bool hasTurned = false; // Flag to ensure the turn happens only once

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
  // Run makeTurnR once after stopping
  else if (!hasTurned) {
    makeTurnR();
    hasTurned = true; // Ensure makeTurnR is called only once
  }
}

// Function to read sensors and convert readings to a 5-digit binary number
int getSensorBinary() {
  int binaryNumber = 0;

  // Read each sensor and shift the result to the correct position
  binaryNumber |= (digitalRead(sensorPinLL) << 4);  // LL is the leftmost bit (4th position)
  binaryNumber |= (digitalRead(sensorPinL) << 3);   // L is the 3rd position
  binaryNumber |= (digitalRead(sensorPinR) << 2);   // R is the 2nd position
  binaryNumber |= (digitalRead(sensorPinRR) << 1);  // RR is the 1st position
  binaryNumber |= digitalRead(sensorPinB);          // B is the rightmost bit (0th position)

  return binaryNumber;
}

void lineFollow() {
  int sensorNumber = getSensorBinary();
  int speed = 200;

  // Check conditions with binary literals
  if (sensorNumber == 0b01101) {
    runMotor(speed, 1, MotorR);
    runMotor(speed, 1, MotorL);
  }
  else if (sensorNumber == 0b01001) {
    runMotor(speed + 50, 1, MotorR);
    runMotor(speed, 1, MotorL);
  }
  else if (sensorNumber == 0b00101) {
    runMotor(speed, 1, MotorR);
    runMotor(speed + 50, 1, MotorL);
  }
  else if (sensorNumber == 0b01000) {
    runMotor(speed + 100, 1, MotorR);
    runMotor(speed, 1, MotorL);
  }
  else if (sensorNumber == 0b00100) {
    runMotor(speed, 1, MotorR);
    runMotor(speed + 100, 1, MotorL);
  }
  else if (sensorNumber == 0b01111 || sensorNumber == 0b11101 || sensorNumber == 0b11111 || sensorNumber == 0b00000) {
    runMotor(0, 1, MotorR);
    runMotor(0, 1, MotorL);
    isStopped = true; // Set the stop flag to true
  }
}

void runMotor(int speed, bool direction, Adafruit_DCMotor *motorObject) {
  if (speed == 0) {
    motorObject->run(RELEASE);
  } else if (direction == 0) {
    motorObject->setSpeed(speed);
    motorObject->run(FORWARD);
  } else if (direction == 1) {
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
  delay(1000);

  // Move forward
  runMotor(200, 0, MotorL);
  runMotor(200, 0, MotorR);
  delay(1000);

  // Turn right
  runMotor(200, 1, MotorL);
  runMotor(0, 1, MotorR);
  delay(3000);

  // Stop both motors after the turn
  runMotor(0, 0, MotorL);
  runMotor(0, 0, MotorR);
}










