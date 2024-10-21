#include <Adafruit_MotorShield.h>
#include <Servo.h>

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
Adafruit_DCMotor *Motor = AFMS.getMotor(3);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);
Servo controlservo; // Servo for turning or controlling something else

bool running = true;
char input;

double speed;
double time;

// Setup function
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor and sensor func test");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Motor Shield found.");

  controlservo.attach(2); // attaches the servo on pin 2 to the servo object
  runServo(0); // Check that the servo is homed

  // Initialize sensors as inputs
  pinMode(sensorPinLL, INPUT);
  pinMode(sensorPinL, INPUT);
  pinMode(sensorPinR, INPUT);
  pinMode(sensorPinRR, INPUT);
}

// Main loop
void loop() {
  int decision = MakeDecision();  // Get decision from sensors

  // Execute action based on the decision
  if (decision == 1) {
    stop(); // Stop at a cross
  } else if (decision == 2) {
    turnRight(90); // Turn right
  } else if (decision == 3) {
    turnLeft(90); // Turn left
  } else if (decision == 4) {
    runForwards(1000); // Move forward for 1 second
  } else if (decision == 5) {
    stop(); // Stop if lost
  } else {
    stop(); // Default action is to stop
  }

  delay(1000); // Delay before next sensor reading
}

// Sensor decision-making functions

bool DetectCross() {
  sensorStateLL = digitalRead(sensorPinLL);
  sensorStateL = digitalRead(sensorPinL);
  sensorStateR = digitalRead(sensorPinR);
  sensorStateRR = digitalRead(sensorPinRR);

  if ((sensorStateLL == HIGH) && (sensorStateL == HIGH) && (sensorStateR == HIGH) && (sensorStateRR == HIGH)) {
    return true;
  } else {
    return false;
  }
}

bool DetectRightTurn() {
  sensorStateRR = digitalRead(sensorPinRR);
  sensorStateL = digitalRead(sensorPinL);
  sensorStateR = digitalRead(sensorPinR);

  if ((sensorStateL == HIGH) && (sensorStateR == HIGH) && (sensorStateRR == HIGH)) {
    return true;
  } else {
    return false;
  }
}

bool DetectLeftTurn() {
  sensorStateLL = digitalRead(sensorPinLL);
  sensorStateL = digitalRead(sensorPinL);
  sensorStateR = digitalRead(sensorPinR);

  if ((sensorStateLL == HIGH) && (sensorStateL == HIGH) && (sensorStateR == HIGH)) {
    return true;
  } else {
    return false;
  }
}

bool IAmOnLine() {
  sensorStateLL = digitalRead(sensorPinLL);
  sensorStateL = digitalRead(sensorPinL);
  sensorStateR = digitalRead(sensorPinR);
  sensorStateRR = digitalRead(sensorPinRR);

  if ((sensorStateLL == LOW) && (sensorStateL == HIGH) && (sensorStateR == HIGH) && (sensorStateRR == LOW)) {
    return true;
  } else {
    return false;
  }
}

bool IAmLost() {
  sensorStateLL = digitalRead(sensorPinLL);
  sensorStateL = digitalRead(sensorPinL);
  sensorStateR = digitalRead(sensorPinR);
  sensorStateRR = digitalRead(sensorPinRR);

  if ((sensorStateLL == LOW) && (sensorStateL == LOW) && (sensorStateR == LOW) && (sensorStateRR == LOW)) {
    return true;
  } else {
    return false;
  }
}

int MakeDecision() {
  if (DetectCross()) {
    return 1;
  }

  if (DetectRightTurn()) {
    return 2;
  }

  if (DetectLeftTurn()) {
    return 3;
  }

  if (IAmOnLine()) {
    return 4;
  }

  if (IAmLost()) {
    return 5;
  }

  return "Undetermined state";
}

// Motor control functions

int runForwards(int time) {
  speed = 100;  // Set the forward speed
  runMotor(speed, 1, Motor);
  runMotor(speed, 1, Motor2);

  if (time != 0) {
    delay(time);
    stop();
  }
}

int runBackwards(int time) {
  speed = 100;  // Set the backward speed
  runMotor(speed, 0, Motor);
  runMotor(speed, 0, Motor2);

  if (time != 0) {
    delay(time);
    stop();
  }
}

int stop() {
  runMotor(0, 1, Motor);
  runMotor(0, 1, Motor2);
}

int turnRight(int degrees) {
  time = 500;  // You will need to calibrate this time

  runMotor(100, 1, Motor);  // Forward for Motor1
  runMotor(50, 0, Motor2);  // Backward for Motor2
  delay(time);
  stop();
}

int turnLeft(int degrees) {
  time = 500;  // You will need to calibrate this time

  runMotor(50, 0, Motor);  // Backward for Motor1
  runMotor(100, 1, Motor2);  // Forward for Motor2
  delay(time);
  stop();
}

int runMotor(int speed, bool direction, Adafruit_DCMotor *motorObject) {
  if (speed == 0) {
    motorObject->run(RELEASE);
    Serial.println("Stopping motor");
  } else if (direction == 1) {
    motorObject->setSpeed(speed);
    motorObject->run(FORWARD);
    Serial.println("Moving forward at speed " + String(speed));
  } else if (direction == 0) {
    motorObject->setSpeed(speed);
    motorObject->run(BACKWARD);
    Serial.println("Moving backward at speed " + String(speed));
  } else {
    Serial.println("Incorrect direction argument received");
  }
}

int runServo(int newpos) {
  controlservo.write(newpos);
  delay(2000);
}

int check_interrupt() {
  if (Serial.available()) {
    input = Serial.read();
  }

  if (input == 't') {
    Serial.println("Interrupt");
    stop();
    runServo(0);
    while (1);
  }
  return true;
}
