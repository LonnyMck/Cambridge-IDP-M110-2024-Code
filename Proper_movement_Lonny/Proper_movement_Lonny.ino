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
Adafruit_DCMotor *Motor = AFMS.getMotor(3);   // Left motor
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);  // Right motor
Servo controlservo;                           // Servo for turning or controlling something else

unsigned long time_since_line;
enum line_follow_state { VEER_LEFT,
                         VEER_RIGHT,
                         FORWARD_MOVE } last_line;

bool running = false;  // Start with the robot not running

double speed;
double time;

// Button pins
const int greenButtonPin = 8;  // Pin connected to the green button
const int redButtonPin = 9;    // Pin connected to the red button

// Button states
int greenButtonState = 0;
int redButtonState = 0;

// Setup function
void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  Serial.println("Motor and sensor func test");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }

  Serial.println("Motor Shield found.");

  controlservo.attach(2);  // attaches the servo on pin 2 to the servo object
  runServo(0);             // Check that the servo is homed

  // Initialize sensors as inputs
  pinMode(sensorPinLL, INPUT);
  pinMode(sensorPinL, INPUT);
  pinMode(sensorPinR, INPUT);
  pinMode(sensorPinRR, INPUT);

  // Initialize button pins as inputs
  pinMode(greenButtonPin, INPUT);
  pinMode(redButtonPin, INPUT);
}

// Main loop
void loop() {
  if (!running) {
    // Wait for the green button to be pressed
    greenButtonState = digitalRead(greenButtonPin);
    if (greenButtonState == HIGH) {
      running = true;
      Serial.println("Green button pressed, starting operation.");
    } else {
      // Do nothing until the green button is pressed
      return;
    }
  }

  // If running is true, proceed with main code
  // Continuously perform line following
  if (running) {
    // Check if the red button is pressed to interrupt
    redButtonState = digitalRead(redButtonPin);
    if (redButtonState == HIGH) {
      running = false;
      stopMotors();
      Serial.println("Red button pressed, stopping operation.");
      return;
    }

    // Perform line following
    lineFollow();

    delay(10);  // Small delay before next loop iteration
  }
}

// Line following function
void lineFollow() {
  // Read the sensor states
  sensorStateLL = digitalRead(sensorPinLL);
  sensorStateL = digitalRead(sensorPinL);
  sensorStateR = digitalRead(sensorPinR);
  sensorStateRR = digitalRead(sensorPinRR);

  // Check for special conditions (cross, turns, lost)
  if (DetectCross()) {
    Serial.println("Cross detected.");
    stopMotors();
    delay(1000);  // Pause at the cross
  } else if (DetectRightTurn()) {
    Serial.println("Right turn detected.");
    turnRight();
  } else if (DetectLeftTurn()) {
    Serial.println("Left turn detected.");
    turnLeft();
  } else if (IAmLost()) {
    // Serial.println("Lost. Stopping motors.");
    stopMotors();
  } else {
    // Adjust motor speeds based on sensor readings
    if (sensorStateL == HIGH && sensorStateR == HIGH) {
      // Both central sensors ON, go straight
      runMotor(200, 1, Motor);   // Left motor
      runMotor(200, 1, Motor2);  // Right motor
      Serial.println("Moving forward.");
    } else if (sensorStateL == HIGH && sensorStateR == LOW) {
      veerRight();
      time_since_line = millis();

    } else if (sensorStateL == LOW && sensorStateR == HIGH) {
      veerLeft();
      time_since_line = millis();

    } else if (sensorStateLL == HIGH) {
      // Far left sensor ON, need to turn sharply right
      runMotor(80, 1, Motor);    // Left motor slower
      runMotor(190, 1, Motor2);  // Right motor faster
      Serial.println("Sharp adjust right.");
    } else if (sensorStateRR == HIGH) {
      // Far right sensor ON, need to turn sharply left
      runMotor(120, 1, Motor);  // Left motor faster
      runMotor(60, 1, Motor2);  // Right motor slower
      Serial.println("Sharp adjust left.");
    } else {
      // All sensors OFF, stop or take appropriate action
      if (time_since_line - millis() > 5000) {
        stopMotors();
        Serial.println("Line lost. Stopping motors.");
        
        time_since_line = millis();
        Serial.println("too much time");
        
        return;
      }
      if (last_line == VEER_RIGHT) {
        veerRight();
        Serial.println("Still veering");
      }
      if (last_line == VEER_LEFT) {
        veerLeft();
        Serial.println("Still veering");
      }
    }
  }
}

// Sensor decision-making functions

bool DetectCross() {
  // Cross detected when all sensors are ON
  return (sensorStateLL == HIGH && sensorStateL == HIGH && sensorStateR == HIGH && sensorStateRR == HIGH);
}

bool DetectRightTurn() {
  // Right turn detected when L, R, and RR are ON
  return (sensorStateL == HIGH && sensorStateR == HIGH && sensorStateRR == HIGH && sensorStateLL == LOW);
}

bool DetectLeftTurn() {
  // Left turn detected when LL, L, and R are ON
  return (sensorStateLL == HIGH && sensorStateL == HIGH && sensorStateR == HIGH && sensorStateRR == LOW);
}

bool IAmLost() {
  // Robot is lost when all sensors are OFF
  return (sensorStateLL == LOW && sensorStateL == LOW && sensorStateR == LOW && sensorStateRR == LOW);
}

// Motor control functions

void runForwards(int time) {
  speed = 100;  // Set the forward speed
  runMotor(speed, 1, Motor);
  runMotor(speed, 1, Motor2);

  if (time != 0) {
    delay(time);
    stopMotors();
  }
}

void runBackwards(int time) {
  speed = 100;  // Set the backward speed
  runMotor(speed, 0, Motor);
  runMotor(speed, 0, Motor2);

  if (time != 0) {
    delay(time);
    stopMotors();
  }
}

void stopMotors() {
  runMotor(0, 1, Motor);
  runMotor(0, 1, Motor2);
}

void turnRight() {
  Serial.println("Executing right turn...");
  // Initialize variable to detect the transition
  bool sensorRRWasBlack = false;

  // Start turning: Left motor forward, Right motor backward at same speed
  runMotor(100, 1, Motor);   // Left motor forward
  runMotor(100, 0, Motor2);  // Right motor backward

  while (true) {
    // Read the RR sensor
    sensorStateRR = digitalRead(sensorPinRR);

    // Check if red button is pressed to interrupt
    redButtonState = digitalRead(redButtonPin);
    if (redButtonState == HIGH) {
      running = false;
      stopMotors();
      Serial.println("Red button pressed during turn, stopping operation.");
      return;
    }

    if (sensorStateRR == HIGH) {
      // RR sensor detects black line
      sensorRRWasBlack = true;
      Serial.println("RR sensor detected black line.");
    }

    if (sensorRRWasBlack && sensorStateRR == LOW) {
      // RR sensor transitions from black to white
      Serial.println("RR sensor transitioned from black to white.");
      break;  // Exit the loop
    }
  }

  // Stop the motors after the turn is complete
  stopMotors();
  Serial.println("Right turn completed.");
}

void turnLeft() {
  Serial.println("Executing left turn...");
  // Initialize variable to detect the transition
  bool sensorLLWasBlack = false;

  // Start turning: Left motor backward, Right motor forward at same speed
  runMotor(100, 0, Motor);   // Left motor backward
  runMotor(100, 1, Motor2);  // Right motor forward

  while (true) {
    // Read the LL sensor
    sensorStateLL = digitalRead(sensorPinLL);

    // Check if red button is pressed to interrupt
    redButtonState = digitalRead(redButtonPin);
    if (redButtonState == HIGH) {
      running = false;
      stopMotors();
      Serial.println("Red button pressed during turn, stopping operation.");
      return;
    }

    if (sensorStateLL == HIGH) {
      // LL sensor detects black line
      sensorLLWasBlack = true;
      Serial.println("LL sensor detected black line.");
    }

    if (sensorLLWasBlack && sensorStateLL == LOW) {
      // LL sensor transitions from black to white
      Serial.println("LL sensor transitioned from black to white.");
      break;  // Exit the loop
    }
  }



  // Stop the motors after the turn is complete
  stopMotors();
  Serial.println("Left turn completed.");
}


//veer left

void veerLeft() {
  // Left sensor OFF, right sensor ON -> turn slightly left
  runMotor(80, 1, Motor);    // Left motor faster
  runMotor(190, 1, Motor2);  // Right motor slower
  Serial.println("Adjusting left.");
  last_line = VEER_LEFT;
}

void veerRight() {
  // Left sensor ON, right sensor OFF -> turn slightly right
  runMotor(190, 1, Motor);  // Left motor slower
  runMotor(80, 1, Motor2);  // Right motor faster
  Serial.println("Adjusting right.");
  last_line = VEER_RIGHT;
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

void runServo(int newpos) {
  controlservo.write(newpos);
  delay(2000);
}
