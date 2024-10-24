
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
Adafruit_DCMotor *MotorR = AFMS.getMotor(3);   // Left motor
Adafruit_DCMotor *MotorL = AFMS.getMotor(2);  // Right motor

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
  
  lineFollow();

  delay(200);

}


void lineFollow() {

  sensorStateLL = digitalRead(sensorPinLL);
  sensorStateL = digitalRead(sensorPinL);
  sensorStateR = digitalRead(sensorPinR);
  sensorStateRR = digitalRead(sensorPinRR);

  int decision = MakeDecision(sensorStateLL , sensorStateL , sensorStateR , sensorStateRR);

  if (decision == 1) {
    stopMotors();
    Serial.println("Stopping at Cross!");
  }

  else if (decision == 2) {
    stopMotors();
    Serial.println("Stopping at Right Turn");
  }

  else if (decision == 3) {
    stopMotors();
    Serial.println("Stopping at Left Turn");
  }

  else if (decision == 4) {
    AdjustR();
    Serial.println("Adjusting to the Right");
  }

  else if (decision == 5) {
    AdjustL();
    Serial.println("Adjusting to the Left");
  }

  else if (decision == 6) {
    runForwards(1000);
    Serial.println("Going Forward");
  }

  else if (decision == 7) {
    stopMotors();
    Serial.println("I am lost:(");
  }
}


bool DetectCross(int sensorStateLL ,int sensorStateL ,int sensorStateR ,int sensorStateRR) {
  // Cross detected when all sensors are ON
  return (sensorStateLL == HIGH && sensorStateL == HIGH && sensorStateR == HIGH && sensorStateRR == HIGH);
}

bool DetectRightTurn(int sensorStateLL ,int sensorStateL ,int sensorStateR ,int sensorStateRR) {
  // Right turn detected when L, R, and RR are ON
  return (sensorStateL == HIGH && sensorStateR == HIGH && sensorStateRR == HIGH && sensorStateLL == LOW);
}

bool DetectLeftTurn(int sensorStateLL ,int sensorStateL ,int sensorStateR ,int sensorStateRR) {
  // Left turn detected when LL, L, and R are ON
  return (sensorStateLL == HIGH && sensorStateL == HIGH && sensorStateR == HIGH && sensorStateRR == LOW);
}

bool OffToTheLeft(int sensorStateLL ,int sensorStateL ,int sensorStateR ,int sensorStateRR) {
  // Left turn detected when LL, L, and R are ON
  return (sensorStateLL == LOW && sensorStateL == LOW && sensorStateR == HIGH && sensorStateRR == LOW);
}

bool OffToTheRight(int sensorStateLL ,int sensorStateL ,int sensorStateR ,int sensorStateRR) {
  // Left turn detected when LL, L, and R are ON
  return (sensorStateLL == LOW && sensorStateL == HIGH && sensorStateR == LOW && sensorStateRR == LOW);
}

bool OnLine(int sensorStateLL ,int sensorStateL ,int sensorStateR ,int sensorStateRR) {
  return (sensorStateLL == LOW && sensorStateL == HIGH && sensorStateR == HIGH && sensorStateRR == LOW);
}

bool IAmLost(int sensorStateLL ,int sensorStateL ,int sensorStateR ,int sensorStateRR) {
  // Robot is lost when all sensors are OFF
  return (sensorStateLL == LOW && sensorStateL == LOW && sensorStateR == LOW && sensorStateRR == LOW);
}

int MakeDecision(int sensorStateLL ,int sensorStateL ,int sensorStateR ,int sensorStateRR) {
  
  if (DetectCross(sensorStateLL , sensorStateL , sensorStateR , sensorStateRR) == true) {
    return 1;
  }
  else if (DetectRightTurn(sensorStateLL , sensorStateL , sensorStateR , sensorStateRR) == true) {
    return 2;
  }

  else if (DetectLeftTurn(sensorStateLL , sensorStateL , sensorStateR , sensorStateRR) == true) {
    return 3;
  }
  
  else if (OffToTheRight(sensorStateLL , sensorStateL , sensorStateR , sensorStateRR) == true) {
    return 4;
  }

  else if (OffToTheLeft(sensorStateLL , sensorStateL , sensorStateR , sensorStateRR) == true) {
    return 5;
  }

  else if (OnLine(sensorStateLL , sensorStateL , sensorStateR , sensorStateRR) == true) {
    return 6;
  }

  else if (IAmLost(sensorStateLL , sensorStateL , sensorStateR , sensorStateRR) == true) {
    return 7; 
  }

  else {
    return 0;
  }
}

void runForwards(int time) {
  speed = 100;  // Set the forward speed
  runMotor(speed, 1, MotorR);
  runMotor(speed, 1, MotorL);

  if (time != 0) {
    delay(time);
    stopMotors();
  }
}

void stopMotors() {
  runMotor(0, 1, MotorR);
  runMotor(0, 1, MotorL);
}

void AdjustR() {
  runMotor(120, 1, MotorR);
  runMotor(100, 1, MotorL);
  delay(500);
  
  runMotor(100, 1, MotorR);
  runMotor(120, 1, MotorL);
  delay(500);
}

void AdjustL() {
  runMotor(100, 1, MotorR);
  runMotor(120, 1, MotorL);
  delay(500);
  
  runMotor(120, 1, MotorR);
  runMotor(100, 1, MotorL);
  delay(500);
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








