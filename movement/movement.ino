#include <Adafruit_MotorShield.h>

// Define sensor pins
const int sensorPinLL = 13;
const int sensorPinL = 12;
const int sensorPinR = 11;
const int sensorPinRR = 10;
const int sensorPinB = 2;
int buttonState = 0;

// Define green button pin
const int redButtonPin = 7;    //Red button connected to pin 7
const int greenButtonPin = 8;  // Green button connected to pin 8


// Define LED pins, consts, vars
int LED_BLUE = 9;
int LED_GREEN = 4;
int LED_RED = 5;

unsigned long time_since_led;  //time variable, time since last LED flash


// Motor and servo control
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorR = AFMS.getMotor(3);  // Right motor
Adafruit_DCMotor *MotorL = AFMS.getMotor(2);  // Left motor

bool isStarted = false;           // Flag to check if the green button has been pressed
bool isHalted = false;

int megacounter = 0;
int counter = 0;

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  Serial.println("Motor and sensor func test");
  isStarted = false;
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }

  Serial.println("Motor Shield found.");

  // Initialize sensors as inputs
  pinMode(sensorPinLL, INPUT);
  pinMode(sensorPinL, INPUT);
  pinMode(sensorPinR, INPUT);
  pinMode(sensorPinRR, INPUT);
  pinMode(sensorPinB, INPUT);

  // Initialize green button pin as input with internal pull-up resistor
  pinMode(greenButtonPin, INPUT);
  pinMode(redButtonPin, INPUT);


  //initialize LEDs
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
}


/*

FUNCTIONS

*/


//function for blue light to flash when moving
void flashWhenMoving( bool isMoving){

  if ( !isMoving ){ //if the robot isn't moving, turn the blue pin OFF
    digitalWrite(LED_BLUE, LOW);
    return false;
  }

  //call this function when the motors running, it will flash the led every 1000 ms
  if ( (millis() - time_since_led) > 1000){ //if the time since we last changed the LED output is > 100, change it and reset the count
    bool isHigh = digitalRead(LED_BLUE);
    digitalWrite(LED_BLUE, !isHigh);
    time_since_led = millis(); //reset the time for the next interval
    return true;
  }

}


bool hasInitiated = false;

void loop() {
  // Check if the red button has been pressed
  if (digitalRead(redButtonPin) == HIGH) {
    isHalted = true;
    isStarted = false;
    stop(); // Immediately stop all motors
  }

  // Check if the green button has been pressed to start
  if (!isStarted) {
    if (digitalRead(greenButtonPin) == HIGH) {
      isStarted = true;
      isHalted = false;
      Serial.println("Button High");
      delay(100); // Debounce delay
    } else {
      return; // Do nothing until the green button is pressed
    }
  }

  if (isHalted) {
    return;  // Exit the loop early
  }

  if (!isStarted) {

    if (buttonState == HIGH) {
      isStarted = true;
      isHalted = false;
      Serial.println("Button High");
    } else {
      return;  // Do nothing until the green button is pressed
    }
    delay(100);
  }

  if (isStarted) {


    bool LL = digitalRead(sensorPinLL);
    bool L = digitalRead(sensorPinL);
    bool R = digitalRead(sensorPinR);
    bool RR = digitalRead(sensorPinRR);
    bool B = digitalRead(sensorPinB);

    if (!hasInitiated){

      goForward(200);

      if (L==HIGH && R==HIGH){
        stop();
        counter++;
        hasInitiated = true;
      }
    }

    Serial.println(counter);
    flashWhenMoving( isStarted );

    if (hasInitiated==true && counter<3 && megacounter==0) {
      countFollow(200 , LL , L , R , RR , B);
    } else if (counter==3 && megacounter==0) {
      stop();
      delay(500);
      makeTurn('L');
      countFollow(200, LL, L, R, RR, B);
    }
  }

  delay(300);
  Serial.println(counter);
}




void normalFollow(int speed, bool L , bool R,  bool B){
  if (L == HIGH && B == HIGH && R == HIGH) {
    goForward(speed);
  } else if (L == HIGH && B == HIGH && R == LOW) {
    runMotor(speed + speed / 4, 1, MotorR);
    runMotor(speed, 1, MotorL);
  } else if (L == HIGH && B == LOW && R == HIGH) {
    runMotor(speed, 1, MotorR);
    runMotor(speed, 1, MotorL);
  } else if (L == LOW && B == HIGH && R == HIGH) {
    runMotor(speed, 1, MotorR);
    runMotor(speed + speed / 4, 1, MotorL);
  } else if (L == HIGH && B == LOW && R == LOW) {
    runMotor(speed + speed / 2, 1, MotorR);
    runMotor(speed, 1, MotorL);
  } else if (L == LOW && B == LOW && R == HIGH) {
    runMotor(speed, 1, MotorR);
    runMotor(speed + speed / 2, 1, MotorL);
  }
}

bool counterability = true;

void countFollow(int speed, bool LL, bool L , bool R, bool RR, bool B) {
  normalFollow(speed, L, R, B);
  if ((RR == HIGH || LL == HIGH) && counterability == true) {
    counter++;
    counterability = false;
  }
  if ((RR == LOW && LL == LOW) && counterability == false){
    counterability = true;
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

void goForward(int speed) {
  runMotor(speed, 1, MotorL);
  runMotor(speed, 1, MotorR);
}

void goBackward(int speed) {
  runMotor(speed, 0, MotorL);
  runMotor(speed, 0, MotorR);
}

void turnR(int speed) {
  runMotor(speed, 1, MotorL);
  runMotor(0, 1, MotorR);
}

void turnL(int speed) {
  runMotor(0, 1, MotorL);
  runMotor(speed, 1, MotorR);
}

void stop() {
  runMotor(0, 1, MotorL);
  runMotor(0, 1, MotorR);
}

void makeTurn(char direction) {
  // Stop both motors initially
  stop();
  delay(500);

  goBackward(200);
  delay(500);

  // Turn in the specified direction
  if (direction == 'R') {
    turnR(200);
  } else if (direction == 'L') {
    turnL(200);
  }

  delay(3000);

  // Continue turning until the correct sensor pin goes HIGH again, aligning with the line
  if (direction == 'R') {
    while (digitalRead(sensorPinL) == LOW) {
      turnR(200);
    }
  } else if (direction == 'L') {
    while (digitalRead(sensorPinR) == LOW) {
      turnL(200);
    }
  }

  // Stop both motors after alignment
  runMotor(0, 0, MotorL);
  runMotor(0, 0, MotorR);

  // Reset flags to resume line following
}

