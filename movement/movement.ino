#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X sensor;


int SENSOR_MAG = 6;
float BLOCK_NEARBY = 150;
float BLOCK_CLOSE = 50;

// Define sensor pins
const int sensorPinLL = 8;
const int sensorPinL = 7;
const int sensorPinR = 6;
const int sensorPinRR = 5;
const int sensorPinB = 4;
int buttonState = 0;

//after block is detected, switch var for if it is or isnt magnetic
bool isMagnetic;
bool grabberEngaged;

// Define green button pin
const int redButtonPin = 3;    //Red button connected to pin 7
const int greenButtonPin = 2;  // Green button connected to pin 8

// Define LED pins, consts, vars
int LED_BLUE = 11;
int LED_GREEN = 12;
int LED_RED = 13;

unsigned long time_since_led;  //time variable, time since last LED flash

// Motor and servo control
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *MotorR = AFMS.getMotor(3);  // Right motor
Adafruit_DCMotor *MotorL = AFMS.getMotor(2);  // Left motor

Servo controlservo; // create servo object to control a servo
int servopos = 0; // variable to store the servo position

bool running = true;
char input;


bool isStarted = false;           // Flag to check if the green button has been pressed

int megacounter = 0;
int counter = 0;
bool counterability = true;






void junctionReset() {
  counterability = true;
  Serial.println("counter on from isStarted");
  counter = 0;
  Serial.println("Counter reset");
  megacounter++;
  Serial.println("Megacounter update");
  Serial.println(megacounter);
  counterability = false;
  Serial.println("counter off from isStarted");
}

void normalFollow(int speed){

  bool L = digitalRead(sensorPinL);
  bool R = digitalRead(sensorPinR);
  bool B = digitalRead(sensorPinB);

  if (L == HIGH && B == HIGH && R == HIGH) {
    goForward(speed);
    //Serial.println("1111");
  } else if (L == HIGH && B == LOW && R == HIGH) {
    goForward(speed);
    //Serial.println("2222");
  } else if (L == HIGH && B == HIGH && R == LOW) {
    runMotor(speed + 25, 1, MotorR);
    runMotor(speed - 25, 1, MotorL);
    //Serial.println("3333");
  } else if (L == LOW && B == HIGH && R == HIGH) {
    runMotor(speed - 25, 1, MotorR);
    runMotor(speed + 25, 1, MotorL);
    //Serial.println("4444");
  } else if (L == HIGH && B == LOW && R == LOW) {
    runMotor(speed - 50, 1, MotorL);
    runMotor(speed + 50, 1, MotorR);
    //Serial.println("5555");
  } else if (L == LOW && B == LOW && R == HIGH) {
    runMotor(speed - 50, 1, MotorR);
    runMotor(speed + 50, 1, MotorL);
    //Serial.println("6666");
  } 
}


void countFollow(int speed) {

  bool LL = digitalRead(sensorPinLL);
  bool RR = digitalRead(sensorPinRR);

  if (RR == LOW && LL == LOW && counterability == false){
    counterability = true;
    Serial.println("counter on from countfollow");
  }
  if (RR == HIGH || LL == HIGH) {
    if (counterability == true) {
      counter++;
      Serial.println("Counter update");
      Serial.println(counter);
      counterability = false;
      Serial.println("counter off from countfollow");
  }
  }
  normalFollow(speed);
  
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
  delay(400);

  // Turn in the specified direction
  if (direction == 'R') {
    turnR(200);
  } else if (direction == 'L') {
    turnL(200);
  }

  delay(2500);

  // Continue turning until the correct sensor pin goes HIGH again, aligning with the line
  if (direction == 'R') {
    while (digitalRead(sensorPinL) == LOW) {
      turnR(200);
    }
    turnR(200);
    delay(100);
  } else if (direction == 'L') {
    while (digitalRead(sensorPinR) == LOW) {
      turnL(200);
    }
    turnL(200);
    delay(100);
  }

  // Stop both motors after alignment
  runMotor(0, 0, MotorL);
  runMotor(0, 0, MotorR);

  // Reset flags to resume line following
}


int runServo(int newpos){ // sends servo to a specific position

  running = check_interrupt(); //Check for interrupt

  Serial.println("Servo moving to position " + String(newpos));
  controlservo.write(newpos); 
  servopos = newpos;
  Serial.println("Servo pos = " + String(servopos));
  delay(2000);

}

int engageGrabber(){

  Serial.println("Engaging grabber");
  grabberEngaged = true;
  runServo(145);
  checkMagnetic();  //try it here

}


int releaseGrabber() {

  Serial.println("Releasing grabber");
  grabberEngaged = false;
  isMagnetic = false;
  runServo(50);
  turnLedsOff();

}


int CheckforBlock(){

  Serial.println(sensor.getDistance());

  if (sensor.getDistance() < BLOCK_CLOSE and grabberEngaged == false) { //check that the obstacle detected isn't the block held in grabber
    Serial.println("Block close");
    stop();
    engageGrabber();
  
  }

  
  else if (sensor.getDistance() < BLOCK_NEARBY and grabberEngaged == false) {
      Serial.println("Block nearby");
      // speed = 150;  will implement later 
  }

  Serial.print("Distance: ");
  Serial.print(sensor.getDistance());

}

int checkMagnetic(){  //Takes three readings, check that this code works

  for (int count = 0; count <= 10; count++){
     			isMagnetic = digitalRead(SENSOR_MAG);
          if (isMagnetic == true){
            break;
          }
  }

  Serial.print(", Magnetic: ");
  Serial.println( digitalRead(SENSOR_MAG) );
  shineLedBlockType( digitalRead(SENSOR_MAG) );

}

void shineLedBlockType( bool isMagnetic ){  //shine correct LED depending on if magnetic or non magnetic
  if (isMagnetic){ 
    digitalWrite(LED_RED, HIGH ); 
    digitalWrite( LED_GREEN, LOW );
  }else{
    digitalWrite(LED_RED, LOW ); 
    digitalWrite( LED_GREEN, HIGH );
  }

}

//turns all LEDs off
void turnLedsOff(){
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, LOW);
}


int check_interrupt(){ // checks for interrupts and breaks loop. Returns boolean. Will eventually be an Estop.

  if(Serial.available()){ // Adds keyboard interrupt
        input = Serial.read();
      }
    
    if(input == 't'){
     Serial.println("Interrupt");
     runMotor(0, 1, MotorR);
     runMotor(0, 1, MotorL);
     releaseGrabber();
     while(1);  //Get stuck in an endless loop and doesn't execute any new code
    }
    return true;
}


void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps
  Serial.println("Motor and sensor func test");
  isStarted = false;
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
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


  controlservo.attach(10); // attaches the servo on pin 3 to the servo object

  releaseGrabber(); //Set the grabber to be released to start

  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  //Laser rangefinder begins to work
  sensor.start();

  
  pinMode(SENSOR_MAG, INPUT); //sets magnetic to input
}


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

  CheckforBlock();
  // // Check if the red button has been pressed
  // if (digitalRead(redButtonPin) == HIGH) {
  //   isStarted = false;
  //   stop(); // Immediately stop all motors
  // }

  // if (!isStarted) {
  //   if (digitalRead(greenButtonPin) == HIGH) {
  //     isStarted = true;
  //     Serial.println(isStarted);
  //     delay(100); // Debounce delay
  //   } else {
  //     return; // Do nothing until the green button is pressed
  //   }
  // }

  // if (isStarted) {

  //   while (hasInitiated == false) {
  //     bool LL = digitalRead(sensorPinLL);
  //     bool L = digitalRead(sensorPinL);
  //     bool R = digitalRead(sensorPinR);
  //     bool RR = digitalRead(sensorPinRR);
  //     bool B = digitalRead(sensorPinB);
  //     goForward(200);  
  //     Serial.println("Moving forward until first cross");
  //     if (L == HIGH || R == HIGH) {
  //       counter += 1;
  //       Serial.println("First cross detected, counter updated");
  //       Serial.println(counter);
  //       counterability = false;
  //       Serial.println("Counter off until line passed");
  //       hasInitiated = true;
  //       delay(100);
  //       }
  //   }

    
  //   while (hasInitiated == true) {
  //     flashWhenMoving( isStarted );

  //     if (counter<3 && megacounter==0) {
  //       //Serial.println("Crossed the first line");
  //       countFollow(200);
  //     }
      
  //     if (megacounter==0 && counter==3) {
  //       stop();
  //       delay(500);
  //       makeTurn('L');
  //       normalFollow(200);
  //       delay(2000);
  //       junctionReset();
  //     }

  //     if (megacounter == 1 && counter<2) {
  //       countFollow(200);
  //     }

  //     if (counter == 1 && megacounter == 1) {
  //       makeTurn('R');
  //       junctionReset();
  //     }

  //     else if (megacounter == 2 && counter < 2) {
  //       countFollow(200);
  //     }
  //     else if (megacounter == 2 && counter == 2) {
  //       makeTurn('R');
  //       junctionReset();
  //     }
  //     else if (megacounter == 3 && counter == 0) {
  //       countFollow(200);
  //     }
  //     else if (megacounter == 3 && counter == 1) {
  //       makeTurn('R');
  //       junctionReset();
  //     }
  //     else if (megacounter == 4 && counter == 0) {
  //       countFollow(200);
  //     }
    
  //     else if (megacounter == 4 && counter == 1) {
  //       makeTurn('R');
  //       junctionReset();
  //     }
  //     else if (megacounter == 5 && counter == 0) {
  //       CheckforBlock();
  //       countFollow(100);
  //       //activate pickup mechanism here
  //     }
      
    

  // }
  // }
  // delay(100);
}


