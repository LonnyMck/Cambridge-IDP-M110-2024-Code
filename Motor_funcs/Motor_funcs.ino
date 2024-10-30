/*
Test for motors in chassis
*/

#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X sensor;

int SENSOR_MAG = 6;
float BLOCK_NEARBY = 150;
float BLOCK_CLOSE = 55; //Try messing about with this value, maybe reducing it

//LEDs are on pins 3, 4, and 5
int LED_BLUE = 3;
int LED_GREEN = 4;
int LED_RED = 5;

//after block is detected, switch var for if it is or isnt magnetic
bool isMagnetic;
bool grabberEngaged;


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *MotorR = AFMS.getMotor(3);   // Left motor
Adafruit_DCMotor *MotorL = AFMS.getMotor(2);  // Right motor


Servo controlservo; // create servo object to control a servo
int servopos = 0; // variable to store the servo position

bool running = true;
char input;

double speed;
double duration;
double time;

//for block sensing
//float block_distance;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor func test");

  //Grabber sensor initialisation
  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  //Laser rangefinder begins to work
  sensor.start();
  pinMode(SENSOR_MAG, INPUT); //sets magnetic to input
  //set all LEDs to output
  pinMode(LED_BLUE, OUTPUT); 
  pinMode(LED_GREEN, OUTPUT); 
  pinMode(LED_RED, OUTPUT); 


  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Motor Shield found.");

  controlservo.attach(3); // attaches the servo on pin 3 to the servo object

  releaseGrabber(); //Set the grabber to be released to start

}

void loop() {

  //releaseGrabber();

  running = check_interrupt(); //Check for interrupt
  if (running) {  //No interrupt has been detected

  runForwards(0,150);

  //checkMagnetic();
  CheckforBlock();

  /*
  engageGrabber();
  delay(5000);
  releaseGrabber();
  */

  //runServo(0);  //Useful, can run this as the only code in the if(running) section to move the servo to certain positions. 0 is an open grabber, increasing the number closes it more and more.





  
  

  }}



///////////////////////////
//Declaring functions
///////////////////////////

void runForwards(int time, double speed) {
  //speed = 100;  // Set the forward speed
  runMotor(speed, 1, MotorR);
  runMotor(speed, 1, MotorL);

  if (time != 0) {
    delay(time);
    stopMotors();
  }
}

int runBackwards(int time){  //Optional argument time. Set time = 0 unless needed.

  speed = 100;  //Work out best value through testing, or add as argument to func.

  runMotor(speed, 0, MotorR);
  runMotor(speed, 0, MotorL);

  if (time != 0){
    delay(time);
    stopMotors();
    }
}

void stopMotors() {
  runMotor(0, 1, MotorR);
  runMotor(0, 1, MotorL);
}

int turnRight(int degrees){ ///////////// Need to calibrate the time and speeds

  time = (degrees/90) * 4400;  //This needs calibrating

  runMotor(100, 1, MotorR);  //May need to switch these around
  runMotor(50, 0, MotorL); //Spins backwards

  delay(time);

  stopMotors();

}

int turnLeft(int degrees){ ///////////// Need to calibrate the time and speeds

  time = (degrees/90) * 4400;  //This needs calibrating

  runMotor(50, 0, MotorR);  //May need to switch these around
  runMotor(100, 1, MotorL); //Spins forwards

  delay(time);

  stopMotors();

}


void runMotor(int speed, bool direction, Adafruit_DCMotor *motorObject) {
  running = check_interrupt(); //Check for interrupt
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
  runServo(100);
  checkMagnetic();  //try it here

}


int releaseGrabber(){

  Serial.println("Releasing grabber");
  grabberEngaged = false;
  isMagnetic = false;
  runServo(0);
  turnLedsOff();

}


int CheckforBlock(){

  //Get the distance
  if (sensor.getDistance() < BLOCK_CLOSE and grabberEngaged == false) { //check that the obstacle detected isn't the block held in grabber
    stopMotors();
    engageGrabber();
  }

  /*
  else if (sensor.getDistance() < BLOCK_NEARBY and grabberEngaged == false){
      runForwards(0,100);
  }*/

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