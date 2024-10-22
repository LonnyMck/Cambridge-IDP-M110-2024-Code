/*
Cambridge University IDP, Team M109, 2023
This is the test code to use the wheel speed to determine the coordinates of our robot on the field.
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// the two motors
Adafruit_DCMotor *wheel_left = AFMS.getMotor(1); //the left wheel
Adafruit_DCMotor *wheel_right = AFMS.getMotor(2); // the right wheel

int x_step = 0; // x coordinates
int y_step = 0; //y coordinates
int bearings = 0; //angle of rotation taken from the starting position, assumed north
int counter = 0; //temporary variable to count the amount of loops the code goes through
int time_loop = 300; //variable used to control the motor, I can't explain it well just ignore it
int inputPin = 4; 

bool start = false; //changes to TRUE and movement starts when button pressed

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  // turn on motor
  wheel_left->run(RELEASE);
  wheel_right->run(RELEASE);

  pinMode(inputPin, INPUT); // declare pushbutton as input

  delay(100);
}

void loop() {


int val = digitalRead(inputPin); // read input value
if (val == HIGH) { // check if the input is HIGH
  start = true;
}


  if (counter < time_loop && start == true){ //counter goes up every 10 ms so this loop runs for 5000 ms, or 5 seconds
    wheel_left->run(BACKWARD);
    wheel_left->setSpeed(255);

    wheel_right->run(BACKWARD);
    wheel_right->setSpeed(255);


    
    counter = counter + 1;
    x_step += 1;
    Serial.println(x_step);

  } 
  else {
    wheel_left->setSpeed(0);

    wheel_right->setSpeed(0);
  }


  delay(10);
}
