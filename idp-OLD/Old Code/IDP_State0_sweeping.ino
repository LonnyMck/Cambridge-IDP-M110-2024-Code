/*
Cambridge University IDP, Team M109, 2023
This is the test code to serve as the first function of the robot, 
which is turning in a sweeping motion and identifying blocks at a 
distance
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// the two motors
Adafruit_DCMotor *wheel_left = AFMS.getMotor(1); //the left wheel
Adafruit_DCMotor *wheel_right = AFMS.getMotor(4); // the right wheel

//general variables
int x_step = 0; // x coordinates
int y_step = 0; //y coordinates
int bearings = 0; //angle of rotation taken from the starting position, assumed north
int counter = 0; //temporary variable to count the amount of loops the code goes through
int time_loop = 500; //variable used to control the motor, I can't explain it well just ignore it

//line sensors
int leftlinesensorPin = 2;
int rightlinesensorPin = 3; // Connect sensor to input pin 3
int farrightlinesensorPin = 5; // Connect sensor to input pin 3

bool line_left; //checks the state of the two line detectors and converts to a boolean
bool line_right;

//ultravsonic sensors
int sensity_pin = ;
int MAX_RANG = 520; //the max measurement value of the module is 520cm(a little bit longer than effective max range)
float ADC_SOLUTION = 1023.0;//ADC accuracy of Arduino UNO is 10bit

//button to start robot
int inputPin = 4; 
bool start = false; //changes to TRUE and movement starts when button pressed


bool right_turn = false;
int right_turn_time = 0;


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
  pinMode(leftlinesensorPin, INPUT); // declare line sensor as input
  pinMode(rightlinesensorPin, INPUT); // declare line sensor as input
  pinMode(farrightlinesensorPin, INPUT); // declare line sensor as input
  delay(100);
}


void loop() {

  int val = digitalRead(inputPin); // read input value of button
  sensity_t = analogRead(sensityPin); // reads input value of ultrasonic sensor


  if (val == HIGH) { // check if the input is HIGH
  
  start = !start; //if button pressed, it will turn robot on and off
  counter = 0; //restarts counter
  delay(500); //cheap and easy solution to prevent 'switch bounce'

}

if (start == true){
  if (counter % 100 == 0 or counter == 0){ //every three seconds the rotation of robot changes
    if (counter % 200 == 0 or counter == 0){ //robot starts rotating right at the start
      rotate_right();
    }
    else{ 
      rotate_left(); //robot rotates left after 3 seconds
    }
  }


dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;//
Serial.print(dist_t,0,);

}else{
    wheel_left->setSpeed(0);
    wheel_right->setSpeed(0);
}
  delay(10);
}



void rotate_right(){
      wheel_left->run(BACKWARD);
    wheel_left->setSpeed(90);

    wheel_right->run(FORWARD);
    wheel_right->setSpeed(90);
}


void rotate_left(){
    wheel_left->run(FORWARD);
    wheel_left->setSpeed(90);

    wheel_right->run(BACKWARD);
    wheel_right->setSpeed(90);
}