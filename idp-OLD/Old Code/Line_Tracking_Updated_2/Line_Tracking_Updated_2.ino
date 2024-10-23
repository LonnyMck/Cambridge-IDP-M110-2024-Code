/*
Cambridge University IDP, Team M109, 2023
This is the test code to use the wheel speed to determine the coordinates of our robot on the field.
*/

#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// the two motors
Adafruit_DCMotor *wheel_left = AFMS.getMotor(1); //the left wheel
Adafruit_DCMotor *wheel_right = AFMS.getMotor(4); // the right wheel

int x_step = 0; // x coordinates
int y_step = 0; //y coordinates
int bearings = 0; //angle of rotation taken from the starting position, assumed north
int counter = 0; //temporary variable to count the amount of loops the code goes through
int time_loop = 500; //variable used to control the motor, I can't explain it well just ignore it
int leftlinesensorPin = 2;
int rightlinesensorPin = 3; // Connect sensor to input pin 3
int farrightlinesensorPin = 5; // Connect sensor to input pin 3
int inputPin = 4; 


bool start = false; //changes to TRUE and movement starts when button pressed
bool line_left; //checks the state of the two line detectors and converts to a boolean
bool line_right;

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

int val = digitalRead(inputPin); // read input value
int valleft = digitalRead(leftlinesensorPin); // read left input value
int valright = digitalRead(rightlinesensorPin); // read right input value
int valfarright = digitalRead(farrightlinesensorPin); // read far right input value



bool left_bef;
bool right_bef;



    if (valfarright > 0.5){  //updates time since seeing a right turn for having detected a right turn is available
      right_turn_time = counter;
    }
    if((counter - right_turn_time) < 5 and line_left == false and line_right == false and counter >5){    //Turns the flag for turn on if it has seen a turn and then goes off track
      right_turn = true;
    }

if (val == HIGH) { // check if the input is HIGH
  start = true;
  counter = 0;
}

     //Line following code for a straight line/////////////////////////////////////////////////////////////////////////


if (counter < 1 and start == true and right_turn == false){ //counter goes up every 10 ms so this loop runs for 5000 ms, or 5 seconds
    wheel_left->run(BACKWARD);
    wheel_left->setSpeed(255);

    wheel_right->run(BACKWARD);
    wheel_right->setSpeed(255);

    left_bef = false;
    right_bef = false;

    if (valleft >0.5){  //updates boolean for line sensors
      line_left = true;
    }else{
      line_left = false;
    }

    if (valright >0.5){
      line_right = true;
    }else{
      line_right = false;
    }


    Serial.println(x_step);

  }else{

    
    if (counter < 2000 and start == true and right_turn == false){
      left_bef = line_left;   //Stores the prior left and right
      right_bef = line_right;

    if (valleft >0.5){   //updates line sensor boolean
      line_left = true;
    }else{
      line_left = false;
    }

    if (valright >0.5){
      line_right = true;
    }else{
      line_right = false;
    }

    wheel_right->run(BACKWARD);
    wheel_left->run(BACKWARD);


    if (line_left == true and line_right == true){
      wheel_left->setSpeed(255);
      wheel_right->setSpeed(255);

      }else{
         if (((line_left == true and right_bef == true) and (left_bef == false or line_right == false)) or ((line_left == false and right_bef == false) and ((left_bef == true and line_right == false) or (line_right == true and left_bef == true) ))){    
        //It has turned to the right so needs to go left
        wheel_left->setSpeed(200);
        wheel_right->setSpeed(255);

      }else{ 
        if (((line_right == true and left_bef == true) and (right_bef == false or line_left == false)) or (line_left == true and line_right == false and left_bef == false) or (left_bef == false and right_bef == true and line_right == false)){   // Has moved to the left
          wheel_left->setSpeed(255);
          wheel_right->setSpeed(200);

      }else{
        if (line_left == true and left_bef == false){   // Moving on the line
          wheel_left->setSpeed(255);
          wheel_right->setSpeed(255);

      }
    }
      }}}}


////At a right turn

if (right_turn == true and start==true and counter <2000){
  wheel_left->setSpeed(255);
  wheel_right->run(FORWARD);
  wheel_right->setSpeed(200);
    if (valright >0.5){
      line_right = true;
    }else{
      line_right = false;
    }


  if(line_right == true){
    right_turn = false;
    wheel_right->run(BACKWARD);
    wheel_right->setSpeed(255);
    }
}


  counter +=1 ;
  x_step += 1;
  delay(10);
}