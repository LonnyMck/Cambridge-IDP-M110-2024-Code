/*
Cambridge University IDP, Team M109, 2023
This is the test code to use the wheel speed to determine the coordinates of our robot on the field.
*/

#include <Adafruit_MotorShield.h>
#include "Wire.h"
#include "DFRobot_VL53L0X.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// the two motors
Adafruit_DCMotor *wheel_left = AFMS.getMotor(1); //the left wheel
Adafruit_DCMotor *wheel_right = AFMS.getMotor(4); // the right wheel

DFRobot_VL53L0X sensor;

int x_step = 0; // x coordinates
int y_step = 0; //y coordinates
int bearings = 0; //angle of rotation taken from the starting position, assumed north
int counter = 0; //temporary variable to count the amount of loops the code goes through
int time_loop = 500; //variable used to control the motor, I can't explain it well just ignore it
int leftlinesensorPin = 2;
int rightlinesensorPin = 3; // Connect sensor to input pin 3
int farrightlinesensorPin = 5; // Connect sensor to input pin 5
int farleftlinesensorPin = 6; // Connect sensor to input pin 6
int led_blue = 7;

int inputPin = 4; 


bool start = false; //changes to TRUE and movement starts when button pressed
bool line_left; //checks the state of the two line detectors and converts to a boolean
bool line_right;

bool right_turn = false;
int right_turn_time = 0;
bool left_turn = false;
int left_turn_time = 0;
bool line_straight = true;

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
  pinMode(farleftlinesensorPin, INPUT); // declare line sensor as input
  pinMode(led_blue, OUTPUT);
//Initialising time of flight stuff
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous,sensor.eHigh);
  sensor.start();
  delay(100);
}
//Arduino was being funny and not letting these be after the loop so they're here nows
void straight_ahead(int speed = 255){
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(speed);
  wheel_right->run(BACKWARD);
  wheel_right->setSpeed(speed);
}

void forwards_right(int speed = 255){
    wheel_right->run(BACKWARD);
    wheel_left->run(BACKWARD);
    wheel_left->setSpeed(0.8*speed);
    wheel_right->setSpeed(speed);
}

void forwards_left(int speed = 255){
    wheel_right->run(BACKWARD);
    wheel_left->run(BACKWARD);
    wheel_right->setSpeed(0.8*speed);
    wheel_left->setSpeed(speed);
}


void loop() {

  int val = digitalRead(inputPin); // read input value
  int valleft = digitalRead(leftlinesensorPin); // read left input value
  int valright = digitalRead(rightlinesensorPin); // read right input value
  int valfarright = digitalRead(farrightlinesensorPin); // read far right input value
  int valfarleft = digitalRead(farleftlinesensorPin); // read far right input value
  int turn_time = 5;   //Delay between spotting a turn and the latest it could take that turn
  int tof_distance;
  int tof_distance_bef;

  bool left_bef;
  bool right_bef;
  bool moving;
  bool sweeping;

  //Serial.print("Distance: ");Serial.println(sensor.getDistance());

  //Decides if it should turn right

  if (valfarright > 0.5){  //updates time since seeing a right turn for having detected a right turn is available
    right_turn_time = counter;
  }
  if((counter - right_turn_time) < turn_time and line_left == false and line_right == false and counter >turn_time){    //Turns the flag for turn on if it has seen a turn and then goes off track
    right_turn = true;
  }

  //Decides whether or not to turn left

  if (valfarleft > 0.5){  //updates time since seeing a left turn for having detected a left turn is available
    left_turn_time = counter;
  }
  if((counter - left_turn_time) < turn_time and line_left == false and line_right == false and counter > turn_time){    //Turns the flag for turn on if it has seen a turn and then goes off track
    left_turn = true;
  }

  //If it isn't turning then it will go straight
  if (left_turn == false and right_turn == false){
    line_straight = true;
  }else{
    line_straight = false;
  }

  tof_distance_bef = tof_distance;
  tof_distance = sensor.getDistance();
///Sets led to flash at 2 Hz



  if (val == HIGH) { // check if the input is HIGH
    start = !start;
    counter = 0;
    delay(500);
  }



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


      //Line following code for a straight line/////////////////////////////////////////////////////////////////////////

  ///First cycle initialising states and moving forwards
  if (counter < 1 and start == true){ //counter goes up every 10 ms so this loop runs for 5000 ms, or 5 seconds
      sweeping = true;
      straight_ahead();
      left_bef = false;
      right_bef = false;
      Serial.println(counter);

    }else{


      if (counter >2000 or start == false){
        straight_ahead(0);
        digitalWrite(led_blue, LOW);
      }
      else{
        Serial.println(sweeping);

      if (sweeping == true){
        Serial.println("SWEEPING!!!!");
          if (counter % 300 == 0 or counter == 2){ //every three seconds the rotation of robot changes
            if (counter % 300 == 0 or counter == 2){ //robot starts rotating right at the start
              rotate_right();
            }
          else{ 
            rotate_left(); //robot rotates left after 3 seconds
          }
        }
        if (tof_distance_bef - tof_distance >500 and counter>5){
          sweeping = false;
        }}

      else{

          if (left_turn == true or right_turn == true or line_straight == true){    //if moving
            if(counter % 50 ==0){
              digitalWrite(led_blue, HIGH);
             }else{
              if(counter %50 == 25)
                digitalWrite(led_blue, LOW);
  }
  }
      ////While moving and following the line
      if (line_straight == true){


      if (line_left == true and line_right == true){
        straight_ahead();

        }else{

          if (((line_left == true and right_bef == true) and (left_bef == false or line_right == false)) or ((line_left == false and right_bef == false) and ((left_bef == true and line_right == false) or (line_right == true and left_bef == true) ))){    
          //It has turned to the right so needs to go left
            forwards_right();

        }else{ 
          if (((line_right == true and left_bef == true) and (right_bef == false or line_left == false)) or (line_left == true and line_right == false and left_bef == false) or (left_bef == false and right_bef == true and line_right == false)){   // Has moved to the left
            forwards_left();

        }else{
          if (line_left == true and left_bef == false){   // Moving on the line
            straight_ahead();

      }}}}}

        else{

  ////At a right turn
            if (right_turn == true){
                turn_right();
                }
                else{

  ////At a left turn
            if (left_turn == true and start==true){
              turn_left();

}}}}}}
  


    counter +=1 ;
    x_step += 1;
    delay(10);
  }




void turn_right(){
    wheel_left->setSpeed(255);
    wheel_right->run(FORWARD);
    wheel_right->setSpeed(200);



    if(line_right == true){
      right_turn = false;
      wheel_right->run(BACKWARD);
      wheel_right->setSpeed(255);
      }
}

void turn_left(){
    wheel_right->setSpeed(255);
    wheel_left->run(FORWARD);
    wheel_left->setSpeed(200);



    if(line_right == true){
      left_turn = false;
      wheel_left->run(BACKWARD);
      wheel_left->setSpeed(255);
      }
}

void rotate_right(){
    wheel_left->run(BACKWARD);
    wheel_left->setSpeed(100);

    wheel_right->run(FORWARD);
    wheel_right->setSpeed(100);
}

void rotate_left(){
    wheel_left->run(FORWARD);
    wheel_left->setSpeed(100);

    wheel_right->run(BACKWARD);
    wheel_right->setSpeed(100);
}
