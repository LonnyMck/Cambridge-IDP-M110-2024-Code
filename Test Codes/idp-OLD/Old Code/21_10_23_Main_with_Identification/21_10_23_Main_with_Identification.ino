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


//Initialise time of flight
DFRobot_VL53L0X sensor;


#define MAX_RANG (520)//the max measurement value of the module is 520cm(a little bit longer than effective max range)
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit

//Initialise all pin inputs

int inputPin = 4; // Button pin

int led_blue = 5;
int led_red = 6;
int led_green = 7;

int farleftlinesensorPin = 8; // Connect sensor to input pin 6
int farrightlinesensorPin = 9; // Connect sensor to input pin 5
int leftlinesensorPin = 10;
int rightlinesensorPin = 11; // Connect sensor to input pin 3

int ultrasonic_pin = A1; 

int state = 0;
int us_distance_bef;
int us_change;
int ultrasonic_dist;
int tof_distance_bef;
int tof_distance;
int tof_change;


//int bearings = 0; //angle of rotation taken from the starting position, assumed north
int counter = 0; //temporary variable to count the amount of loops the code goes through
int density = 0;

//int time_loop = 500; //variable used to control the motor, I can't explain it well just ignore it


bool start = false; //changes to TRUE and movement starts when button pressed
bool line_left; //checks the state of the two line detectors and converts to a boolean
bool line_right;
bool left_bef;
bool right_bef;

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
 //   while (1);
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
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
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

  // Serial.println("US:");
  // Serial.println(ultrasonic_dist * 10);
  // Serial.println("TOF:");
  // Serial.println(tof_distance);
  bool moving;


  ///Updates values for the sensors to check if its on a line
  
  left_bef = line_left;   //Stores the prior left and right
  right_bef = line_right;

  us_distance_bef = ultrasonic_dist;

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

  //Updates time of flight variables
  tof_distance_bef = tof_distance;
  tof_distance = sensor.getDistance();

  if ( tof_distance != 0 and tof_distance != 2047.5){ //Checks if it is reporting 0 or max to exclude that data
    tof_change = tof_distance - tof_distance_bef;
  }
  //Updates ultrasonic variables
  us_distance_bef = ultrasonic_dist;
  ultrasonic_dist = analogRead(ultrasonic_pin) * 10 * MAX_RANG / ADC_SOLUTION;
  if ( ultrasonic_dist != 0){ //Checks if it is reporting 0 or max to exclude that data
    us_change = ultrasonic_dist - us_distance_bef;
  }


  if (val == HIGH) { // check if the input is HIGH
    start = !start;
    counter = 0;
    delay(500);
    state = 0;
  }


  ///First cycle initialising states and moving forwards


  if (start){
    switch (state){

    case 0:
  // Sweeping
    sweeping();
    moving = true;
    break;

    case 1:

    // Hunting
    hunting();
    moving = true;
    //Grab if distance is small
    break;

    case 2:
    static int low_dens_count = 0;
    static int high_dens_count = 0;
    // Identifying
    moving = false;

    //Checks the identifying function and records how many times it says high or low
    if (identifying() == 1){
      low_dens_count += 1;
    }else{
      if (identifying() == 2){
        high_dens_count += 1;
      }
    }
    if(high_dens_count + low_dens_count == 20){   //Once it has checked 20 times

      if (high_dens_count > 10){     //If it has been high desnity more it lights red LED
        digitalWrite(led_red, HIGH);
      }else{      
        digitalWrite(led_green, HIGH);
        }
      delay(5000);    //Waits 5 seconds
      digitalWrite(led_red, LOW);  //Resets everything
      digitalWrite(led_green, LOW);
      high_dens_count = 0;
      low_dens_count = 0;   
  }
    break;


    case 3:

    // Delivering
    moving = true;
    break;

    case 4:

    // Returning
    moving = true;
    break;
  }}

  //Turns LED on if its moving
  if(moving == true){
      if(counter % 50 < 25){
        digitalWrite(led_blue, HIGH);
      }else{
          digitalWrite(led_blue, LOW);
  }}else{
      digitalWrite(led_blue, LOW);  
  }

  

  if (counter > 2000 or start == false){
    straight_ahead(0);
    moving = false;
    start = false;
  }

  counter +=1 ;
  delay(10);
}


// Range of functions that let it drive around and do stuff
  void line_turn_check(){

    int valfarright = digitalRead(farrightlinesensorPin); // read far right input value
    int valfarleft = digitalRead(farleftlinesensorPin); // read far right input value
    int turn_time = 5;   //Delay between spotting a turn and the latest it could take that turn


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

  }
  void follow_line(){

    line_turn_check();
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
        if (left_turn == true){
          turn_left();
    }}}
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
//
//  State 0 main function
void sweeping(){

    Serial.println("SWEEPING!!!!");

      if (counter % 300 == 0){ //every three seconds the rotation of robot changes
        if (counter % 300 == 0){ //robot starts rotating right at the start
          rotate_right();
        }
      else{ 
        rotate_left(); //robot rotates left after 3 seconds
        }
      }

    //If theres a big drop in distance then it starts hunting
    if (us_change < -50 and counter > 5){
      state = 1;
    }

    /////////////NEED A CHECK FOR IF IT DOESNT FIND ANYTHING SO IT DOESNT CSRRY ON FOREVER
}
//
//  State 1 main function
void hunting(){
  // Drives ahead towards block
  straight_ahead(255);
  // If it no longer sees the block it starts to sweep
  if (us_change > 0){
    state = 0;
  }
  if (tof_distance < 80){
    state = 2;
    straight_ahead(0);
  }

}

void collection(){
  //Grab block
}

int identifying(){

  if (tof_distance > 150){
    return 0;
  }else{
    if (ultrasonic_dist >500 and sensor.getDistance()<150) {  //use US sensor to determine density
      return 1;     //low dens
    }
  else {
    return 2;    //high dens
  }
}}
