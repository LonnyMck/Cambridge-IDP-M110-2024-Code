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
  pinMode(leftlinesensorPin, INPUT); // declare LED as output
  pinMode(rightlinesensorPin, INPUT); // declare Micro switch as input
  delay(100);
}

void loop() {

int val = digitalRead(inputPin); // read input value
int valleft = digitalRead(leftlinesensorPin); // read left input value
int valright = digitalRead(rightlinesensorPin); // read right input value

bool left_flag;
bool right_flag;


if (val == HIGH) { // check if the input is HIGH
  start = true;
}


  if (counter < 2 and start == true){ //counter goes up every 10 ms so this loop runs for 5000 ms, or 5 seconds
    wheel_left->run(BACKWARD);
    wheel_left->setSpeed(255);

    wheel_right->run(BACKWARD);
    wheel_right->setSpeed(255);

    left_flag = false;
    right_flag = false;

    Serial.println(x_step);

  }else{

    
    //wheel_right->setSpeed(150*valleft)
    //wheel_left->setSpeed(150)
    if (counter < 2000 and start == true){

      wheel_right->run(BACKWARD);
      wheel_left->run(BACKWARD);
      if (valleft > 0.5 and valright < 0.5){    //left is on line, right is off line
      //It has turned to the right so needs to go left
      wheel_left->setSpeed(200);
      wheel_right->setSpeed(255);
      right_flag = true;
      }else{
        if (valright > 0.5 and valleft < 0.5){   // Has moved to the left
          wheel_left->setSpeed(255);
          wheel_right->setSpeed(200);
          left_flag = true;
          right_flag = false;
      }else{
        if (valright > 0.5 and valleft > 0.5){   // Moving on the line
          wheel_left->setSpeed(255);
          wheel_right->setSpeed(255);
      }else{
        if (valright < 0.5 and valleft < 0.5){   // Completely off line
          if (left_flag == true)
          }
        }
        }
        }
      }else{
      wheel_left->setSpeed(0);
      wheel_right->setSpeed(0);
    }
    }
  counter +=1 ;
  x_step += 1;
  delay(10);
}