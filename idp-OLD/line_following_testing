/* Cambridge University IDP, Team M109, 2023 */
/*************************** HEADER ********************************************/
#include <Adafruit_MotorShield.h>
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
enum block_type {NONE, LOW_DENSITY, HIGH_DENSITY};
/* Exported function prototypes */
void straight_ahead(int speed = 255);
void straight_backwards(int speed = 255);
void forwards_right(int speed = 255);
void forwards_left(int speed = 255);
void turn_right(int speed = 255);
void turn_left(int speed = 255);
void rotate_right(int speed = 255);
void rotate_left(int speed = 255);
void line_turn_check();
void follow_line();
void line_sensor_read();
void ultrasound_update();
void tof_update();
void sweeping();
void hunting();
void identifying();
void pick_block(int speed);
void release_block(int speed);
void mini_sweep();
block_type density_check();
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// the two motors
Adafruit_DCMotor *wheel_left = AFMS.getMotor(1); //the left wheel
Adafruit_DCMotor *wheel_right = AFMS.getMotor(2); // the right wheel
/* Claw Motor */
#define CLAW_CLOSE_DISTANCE    2.3 * 255// Distance in arbitrary units
Adafruit_DCMotor *claw_motor = AFMS.getMotor(3); // The motor for the claw
//Initialise time of flight
DFRobot_VL53L0X sensor;
#define MAX_RANG (5200)         //the max measurement value of the module is 520cm(a little bit longer than effective max range)
#define ADC_SOLUTION (1023.0)   //ADC accuracy of Arduino UNO is 10bit
#define LEFT_WHEEL_RATIO (0.95)  //Variables to allow easy calibration of wheels
#define RIGHT_WHEEL_RATIO (1)

//Initialise all pin inputs
#define PIN_INPUT                     4 // Button pin
#define PIN_LED_BLUE                  5
#define PIN_LED_GREEN                 6
#define PIN_LED_RED                   7
#define PIN_FAR_LEFT_LINE_SENSOR      8 
#define PIN_LEFT_LINE_SENSOR          9
#define PIN_RIGHT_LINE_SENSOR         10
#define PIN_LEFT_LINE_SENSOR          10
#define PIN_RIGHT_LINE_SENSOR         9
#define PIN_FAR_RIGHT_LINE_SENSOR     11 

#define PIN_ULTRASOUND                A1
enum claw_pos {OPEN = 0, CLOSED = 1};
claw_pos claw_status = OPEN;
enum state_type {SWEEPING, HUNTING, IDENTIFYING, DELIVERING, RETURNING,FOLLOW};
state_type state = 0;
int us_distance_bef;
int us_change;
int ultrasonic_dist;
int tof_distance_bef;
int tof_distance;
int tof_change;
//int bearings = 0; // angle of rotation taken from the starting position, assumed north
int counter = 0; //temporary variable to count the amount of loops the code goes through
bool density = 0;
//int time_loop = 500; //variable used to control the motor, I can't explain it well just ignore it
bool start = false; //changes to TRUE and movement starts when button pressed
unsigned short line_sensor_prev = 0b0000; // The previous status of the 4 line sensors (e.g middle 2 on would be 0110)
unsigned short line_sensor = 0b0000; // The status of the 4 line sensors (e.g middle 2 on would be 0110)
bool right_turn = false;
int right_turn_time = 0;
bool left_turn = false;
int left_turn_time = 0;
bool forward_line = true;
int forward_line_time = 0;
bool line_straight = true;
enum junc_type {NO_LINE, LINE, T_JUNC, LEFT_BEND, RIGHT_BEND, LEFT_SIDE, RIGHT_SIDE, CROSS} junction;
bool bump = false;
bool moving;
void setup() {
  // Set up Serial library at 9600 bps
  Serial.begin(9600);           
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
 //   while (1);
  }
  Serial.println("Motor Shield found.");
  // Set the speed to start, from 0 (off) to 255 (max speed)
  // Turn on motor
  wheel_left->run(RELEASE);
  wheel_right->run(RELEASE);
// Declares pushbutton  and line sensors as inputs
  pinMode(PIN_INPUT, INPUT); 
  pinMode(PIN_LEFT_LINE_SENSOR, INPUT); 
  pinMode(PIN_RIGHT_LINE_SENSOR, INPUT); 
  pinMode(PIN_FAR_RIGHT_LINE_SENSOR, INPUT); 
  pinMode(PIN_FAR_LEFT_LINE_SENSOR, INPUT); 
// Declares all leds as outputs
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
// Initialising time of flight sensor
  Wire.begin();
  sensor.begin(0x50);
  sensor.setMode(sensor.eContinuous,sensor.eHigh);
  sensor.start();
  delay(100);
}
/************************** MAIN LOOP *****************************/
void loop() {
  // Serial.println(ultrasonic_dist);
  // Serial.println(tof_distance);
  // Serial.println(tof_change);
  // Sensor initialisation
  line_sensor_read();
  tof_update();
  ultrasound_update();
  /* BUTTON START CONDITION */
  if (digitalRead(PIN_INPUT) == HIGH) { // check if the input is HIGH
    start = !start;
    release_block(255);
    counter = 0;
    delay(500);
    state = FOLLOW;
  }
  // First cycle initialising states and moving forwards
  if (start) {
    switch (state) {
    case SWEEPING:
      // Sweeping
      sweeping();
      moving = true;
      digitalWrite(PIN_LED_RED, LOW);  
      digitalWrite(PIN_LED_GREEN, HIGH);
      Serial.println("SWEEPING");
      break;
    case HUNTING:
      // Hunting
      hunting();
      moving = true;
      digitalWrite(PIN_LED_RED, HIGH);  
      digitalWrite(PIN_LED_GREEN, LOW);
      Serial.println("HUNTING");
      break;
    case IDENTIFYING:
      Serial.println("IDENTIFYING");
      identifying();
      break;
    case DELIVERING:
      Serial.println("DELIVERING");
      // Delivering
      moving = true;
      /* TODO: INSERT DELIVERY FUNCTION */
      release_block(255);
      delay(5000);
      state = RETURNING;
      break;
    case RETURNING:
      Serial.println("RETURNING");
      // Returning
      moving = true;
      /* TODO: INSERT RETURN FUNCTION */
      delay(5000);
      state = SWEEPING;
      break;
    case FOLLOW:
      follow_line();
    }
  }
  // Turns LED on if its moving
  if (moving == true) {
    if (counter % 50 < 25) {
      digitalWrite(PIN_LED_BLUE, HIGH);
    } else {
        digitalWrite(PIN_LED_BLUE, LOW);
    }
  } else {
    digitalWrite(PIN_LED_BLUE, LOW);  
  }
  if (counter > 5000 or start == false) {
    straight_ahead(0);
    moving = false;
    start = false;
    digitalWrite(PIN_LED_RED, LOW);  //Resets everything
    digitalWrite(PIN_LED_GREEN, LOW);
  }
  counter++;
  delay(10);
}
// Arduino was being funny and not letting these be after the loop so they're here now
void straight_ahead(int speed = 255) {   // Drives forwards at max speed
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(BACKWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}
void straight_backwards(int speed = 255) {   // Drives backwards at max speed
  wheel_left->run(FORWARD);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(FORWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}
void veer_right(int speed = 255) {   //Continues moving forwards while drifting to the right
    wheel_right->run(BACKWARD);
    wheel_left->run(BACKWARD);
    wheel_left->setSpeed(0.7 * speed * LEFT_WHEEL_RATIO);
    wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO );
}
void veer_left(int speed = 255) {   //Continues moving forwards while drifting to the left
    wheel_right->run(BACKWARD);
    wheel_left->run(BACKWARD);
    wheel_right->setSpeed(0.7 * speed * RIGHT_WHEEL_RATIO);
    wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
}

void maintain_line(int speed = 255) {
  wheel_right->run(BACKWARD);
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(0.8 * speed * LEFT_WHEEL_RATIO);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO );
}

void turn_right(int speed = 255) {    //Turns quickly to the right until it hits line
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(FORWARD);   ///////
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO*0.9);
  // if (line_sensor & 0b1000) {
  //   right_turn = false;
  //   wheel_right->run(BACKWARD);
  //   wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
  // }
}
void turn_left(int speed = 255) {      //Turns quickly to the left until it hits line
  wheel_right->run(BACKWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
  wheel_left->run(FORWARD);    /////
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  // if (line_sensor & 0b0001){
  //   left_turn = false;
  //   wheel_left->run(BACKWARD);
  //   wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  // }
}
void rotate_right(int speed = 255) {      //Steadily turns on the spot to the right
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(FORWARD);    
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}
void rotate_left(int speed = 255) {         //Steadily turns on the spot to the left
  wheel_left->run(FORWARD);   
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(BACKWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}
void rotate_180(int speed = 255) {
  wheel_left->run(FORWARD);   
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(BACKWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
  delay(2000);
}
/******* MOVEMENT FUNCTIONS *********/
/*!
 * \fn void line_turn_check()
 * \brief Runs a 
*/
void line_turn_check() {
  int turn_time = 7;   //Delay between spotting a turn and the latest it could take that turn
  static int line_sensor_snapshot = 0b0000;

  if ((line_sensor ^ line_sensor_prev) & 0b1001) {
    left_turn_time = counter;
    line_sensor_snapshot = line_sensor | line_sensor_prev;
  }
  
  if (counter - left_turn_time == turn_time) { //5 Cycles later check what type of intersection it has hit
    if (!line_sensor) { // If now all sensors are reading nothing
      switch (line_sensor_snapshot | 0b0110) {
        case 0b1110:
          junction = LEFT_BEND;
          Serial.println("Left Bend");
          break;
        case 0b0111:
          junction = RIGHT_BEND;
          Serial.println("Right Bend");
          break;
        case 0b1111:
          junction = T_JUNC;
          Serial.println("T Junction");
          break;
        default:
          junction = NO_LINE;
          Serial.println("No Line???");
      }
    } else if (line_sensor & 0b0110) { // Checks if middle sensors are on
      switch (line_sensor_snapshot | 0b0110) {
        case 0b1110:
          junction = LEFT_SIDE;
          break;
        case 0b0111:
          junction = RIGHT_SIDE;
          break;
        case 0b1111:
          junction = CROSS;
          break;
        default:
          junction = LINE;
          break;
      }
    }
  }
}

void follow_line() {
  line_sensor_read();
  line_turn_check();
  Serial.println(line_sensor, BIN);
  Serial.println(junction);
   switch (junction) {
    /* Use line following */
    /* Turn Left Cases */

    default:
      if (~line_sensor & 0b0100) { // If sensor 2 is off
        // Serial.println("veer left");

        veer_right(220);
      } else if (~line_sensor & 0b0010) {// If sensor 3 is off
        // Serial.println("veer right");

        veer_left(220);
      } else {
        maintain_line(180);
      }
      break;
  }
}
/*************** ROBOT STATE FUNCTIONS *************/
/*!
 * \fn void sweeping()
 * \brief Sweeping state main function
*/  
void sweeping() {
  bool rotation_direction = 1;
  if (rotation_direction == 1) {
    rotate_right();
  } else {
    rotate_left();
  }
    // if (counter % 300 == 0){ //every three seconds the rotation of robot changes
    //   if (counter % 300 == 0){ //robot starts rotating right at the start
    //     rotate_right();
    //   }
    // else{ 
    //   rotate_left(); //robot rotates left after 3 seconds
    //   }
    // }
  //If theres a big drop in distance then it starts hunting
  if (us_change < -200){
    delay(50);  // Slightly overrotates as it will be detected on the edge and we want middle
    state = HUNTING;
    rotation_direction = !rotation_direction;
  }
  /////////////NEED A CHECK FOR IF IT DOESNT FIND ANYTHING SO IT DOESNT CSRRY ON FOREVER
}
/*!
 * \fn void hunting()
 * \brief Sweeping state main function
*/  
void hunting() {
  // Drives ahead towards block
  straight_ahead(255);
  // If it no longer sees the block it starts to sweep
  if (us_change > 200) {
    mini_sweep();
  }
  if (tof_distance < 80) {
    straight_ahead(0);
    state = IDENTIFYING;
  }
}
void identifying() {
  static int low_dens_count = 0;
  static int high_dens_count = 0;
  // Identifying
  moving = false;
  if(low_dens_count + high_dens_count == 0) {
    pick_block(255);
    release_block(255);
  }
  // Checks the identifying function and records how many times it says high or low
  if (density_check() == LOW_DENSITY) {
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
    low_dens_count += 1;
  } else {
    if (density_check() == HIGH_DENSITY) {
      digitalWrite(PIN_LED_RED, HIGH);
      digitalWrite(PIN_LED_GREEN, LOW);
      high_dens_count += 1;
    } else {
      digitalWrite(PIN_LED_RED, HIGH);  //PANIC
      digitalWrite(PIN_LED_GREEN, HIGH);
      state = SWEEPING;
    }
  }
  if (high_dens_count + low_dens_count == 100) {   // Once it has checked 20 times
    if (high_dens_count > 50) {     // If it has been high desnity more it lights red LED
      digitalWrite(PIN_LED_RED, HIGH);
      digitalWrite(PIN_LED_GREEN, LOW);
      density = HIGH;
    } else {      
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);
      density = LOW;
    }
    pick_block(255);
    delay(5000);    // Waits 5 seconds
    digitalWrite(PIN_LED_RED, LOW);  //Resets everything
    digitalWrite(PIN_LED_GREEN, LOW);
    // Resets counters for how many times each
    high_dens_count = 0;
    low_dens_count = 0;   
    state = DELIVERING;
  }
}
/*!
 * \fn pick_block(int speed)
 * \brief runs the motors to pick the block
 * \param speed speed of grabber (0-255) 
*/
void pick_block(int speed = 255) {
  if (claw_status == OPEN) {
    claw_motor->setSpeed(speed);
    claw_motor->run(FORWARD);
    delay(CLAW_CLOSE_DISTANCE * 1000.0 / speed);
    claw_motor->setSpeed(0);
    claw_motor->run(FORWARD);
    claw_status = CLOSED;
  } else {
      // serial.ln("Claw already closed");
  }
}
/*!
 * \fn release_block(int speed)
 * \brief runs the motors to release the block
 * \return type of block identified (block_type)
*/
block_type density_check() {
  // Serial.print("TOF:");
  // Serial.println(tof_distance);
  // Serial.print("US:");
  // Serial.println(ultrasonic_dist);
  if (tof_distance > 150){
    return NONE;
  }else{
    if (ultrasonic_dist >5000 or ultrasonic_dist < 150) {  //use US sensor to determine density
      return LOW_DENSITY;     //low dens
    }
  else {
    return HIGH_DENSITY;    //high dens
  }
}}
/*!
 * \fn void release_block(int speed)
 * \brief Runs the motors to release the block
 * \param speed speed of grabber (0-255) 
*/
void release_block(int speed = 255) {
  if (claw_status == CLOSED) {
    claw_motor->setSpeed(speed);
    claw_motor->run(BACKWARD);
    delay(CLAW_CLOSE_DISTANCE * 1000.0 / speed);
    claw_motor->setSpeed(0);
    claw_motor->run(BACKWARD);
    claw_status = OPEN;
  } else {
    Serial.println("Claw already open");
  }  
}
void line_sensor_read() {
  line_sensor_prev = line_sensor;
  line_sensor = 0b0000;
  if (digitalRead(PIN_FAR_LEFT_LINE_SENSOR) > 0.5) {
    line_sensor |= 0b1000;
  }
  if (digitalRead(PIN_LEFT_LINE_SENSOR) > 0.5) {
    line_sensor |= 0b0100;
  }
  if (digitalRead(PIN_RIGHT_LINE_SENSOR) > 0.5) {
    line_sensor |= 0b0010;
  }
  if (digitalRead(PIN_FAR_RIGHT_LINE_SENSOR) > 0.5) {
    line_sensor |= 0b0001;
  }
}
void mini_sweep() {
  Serial.println("Mini Sweep");
  int mini_counter = 0;
  while (mini_counter < 301) {
    if (mini_counter < 100) {
    rotate_left(100);  
    } else {
    rotate_right(100);
    }
  

    ultrasound_update();

    if (us_change < -200){
    if (us_change < -200) {
      delay(100);  // Slightly overrotates as it will be detected on the edge and we want middle
      straight_ahead(0);
      state = HUNTING;
      mini_counter = 500;
    } else {
      if (mini_counter == 300) {
        state = SWEEPING;
      }
    }
    mini_counter++;
    delay(10);
    }
  }
}
/*!
 * \fn void ultrasound_update();
 * \brief refreshes us_distance_bef and us_distance
*/
void ultrasound_update() {
  // Updates ultrasonic variables
  us_distance_bef = ultrasonic_dist;
  ultrasonic_dist = analogRead(PIN_ULTRASOUND) * (MAX_RANG / ADC_SOLUTION);
  if (ultrasonic_dist != 0) { //Checks if it is reporting 0 or max to exclude that data
    us_change = ultrasonic_dist - us_distance_bef;
  }
}
/*!
 * \fn void tof_update()
 * \brief refreshes tof_distance_bef and tof_distance
*/
void tof_update() {
  tof_distance_bef = tof_distance;
  if (sensor.getDistance() > 0) {
    tof_distance = sensor.getDistance();
  }
  if (tof_distance > 0 and tof_distance < 2047) { //Checks if it is reporting 0 or max to exclude that data
    tof_change = tof_distance - tof_distance_bef;
  }
}