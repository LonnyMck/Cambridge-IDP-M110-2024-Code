/* Cambridge University IDP, Team M109, 2023 */
/*************************** HEADER ********************************************/
#include <Adafruit_MotorShield.h>
#include "Wire.h"

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

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// the two motors
Adafruit_DCMotor *wheel_left = AFMS.getMotor(1);   //the left wheel
Adafruit_DCMotor *wheel_right = AFMS.getMotor(2);  // the right wheel
/* Claw Motor */
#define CLAW_CLOSE_DISTANCE 2.3 * 255             // Distance in arbitrary units
Adafruit_DCMotor *claw_motor = AFMS.getMotor(3);  // The motor for the claw

#define MAX_RANG (5200)          //the max measurement value of the module is 520cm(a little bit longer than effective max range)
#define ADC_SOLUTION (1023.0)    //ADC accuracy of Arduino UNO is 10bit
#define LEFT_WHEEL_RATIO (0.95)  //Variables to allow easy calibration of wheels
#define RIGHT_WHEEL_RATIO (1)

//Initialise all pin inputs
#define PIN_INPUT 2  // Button pin
#define PIN_LED_BLUE 3
#define PIN_LED_GREEN 4
#define PIN_LED_RED 5
#define PIN_FAR_LEFT_LINE_SENSOR 8
#define PIN_LEFT_LINE_SENSOR 9
#define PIN_RIGHT_LINE_SENSOR 10
#define PIN_FAR_RIGHT_LINE_SENSOR 11

#define PIN_ULTRASOUND A1
enum claw_pos { OPEN = 0,
                CLOSED = 1 };
claw_pos claw_status = OPEN;
enum state_type { SWEEPING,
                  HUNTING,
                  IDENTIFYING,
                  DELIVERING,
                  RETURNING,
                  FOLLOW };
state_type state = 0;

//int bearings = 0; // angle of rotation taken from the starting position, assumed north
int counter = 0;  //temporary variable to count the amount of loops the code goes through
bool density = 0;
//int time_loop = 500; //variable used to control the motor, I can't explain it well just ignore it
bool start = false;                        //changes to TRUE and movement starts when button pressed
unsigned short line_sensor_prev = 0b0000;  // The previous status of the 4 line sensors (e.g middle 2 on would be 0110)
unsigned short line_sensor = 0b0000;       // The status of the 4 line sensors (e.g middle 2 on would be 0110)
bool right_turn = false;
int right_turn_time = 0;
bool left_turn = false;
int left_turn_time = 0;
bool forward_line = true;
int forward_line_time = 0;
bool line_straight = true;
enum junc_type { NO_LINE,
                 LINE,
                 T_JUNC,
                 LEFT_BEND,
                 RIGHT_BEND,
                 LEFT_SIDE,
                 RIGHT_SIDE,
                 CROSS } junction;
bool bump = false;
bool moving;
void setup() {
  // Set up Serial library at 9600 bps
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  if (!AFMS.begin()) {  // create with the default frequency 1.6KHz
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

}
/************************** MAIN LOOP *****************************/
void loop() {
  // Sensor initialisation
  line_sensor_read();
  /* BUTTON START CONDITION */
  if (digitalRead(PIN_INPUT) == HIGH) {  // check if the input is HIGH
    start = !start;
    
    counter = 0;
    delay(500);
    state = FOLLOW;
  }
  // First cycle initialising states and moving forwards
  if (start) {
    //switch (state) {
    // case SWEEPING:
    //   // Sweeping
    //   sweeping();
    //   moving = true;
    //   digitalWrite(PIN_LED_RED, LOW);
    //   digitalWrite(PIN_LED_GREEN, HIGH);
    //   Serial.println("SWEEPING");
    //   break;
    // case HUNTING:
    //   // Hunting
    //   hunting();
    //   moving = true;
    //   digitalWrite(PIN_LED_RED, HIGH);
    //   digitalWrite(PIN_LED_GREEN, LOW);
    //   Serial.println("HUNTING");
    //   break;
    // case IDENTIFYING:
    //   Serial.println("IDENTIFYING");
    //   identifying();
    //   break;
    // case DELIVERING:
    //   Serial.println("DELIVERING");
    //   // Delivering
    //   moving = true;
    //   /* TODO: INSERT DELIVERY FUNCTION */
    //   
    //   delay(5000);
    //   state = RETURNING;
    //   break;
    // case RETURNING:
    //   Serial.println("RETURNING");
    //   // Returning
    //   moving = true;
    //   /* TODO: INSERT RETURN FUNCTION */
    //   delay(5000);
    //   state = SWEEPING;
    //   break;
    // case FOLLOW:
    follow_line();
  }

delay(10);
}

// Arduino was being funny and not letting these be after the loop so they're here now
void straight_ahead(int speed = 255) {  // Drives forwards at max speed
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(BACKWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}
void straight_backwards(int speed = 255) {  // Drives backwards at max speed
  wheel_left->run(FORWARD);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(FORWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}
void veer_right(int speed = 255) {  //Continues moving forwards while drifting to the right
  wheel_right->run(BACKWARD);
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(0.7 * speed * LEFT_WHEEL_RATIO);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}
void veer_left(int speed = 255) {  //Continues moving forwards while drifting to the left
  wheel_right->run(BACKWARD);
  wheel_left->run(BACKWARD);
  wheel_right->setSpeed(0.7 * speed * RIGHT_WHEEL_RATIO);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
}

void maintain_line(int speed = 255) {
  wheel_right->run(BACKWARD);
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(0.8 * speed * LEFT_WHEEL_RATIO);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}

void turn_right(int speed = 255) {  //Turns quickly to the right until it hits line
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(FORWARD);  ///////
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO * 0.9);
  // if (line_sensor & 0b1000) {
  //   right_turn = false;
  //   wheel_right->run(BACKWARD);
  //   wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
  // }
}
void turn_left(int speed = 255) {  //Turns quickly to the left until it hits line
  wheel_right->run(BACKWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
  wheel_left->run(FORWARD);  /////
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  // if (line_sensor & 0b0001){
  //   left_turn = false;
  //   wheel_left->run(BACKWARD);
  //   wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  // }
}
void rotate_right(int speed = 255) {  //Steadily turns on the spot to the right
  wheel_left->run(BACKWARD);
  wheel_left->setSpeed(speed * LEFT_WHEEL_RATIO);
  wheel_right->run(FORWARD);
  wheel_right->setSpeed(speed * RIGHT_WHEEL_RATIO);
}
void rotate_left(int speed = 255) {  //Steadily turns on the spot to the left
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
  int turn_time = 7;  //Delay between spotting a turn and the latest it could take that turn
  static int line_sensor_snapshot = 0b0000;

  if ((line_sensor ^ line_sensor_prev) & 0b1001) {
    left_turn_time = counter;
    line_sensor_snapshot = line_sensor | line_sensor_prev;
  }

  if (counter - left_turn_time == turn_time) {  //5 Cycles later check what type of intersection it has hit
    if (!line_sensor) {                         // If now all sensors are reading nothing
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
    } else if (line_sensor & 0b0110) {  // Checks if middle sensors are on
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
}
//   Serial.println(line_sensor, BIN);
//   Serial.println(junction);
//   switch (junction) {
//     /* Use line following */
//     /* Turn Left Cases */
//     case LEFT_BEND:

//       turn_left(180);
//       delay(1000);
//       straight_ahead();
//       junction = LINE;
//       break;

//         turn_right();
//         turn_right();
  
//       junction = LINE;
//       break;
//     case LEFT_SIDE:

//         turn_left();
//         turn_left();
      
//       junction = LINE;
//       break;

//     case T_JUNC:
//       if (density == HIGH_DENSITY) {
//         turn_left();
//       } else {
//         turn_right();
//       }
//       junction = LINE;
//       break;
//     default:
//       if (~line_sensor & 0b0100) {  // If sensor 2 is off
//         // Serial.println("veer left");

//         veer_right(220);
//       } else if (~line_sensor & 0b0010) {  // If sensor 3 is off
//         // Serial.println("veer right");

//         veer_left(220);
//       } else {
//         maintain_line(180);
//       }
//       break;
//   }
// }
/*************** ROBOT STATE FUNCTIONS *************/

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
