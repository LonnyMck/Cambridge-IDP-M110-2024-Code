/*
Test for motors in chassis
*/

#include <Adafruit_MotorShield.h>
#include <Servo.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor = AFMS.getMotor(2);
// You can also make another motor on port M2
Adafruit_DCMotor *Motor2 = AFMS.getMotor(3);


Servo controlservo; // create servo object to control a servo
int servopos = 0; // variable to store the servo position

bool running = true;
char input;

double speed;
double duration;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor func test");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Motor Shield found.");

  controlservo.attach(2); // attaches the servo on pin 2 to the servo object

  runServo(0); //Check that the servo is homed

}

void loop() {

  running = check_interrupt(); //Check for interrupt
  if (running) {  //No interrupt has been detected

  /*
  runServo(100);
  runServo(200);
  runServo(100);
  runServo(0);
  

  
  runMotor(150, 1, Motor);
  runMotor(150, 1, Motor2);
  delay(1000);
  runMotor(0, 1, Motor);
  runMotor(0, 1, Motor2);
  delay(1000);
  */

  runForwards(10);

  }}



///////////////////////////
//Declaring functions
///////////////////////////

int runForwards(int distance){  //Higher level function taht will require calibration. Robot moves forwards a certain distance in cm. 

speed = 100;  //Work out best value through testing, or add as argument to func.
duration = distance / (speed * 2 * PI * 3); //Need to measure wheels more exactly

runMotor(speed, 1, Motor);
runMotor(speed, 1, Motor2);

delay(duration);

runMotor(0, 1, Motor);
runMotor(0, 1, Motor2);

}



int runMotor(int speed, bool direction, Adafruit_DCMotor *motorObject){ // direction should be 1 for Forward, 0 for Backward. Speed should be an int between 0 and 255.
  
  running = check_interrupt(); //Check for interrupt

  Serial.println("Change in Motor");

  if (speed == 0){
    motorObject->run(RELEASE);
    Serial.println("Stopping");
  }

  else if (direction == 1){
    motorObject->setSpeed(speed);
    motorObject->run(FORWARD);
    Serial.println("Moving forward at " + String(speed));
  } 

  else if (direction == 0){
    motorObject->setSpeed(speed);
    motorObject->run(BACKWARD);
    Serial.println("Moving backward at " + String(speed));
  }


  else {
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


int check_interrupt(){ // checks for interrupts and breaks loop. Returns boolean. Will eventually be an Estop.

  if(Serial.available()){ // Adds keyboard interrupt
        input = Serial.read();
      }
    
    if(input == 't'){
     Serial.println("Interrupt");
     runMotor(0, 1, Motor);
     runMotor(0, 1, Motor2);
     runServo(0);
     while(1);  //Get stuck in an endless loop and doesn't execute any new code
    }
    return true;
}