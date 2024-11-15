/*
IDP
LIGHT FLASHING CODE
*/

//LEDs are on pins 3, 4, and 5
int LED_BLUE = 3;
int LED_GREEN = 4;
int LED_RED = 5;

//time variable, time since last LED flash
unsigned long time_since_led;

//variable is on when robo is moving, off when it is still
bool isMoving; 

//after block is detected, switch var for if it is or isnt magnetic
bool isMagnetic;

void setup() {
  Serial.begin(9600);
  //set all LEDs to output
  pinMode(LED_BLUE, OUTPUT); 
  pinMode(LED_GREEN, OUTPUT); 
  pinMode(LED_RED, OUTPUT); 

}
void loop() {
  isMoving = true;
  flashLedWhenMoving( isMoving );

  isMagnetic = true;
  shineLedBlockType( isMagnetic );

  delay(2000);

  turnLedsOff();
}


void flashWhenMoving( bool isMoving){

  if ( !isMoving ){ //if the robot isn't moving, turn the blue pin OFF
    digitalWrite(LED_BLUE, LOW);
    return false;
  }

  //call this function when the motors running, it will flash the led every 1000 ms
  if ( (millis() - time_since_led) > 1000){ //if the time since we last changed the LED output is > 100, change it and reset the count
    bool isHigh = digitalRead(LED_BLUE);
    digitalWrite(LED_BLUE, !isHigh);
    time_since_led = millis(); //reset the time for the next interval
    return true;
  }

}

void shineLedBlockType( bool isMagnetic ){
  if (isMagnetic){ 
    digitalWrite(LED_RED, HIGH ); 
  }else{
    digitalWrite( LED_GREEN, HIGH );
  }

}

void turnLedsOff(){}
