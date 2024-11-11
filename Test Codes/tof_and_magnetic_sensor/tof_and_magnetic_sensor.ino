#include "Arduino.h"
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X sensor;

//sensor mag consts
float BLOCK_CLOSE = 75; //
int SENSOR_MAG = 9;


int LED_BLUE = 11;
int LED_GREEN = 12;
int LED_RED = 13;


//after block is detected, switch var for if it is or isnt magnetic
bool isMagnetic;

void setup() {
  //initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  sensor.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  sensor.setMode(sensor.eContinuous, sensor.eHigh);
  //Laser rangefinder begins to work
  sensor.start();


  pinMode(SENSOR_MAG, INPUT); //sets magnetic to input

    //set all LEDs to output
  pinMode(LED_BLUE, OUTPUT); 
  pinMode(LED_GREEN, OUTPUT); 
  pinMode(LED_RED, OUTPUT); 
}
void loop() {
  //Get the distance
  Serial.print("Distance: ");
  Serial.print(sensor.getDistance());

  isMagnetic = digitalRead(SENSOR_MAG);
  Serial.print(", Magnetic: ");
  Serial.println( digitalRead(SENSOR_MAG) );
  shineLedBlockType( digitalRead(SENSOR_MAG) );
  

  delay(500);
}



//shine correct LED depending on if magnetic or non magnetic

void shineLedBlockType( bool isMagnetic ){
  if (isMagnetic){ 
    digitalWrite(LED_RED, HIGH ); 
    digitalWrite(LED_GREEN, LOW ); 
  }else{
    digitalWrite( LED_GREEN, HIGH );
    digitalWrite(LED_RED, LOW ); 
  }

}

//turns all LEDs off
void turnLedsOff(){
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, LOW);
}
