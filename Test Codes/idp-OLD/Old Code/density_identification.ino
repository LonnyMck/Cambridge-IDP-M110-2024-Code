
#include "Arduino.h"      //include necessary libraries
#include "Wire.h"
#include "DFRobot_VL53L0X.h"
DFRobot_VL53L0X sensor;

#define MAX_RANG (520)//the max measurement value of the module is 520cm(a little bit longer than effective max range)
#define ADC_SOLUTION (1023.0)//ADC accuracy of Arduino UNO is 10bit

int sensityPin = A5; // select the pins
int GreenLedPin = 2;
int RedLedPin = 3;
int BlueLedPin = 1;
int button = 4;

void setup() {
 // Serial init
  Serial.begin(9600);

  pinMode(GreenLedPin, OUTPUT); // declare LED as output
  pinMode(button,INPUT);        // use button to start detection, temporary - replace later

 Serial.begin(115200);
 //join i2c bus (address optional for master)
 Wire.begin();
 //Set I2C sub-device address
 sensor.begin(0x50);
 //Set to Back-to-back mode and high precision mode
 sensor.setMode(sensor.eContinuous,sensor.eHigh);
 //Laser rangefinder begins to work
 sensor.start();                        //TOF sensor
}

float dist_t, sensity_t, density;

void loop() {
 // read the value from the sensor:
sensity_t = analogRead(sensityPin);
 // turn the ledPin on
dist_t = sensity_t * MAX_RANG / ADC_SOLUTION;   //distance from US sensor
Serial.print(dist_t,0);
Serial.println("cm");
Serial.print(sensor.getDistance());

int val = digitalRead(button); // read input value from button

if (val == 1) {
      if (dist_t <10 or dist_t >519 and sensor.getDistance()<100) {  //use US sensor to determine density
        density = 0;     //low dens
      }
      else {
        density = 1;    //high dens
      }
}
if (density = 0) {
        digitalWrite(GreenLedPin,HIGH); //Light LED
        digitalWrite(RedLedPin,LOW);
        delay(5000);
}
if (density =1) {
        digitalWrite(RedLedPin,HIGH);   //Light LED
        digitalWrite(GreenLedPin,LOW);
        delay(5000);
}
digitalWrite(RedLedPin,LOW);       //Turn off LED
digitalWrite(GreenLedPin,LOW);
delay(100);
}
