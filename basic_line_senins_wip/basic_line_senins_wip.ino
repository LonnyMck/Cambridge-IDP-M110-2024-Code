/*
IDP
David Paterson
Line Sensor Module Example Code V1
Move the line sensor across the black and white line, monitor on serial
*/

const int sensorPinLL = 10;
const int sensorPinL = 11;
const int sensorPinR = 12;
const int sensorPinRR = 13;

// Sensor states
int sensorStateLL = 0;
int sensorStateL = 0;
int sensorStateR = 0;
int sensorStateRR = 0;

void setup() {
 Serial.begin(9600); // Init the serial port

 pinMode(sensorPinLL, INPUT);
 pinMode(sensorPinL, INPUT);
 pinMode(sensorPinR, INPUT);
 pinMode(sensorPinRR, INPUT);

}

void loop(){
  int sensorStateLL = digitalRead(sensorPinLL);
  int sensorStateL = digitalRead(sensorPinL);
  int sensorStateR = digitalRead(sensorPinR);
  int sensorStateRR = digitalRead(sensorPinRR);
 Serial.print(sensorStateLL);
 Serial.print(sensorStateL);
 Serial.print(sensorStateR);
 Serial.println(sensorStateRR);
 delay(100);
}