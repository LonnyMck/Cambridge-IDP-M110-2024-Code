/*
IDP
can be used for any sensor just to test if you have the connections right
*/
int inputPin = A0;  // select the input pin
void setup() {
  //Serial init
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  int input_t = analogRead(inputPin) * 100;

  Serial.print(input_t);
  delay(500);
}