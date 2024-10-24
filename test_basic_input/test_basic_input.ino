/*
IDP
can be used for any sensor just to test if you have the connections right
*/
int inputPin = 2;  // select the input pin
void setup() {
  //Serial init
  Serial.begin(9600);
  pinMode(inputPin, INPUT);
}

void loop() {
  // read the value from the sensor:
  int input_t = digitalRead(inputPin);

  Serial.println(input_t);
  delay(500);
}