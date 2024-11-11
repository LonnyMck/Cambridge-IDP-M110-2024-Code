/*
IDP
David Paterson
Push Button Module Example V1
When you push the digital button the Led 2 will turn off otherwise the LED turns on.
*/

//sensor placement
int PIN_BUTTON = 2;  // Connect sensor to input pin 3


//time variable, time since arduino started
unsigned long time_since;


void setup() {
  Serial.begin(9600);
  pinMode(PIN_BUTTON, INPUT);  // declare pushbutton as input
}
void loop() {
  int val = digitalRead(PIN_BUTTON);  // read input value
  if (val == HIGH) { 
    
    if ( (millis() - time_since) > 100){
      Serial.println("REGISTERED");
    };
    time_since = millis();

  } else {

  }
}
