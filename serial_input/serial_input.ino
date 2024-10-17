#include <Adafruit_MotorShield.h>
#include <StandardCplusplus.h>
#include <vector>
#include <string>
#include <sstream>

using namespace std;

String input;
stringstream ss;
vector<string> inputs;
string element;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Motor func test");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }

  Serial.println("Motor Shield found.");

}

void loop() {

  // This currently doesn't work
  
  if(Serial.available()){
        input = Serial.readStringUntil('\n');;
        Serial.print("You typed: " );
        Serial.println(input);

        ss << input;

        while (getline(ss, element, ' ')) {
      
        inputs.push_back(element);

        ss.clear()
    }

    
    Serial.println(inputs);
    }

}
