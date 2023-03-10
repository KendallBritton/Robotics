//*******************************************************************
// ServoExample - Run an inexpensive Servo Motor
// James Conrad, 2020-06-10
//*******************************************************************
#include <Servo.h> 
#include "SimpleRSLK.h"

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
  
void setup() {    // put your setup code here, to run once:
   // initialize two digital pins as outputs.
   setupRSLK(); 
   pinMode(76, OUTPUT);  //RGB LED - P2.1 GREEN_LED
   pinMode(77, OUTPUT);  //RGB LED - P2.2 BLUE_LED 
   myservo.attach(38);  // attaches the servo on Port 2.4 to the servo object
   myservo.write(0);    // Send it to the default position
   Serial.begin(9600);
}

void loop() {    // put your main code here, to run repeatedly: 
   int pos = 0;    // variable to store the servo position
   digitalWrite(77, HIGH);
   for(pos = 0; pos < 180; pos += 1) { // goes from 0 degrees to 180 degrees 
      myservo.write(pos);              // tell servo to go to position in variable 'pos' 
      Serial.println(pos);
      delay(15);                       // waits 15ms for the servo to reach the position 
   } 
   digitalWrite(77, LOW);
   delay(500);
   digitalWrite(76, HIGH);
   for(pos = 180; pos>=1; pos-=1)  {   // goes from 180 degrees to 0 degrees 
      myservo.write(pos);              // tell servo to go to position in variable 'pos' 
      Serial.println(pos);
      delay(15);                       // waits 15ms for the servo to reach the position 
   } 
  digitalWrite(76, LOW);  
  delay(500);
}
