/*
 * HC-SR04 Ultrasonic Distance Sensor Example
 * Demonstrates sensing distance with the HC-SR04 using Texas Instruments LaunchPads.
 * 
 * Created by Frank Milburn 5 Jun 2015, Released into the public domain.
 * Modified by James Conrad 2 Mar 2021
 * 
 */

 const int trigPin = 32;    // This is Port Pin 3.5 on the MSP432 Launchpad
 const int echoPin = 33;    // This is Port Pin 5.1 on the MSP432 Launchpad
 #define BLUE 77            // Define BLUE of the tri-color LED as pin 77

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(BLUE, OUTPUT);      // Blue LED
  Serial.begin(9600);
  delay(1000);                // Allow the serial monitor to settle. Make sure to turn the monitor on! (Tools)
  Serial.println("Starting HC-SR04 Test...");

}

void loop() {
  float pulseLength, centimeters;

  digitalWrite(trigPin, LOW);                   // send low to get a clean pulse
  delayMicroseconds(10);                        // let it settle
  digitalWrite(trigPin, HIGH);                  // send high to trigger device
  delayMicroseconds(10);                        // let it settle
  digitalWrite(trigPin, LOW);                   // send low to get a clean pulse
  delayMicroseconds(10);                        // let it settle

  pulseLength = pulseIn(echoPin, HIGH);         // measure pulse coming back
  centimeters = pulseLength / 58;

  delay(1000);
  digitalWrite(BLUE, HIGH);                     // turn the RBG (BLUE) LED on (HIGH is the v level)

  if((centimeters > 0) && (centimeters <= 400)){

     Serial.print("Distance = ");
      Serial.print("       ");
      Serial.print(centimeters);
      Serial.println(" centimeters");
      delay(1000);
      digitalWrite(BLUE, LOW);                      // turn the RBG (BLUE) LED off (LOW is the v level)
  
}
}
