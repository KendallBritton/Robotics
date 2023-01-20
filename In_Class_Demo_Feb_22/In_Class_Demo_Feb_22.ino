#include "SimpleRSLK.h"

float wheel_diam = 6.99999;   // Wheel diameter in cm

float distanceTraveled(float wheel_diam, uint16_t cnt_per_rev, uint8_t current_cnt){
  float temp = (wheel_diam * PI * current_cnt) / cnt_per_rev;
  return temp;
  }

void setup() {
  // put your setup code here, to run once:
  setupRSLK();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly: 
  //Serial.println("Its Working");
  Serial.print(getEncoderLeftCnt());
  Serial.print("\t");
  Serial.println(getEncoderRightCnt());
  delay(500);
}
