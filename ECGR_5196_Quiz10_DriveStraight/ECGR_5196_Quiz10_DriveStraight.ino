//============================================================================
// File:  DriveStraightSimple
//
// 2021-03-11 - James Conrad, from code borrowed from TI
//    (some of this original code by Franklin S. Cooper Jr.)
// Summary:
//  This example will demonstrate the various features of the motor library.
//  The robot will go forward for a specified amount of time. 
//
//============================================================================

#include "SimpleRSLK.h"

#define WHEELSPEED 40          // Default raw pwm speed for motor.
#define LEFT_MOTOR_SPEED 24
#define RIGHT_MOTOR_SPEED 26
#define DELAY_MILLI_SECONDS_100CM 500  // Milliseconds to approximate 50 cm straight
#define PULSES_100CM 1637     // Number of pulses for 100 cm straight

//============================================================================
// The setup() funtion runs one time at the beginning of the Energia program 
//============================================================================
void setup() {
  setupRSLK();  // Set up all of the pins & functions needed to be used by the TI bot
}

//============================================================================
// The loop() function runs after the setup() function completes in an 
// Energia program and will continue to run in a repeating loop until the 
// LaunchPad is reset or powered off 
//============================================================================
void loop() {
  delay(2000);
  for(int i = 0; i < 1; i++){
  // === DRIVE Straight ==========================
  driveStraight100cm_encoder();                       // Halt motors 
  delay(10000);
  //==============================================
  }

}  //end of loop function

void driveStraight100cm_encoder(void){
  uint16_t l_totalCount = 0;
  uint16_t r_totalCount = 0;
  uint16_t l_motor_speed = LEFT_MOTOR_SPEED;
  uint16_t r_motor_speed = RIGHT_MOTOR_SPEED;

  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD); // Cause the robot to drive forward 
  enableMotor(BOTH_MOTORS);

  setMotorSpeed(RIGHT_MOTOR,RIGHT_MOTOR_SPEED); // Set motor speed
  setMotorSpeed(LEFT_MOTOR,LEFT_MOTOR_SPEED);             
  

  while( (l_totalCount < PULSES_100CM) || (r_totalCount < PULSES_100CM) ) {  
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    // Stop motors if they reach 1 meter
    
    if (r_totalCount >= PULSES_100CM){
      disableMotor(RIGHT_MOTOR);
    }
    
    if (l_totalCount >= PULSES_100CM){
      disableMotor(LEFT_MOTOR);
    }

    
   if((l_totalCount + 1) < r_totalCount){
     delay(10);
     setMotorSpeed(LEFT_MOTOR, l_motor_speed + 1);
     setMotorSpeed(RIGHT_MOTOR, r_motor_speed - 1);
    }

    if((r_totalCount + 1) < l_totalCount){
      delay(10);
      setMotorSpeed(LEFT_MOTOR, l_motor_speed - 1);
      setMotorSpeed(RIGHT_MOTOR, r_motor_speed + 1);
   }
    

  }
  delay(200);                         
  
}
