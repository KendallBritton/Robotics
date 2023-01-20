//*******************************************************************
// This program will allow the robot to travel a specificed course. *
// The course starts with the robot traveling the radius of a       *
// circle, then traveling the whole 360 degree circumference of the *
// circle, then will travel the radius again to return to the center*
// of the circle where it once started.                             *
//                                                                  *
//                                                                  *
// Kendall Britton, 2022-02-28                                      *
//*******************************************************************

#include "SimpleRSLK.h"

#define LEFT_MOTOR_SPEED 15           // Inital left motor speed for traveling striaght
#define RIGHT_MOTOR_SPEED 17          // Inital right motor speed for traveling striaght

#define LEFT_MOTOR_SPEED_ROTATE 10    // Inital left motor speed for rotating
#define RIGHT_MOTOR_SPEED_ROTATE 10   // Inital right motor speed for rotating

#define LEFT_MOTOR_SPEED_CIRCLE 20    // Inital left motor speed for traveling the circle
#define RIGHT_MOTOR_SPEED_CIRCLE 25   // Inital right motor speed for traveling the circle

#define ERROR_OVERSHOOT_STRAIGHT 16   // Error overshoot that makes encoder more accurate for traveling straight
#define ERROR_OVERSHOOT_ROTATE 10     // Error overshoot that makes encoder more accurate for rotating
#define ERROR_OVERSHOOT_CIRCLE 58     // Error overshoot that makes encoder more accurate for traveling the circle

float wheel_diam = 6.99999;       // Wheel diameter of the TI RSLK
uint16_t gear_ratio = 120;        // Gear ratio of the TI RSLK
uint8_t pulses_per_motor = 3;     // Encoder pulse to motor turn ration for TI RSLK
float wheel_base = 13.99999;      // Wheel base distance of the TI RSLK

int innerWheelPulses;     // Variable to keep track out inner wheel encoder pulses (circle)
int outerWheelPulses;     // Variable to keep track out outer wheel encoder pulses (circle)
float circlePulseRatio;   // Variable to keep wheel encoder ratio (circle)



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
  delay(5000);              // Waits for 5 seconds
  driveStraightCM(75);      // Drives straight for 75 cm                   
  rotateCCW(90);            // Rotates 90 CCW
  driveCircle(150);         // Drives the circumference of 1.5 diameter circle
  rotateCCW(90);            // Rotates 90 CCW
  driveStraightCM(75);      // Drives straight for 75 cm
  delay(10000);             // Waits for 10 seconds

} 


//============================================================================
// The pulseCountStraightCM() function calculates the total amount of 
// encoder pulses that are needed to travel a specific distance in cm. 
//============================================================================
int pulseCountStraightCM(int distanceCM){
  float temp = (distanceCM * (1 / (wheel_diam * PI)) * gear_ratio * pulses_per_motor) - ERROR_OVERSHOOT_STRAIGHT;
  return temp;
}


//============================================================================
// The pulseCountRotate() function calculates the total amount of encoder
// pulses that are needed to rotate a specific amount of degrees.
//============================================================================
int pulseCountRotate(int degree){
  float temp = (((degree * PI* wheel_base)/ 360) * (1 / (wheel_diam * PI)) * gear_ratio * pulses_per_motor) - ERROR_OVERSHOOT_ROTATE;
  return temp;
}


//============================================================================
// The pulseCountCircle() function calculates the total amount of encoder
// pulses that are needed to drive the circumference of a specific circle. 
//============================================================================
int pulseCountCircle(int circleDiameter){
  int innerDistance = circleDiameter - wheel_base;      // Keeps track of the inner wheel distance
  int outerDistance = circleDiameter + wheel_base;      // Keeps track of the outer wheel distance
  
  float temp = round(((innerDistance * PI) * (1 / (wheel_diam * PI)) * gear_ratio * pulses_per_motor)) - ERROR_OVERSHOOT_CIRCLE;
  innerWheelPulses = temp;
  temp = round(((outerDistance * PI) * (1 / (wheel_diam * PI)) * gear_ratio * pulses_per_motor)) - ERROR_OVERSHOOT_CIRCLE;
  outerWheelPulses = temp;
}


//============================================================================
// The driveCircle() functions allows the robot to drive the circumference
// of a specificed circle.
//============================================================================
void driveCircle(int circleDiameter){
  uint16_t l_totalCount = 0;                              // Initializes left encoder count
  uint16_t r_totalCount = 0;                              // Initializes right encoder count
  uint16_t l_motor_speed = LEFT_MOTOR_SPEED_CIRCLE;       // Initializes left motor speed
  uint16_t r_motor_speed = RIGHT_MOTOR_SPEED_CIRCLE;      // Initializes right motor speed

  int circlePulse = pulseCountCircle(circleDiameter);     // Calculates encoder pulses need to drive distance

  resetLeftEncoderCnt();          // Resets current encoder count
  resetRightEncoderCnt();         // Resets current encoder count
  
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);       // Set motor direction
  enableMotor(BOTH_MOTORS);                               // Enable motors

  setMotorSpeed(RIGHT_MOTOR,RIGHT_MOTOR_SPEED_CIRCLE);    // Set motor speed
  setMotorSpeed(LEFT_MOTOR,LEFT_MOTOR_SPEED_CIRCLE);      // Set motor speed

  circlePulseRatio = outerWheelPulses / innerWheelPulses;   // Calculate outer to inner wheel encoder ratio

  while( (l_totalCount < innerWheelPulses) || (r_totalCount < outerWheelPulses) ) {         // Stop motors if they reach desired encoder count
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    
    if (r_totalCount >= outerWheelPulses){
      disableMotor(RIGHT_MOTOR);
      disableMotor(LEFT_MOTOR);
    }
    
  }
  delay(2000);         // Pauses before next action                 
  
}


//============================================================================
// The rotateCCW() function allows the robot to rotate a specific amount of
// degrees.
//============================================================================
void rotateCCW(int degree){

  uint16_t l_totalCount = 0;                            // Initializes left encoder count
  uint16_t r_totalCount = 0;                            // Initializes right encoder count
  uint16_t l_motor_speed = LEFT_MOTOR_SPEED_ROTATE;     // Initializes left motor speed
  uint16_t r_motor_speed = RIGHT_MOTOR_SPEED_ROTATE;    // Initializes right motor speed

  int rotatePulse = pulseCountRotate(degree);           // Calculates encoder pulses need to rotate degrees

  resetLeftEncoderCnt();      // Resets current encoder count
  resetRightEncoderCnt();     // Resets current encoder count

  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);     // Set motor direction
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);     // Set motor direction
  enableMotor(BOTH_MOTORS);                             // Enable motors

  setMotorSpeed(RIGHT_MOTOR,RIGHT_MOTOR_SPEED_ROTATE);    // Set motor speed
  setMotorSpeed(LEFT_MOTOR,LEFT_MOTOR_SPEED_ROTATE);      // Set motor speed

  while( (l_totalCount < rotatePulse) || (r_totalCount < rotatePulse) ) {       // Stop motors if they reach desired encoder count
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    
    if (r_totalCount >= rotatePulse){
      disableMotor(RIGHT_MOTOR);
    }
    
    if (l_totalCount >= rotatePulse){
      disableMotor(LEFT_MOTOR);
    }
    
  }
  delay(2000);    // Pauses before next action
}


//============================================================================
// The driveStraightCM() allows the robot to drive straight for a specificed
// distance in cm.
//============================================================================
void driveStraightCM(int distanceCM){
  uint16_t l_totalCount = 0;                                // Initializes left encoder count
  uint16_t r_totalCount = 0;                                // Initializes right encoder count
  uint16_t l_motor_speed = LEFT_MOTOR_SPEED;                // Initializes left motor speed
  uint16_t r_motor_speed = RIGHT_MOTOR_SPEED;               // Initializes right motor speed

  int straightPulse = pulseCountStraightCM(distanceCM);     // Calculates encoder pulses need to drive distance

  resetLeftEncoderCnt();        // Resets current encoder count
  resetRightEncoderCnt();       // Resets current encoder count
  
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);   // Cause the robot to drive forward 
  enableMotor(BOTH_MOTORS);                           // Enable motors

  setMotorSpeed(RIGHT_MOTOR,RIGHT_MOTOR_SPEED);     // Set motor speed
  setMotorSpeed(LEFT_MOTOR,LEFT_MOTOR_SPEED);       // Set motor speed            
  

  while( (l_totalCount < straightPulse) || (r_totalCount < straightPulse) ) {         // Stop motors if they reach desired encoder count
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();
    
    if (r_totalCount >= straightPulse){
      disableMotor(RIGHT_MOTOR);
    }
    
    if (l_totalCount >= straightPulse){
      disableMotor(LEFT_MOTOR);
    }

    
   if((l_totalCount + 1) < r_totalCount){             // Adjusts speed if one motor is faster than the other
     setMotorSpeed(LEFT_MOTOR, l_motor_speed + 1);
    }

    if((r_totalCount + 1) < l_totalCount){            // Adjusts speed if one motor is faster than the other
      setMotorSpeed(LEFT_MOTOR, l_motor_speed - 1);
   }
    
  }
  delay(2000);      // Pauses before next action                  
  
}
