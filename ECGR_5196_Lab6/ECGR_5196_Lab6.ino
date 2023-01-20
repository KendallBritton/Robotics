//*******************************************************************
// This program will allow the robot to find the shortest path to   *
// the nearest wall using an ultrasonic sensor. Once the robot      *
// finds the nearest wall, it will drive straight and stop 30 cm    *
// from the wall. The robot will then turn 90 degrees clockwise and *
// travel 100 cm straight.                                          *
//                                                                  *
//                                                                  *
// Kendall Britton, 2022-03-21                                      *
//*******************************************************************

#include "SimpleRSLK.h"

#define LEFT_MOTOR_SPEED 15           // Inital left motor speed for traveling striaght
#define RIGHT_MOTOR_SPEED 17          // Inital right motor speed for traveling striaght

#define LEFT_MOTOR_SPEED_ROTATE 10    // Inital left motor speed for rotating
#define RIGHT_MOTOR_SPEED_ROTATE 10   // Inital right motor speed for rotating

#define ERROR_OVERSHOOT_STRAIGHT 16   // Error overshoot that makes encoder more accurate for traveling straight
#define ERROR_OVERSHOOT_ROTATE 10     // Error overshoot that makes encoder more accurate for rotating

#define ARRAY_SIZE  24                  // Allows for max range of 120 degree sweep
#define DISTANCE_FROM_WALL  30          // Distance to drive to from the wall
#define SENSOR_PLACEMENT_CORRECTION 7   // Distance from ultrasonic to front of robot

float wheel_diam = 6.99999;       // Wheel diameter of the TI RSLK
uint16_t gear_ratio = 120;        // Gear ratio of the TI RSLK
uint8_t pulses_per_motor = 3;     // Encoder pulse to motor turn ration for TI RSLK
float wheel_base = 13.99999;      // Wheel base distance of the TI RSLK

const int trigPin = 32;    // This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33;    // This is Port Pin 5.1 on the MSP432 Launchpad

float readingsPerRotate[9] = {};          // Array to filter the various sensor measurements
float distanceToWall[ARRAY_SIZE] = {};    // Array to hold the filtered sensor measurements


//============================================================================
// The setup() funtion runs one time at the beginning of the Energia program 
//============================================================================
void setup() {
 setupRSLK();               // Set up all of the pins & functions needed to be used by the TI bot
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
}


//============================================================================
// The loop() function runs after the setup() function completes in an 
// Energia program and will continue to run in a repeating loop until the 
// LaunchPad is reset or powered off 
//============================================================================
void loop() {
  float pulseLength, centimeters;
  float closestWallDistance, temp;
  
  delay(5000);              // Waits for 5 seconds

    for(int iter = 0; iter < ARRAY_SIZE; iter++){     // Performs full 120 degree range search
      for(int i = 0; i < 9; i++){
    
          digitalWrite(trigPin, LOW);                   // send low to get a clean pulse
          delayMicroseconds(10);                        // let it settle
          digitalWrite(trigPin, HIGH);                  // send high to trigger device
          delayMicroseconds(10);                        // let it settle
          digitalWrite(trigPin, LOW);                   // send low to get a clean pulse
          delayMicroseconds(10);                        // let it settle
    
          pulseLength = pulseIn(echoPin, HIGH);         // measure pulse coming back
          centimeters = pulseLength / 58;
    
          readingsPerRotate[i] = centimeters;

          delayMicroseconds(10);
      }

      for(int i = 0; i < 9; i++){                         // Sorts the ultrasonic values in ascending order 
          for(int j = i + 1; j < 9; j++){
            temp = readingsPerRotate[i];
            readingsPerRotate[i] = readingsPerRotate[j];
            readingsPerRotate[j] = temp;
          }  
      }
      
      distanceToWall[iter] = (readingsPerRotate[4]);      // Takes median value of sorted array
      
     if((distanceToWall[iter] <= distanceToWall[iter - 1]) && (iter > 1)){    // Filter to find shortest distance, turns it current distance is shorter than previous
        rotateCCW(3);
      }   
    }

  rotateCW(3);      // Turn right 3 degrees (Error correction)
  rotateCW(3);      // Turn right 3 degrees (Error correction)
  rotateCW(3);      // Turn right 3 degrees (Error correction)
  
  calcForwardDistance();

  rotateCW(90);           // Turn right 90 degrees

  driveStraightCM(100);   // Drive straight 100 cm
}


//============================================================================
// The calcForwardDistance() function calculates/sorts the total distance the
// closest wall is. It will then drive straight until it is 30 cm away from wall
//============================================================================
void calcForwardDistance(){

  long pulseLength, centimeters;
  int temp;

   for(int i = 0; i < 9; i++){
    
          digitalWrite(trigPin, LOW);                   // send low to get a clean pulse
          delayMicroseconds(10);                        // let it settle
          digitalWrite(trigPin, HIGH);                  // send high to trigger device
          delayMicroseconds(10);                        // let it settle
          digitalWrite(trigPin, LOW);                   // send low to get a clean pulse
          delayMicroseconds(10);                        // let it settle
    
          pulseLength = pulseIn(echoPin, HIGH);         // measure pulse coming back
          centimeters = pulseLength / 58;
    
          readingsPerRotate[i] = centimeters;           // Measurement is added to array

          delay(50);
   }

   for(int i = 0; i < 9; i++){                          // Sorts the ultrasonic values in ascending order
      for(int j = i + 1; j < 9; j++){
        temp = readingsPerRotate[i];
        readingsPerRotate[i] = readingsPerRotate[j];
        readingsPerRotate[j] = temp;
      }     
  }
   driveStraightCM((readingsPerRotate[4]) - DISTANCE_FROM_WALL - SENSOR_PLACEMENT_CORRECTION);    // Drives straight for distance measured minus 30 cm 
 
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
// The pulseCountRotateCCW() function calculates the total amount of encoder
// pulses that are needed to rotate a specific amount of degrees.
//============================================================================
int pulseCountRotate(int degree){
  float temp = (((degree * PI* wheel_base)/ 360) * (1 / (wheel_diam * PI)) * gear_ratio * pulses_per_motor);
  return temp;
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
// The rotateCW() function allows the robot to rotate a specific amount of
// degrees.
//============================================================================
void rotateCW(int degree){

  uint16_t l_totalCount = 0;                            // Initializes left encoder count
  uint16_t r_totalCount = 0;                            // Initializes right encoder count
  uint16_t l_motor_speed = LEFT_MOTOR_SPEED_ROTATE;     // Initializes left motor speed
  uint16_t r_motor_speed = RIGHT_MOTOR_SPEED_ROTATE;    // Initializes right motor speed
  int rotatePulse;

  if(degree > 3){
      rotatePulse = pulseCountRotate(degree) - ERROR_OVERSHOOT_ROTATE;    // Calculates encoder pulses need to rotate degrees
  } else {
     rotatePulse = pulseCountRotate(degree);
  }

  resetLeftEncoderCnt();      // Resets current encoder count
  resetRightEncoderCnt();     // Resets current encoder count

  setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);     // Set motor direction
  setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);     // Set motor direction
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
