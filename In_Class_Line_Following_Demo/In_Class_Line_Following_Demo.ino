/*
 * Energia Robot Library for Texas Instruments' Robot System Learning Kit (RSLK)
 * Line Following Example
 *
 * Summary:
 * This example has the TI Robotic System Learning Kit (TI RSLK) follow a line
 * using a basic line following algorithm. This example works on a dark floor with
 * a white line or a light floor with a dark line. The robot first needs to be calibrated
 * Then place the robot on the hit the left button again to begin the line following.
 *
 * How to run:
 * 1) Push left button on Launchpad to have the robot perform calibration.
 * 2) Robot will drive forwards and backwards by a predefined distance.
 * 3) Place the robot center on the line you want it to follow.
 * 4) Push left button again to have the robot begin to follow the line.
 *
 * Parts Info:
 * o Black eletrical tape or white electrical tape. Masking tape does not work well
 *   with IR sensors.
 *
 * Learn more about the classes, variables and functions used in this library by going to:
 * https://fcooper.github.io/Robot-Library/
 *
 * Learn more about the TI RSLK by going to http://www.ti.com/rslk
 *
 * created by Franklin Cooper Jr.
 *
 * This example code is in the public domain.
 */

#include "SimpleRSLK.h"

float wheel_diam = 6.99999;       // Wheel diameter of the TI RSLK
uint16_t gear_ratio = 120;        // Gear ratio of the TI RSLK
uint8_t pulses_per_motor = 3;     // Encoder pulse to motor turn ration for TI RSLK
float wheel_base = 13.99999;      // Wheel base distance of the TI RSLK

uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];

#define LEFT_MOTOR_SPEED_ROTATE 10    // Inital left motor speed for rotating
#define RIGHT_MOTOR_SPEED_ROTATE 10   // Inital right motor speed for rotating

#define ERROR_OVERSHOOT_ROTATE 18     // Error overshoot that makes encoder more accurate for rotating

void setup()
{
  Serial.begin(115200);

  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_LEFT_BTN);
  /* Red led in rgb led */
  setupLed(RED_LED);
  clearMinMax(sensorMinVal,sensorMaxVal);
}

void floorCalibration() {
  /* Place Robot On Floor (no line) */
  delay(2000);
  String btnMsg = "Push left button on Launchpad to begin calibration.\n";
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);

  delay(1000);

  Serial.println("Running calibration on floor");
  simpleCalibrate();
  Serial.println("Reading floor values complete");

  btnMsg = "Push left button on Launchpad to begin line following.\n";
  btnMsg += "Make sure the robot is on the line.\n";
  /* Wait until button is pressed to start robot */
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);
  delay(1000);

  enableMotor(BOTH_MOTORS);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
  //setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  /* Enable both motors */
  //enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  //setMotorSpeed(BOTH_MOTORS,20);

  for(int x = 0;x<100;x++){
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
  }

  /* Disable both motors */
  //disableMotor(BOTH_MOTORS);
}

bool lineFound = false;

bool isCalibrationComplete = false;
void loop() {
  uint16_t normalSpeedL = 15;
  uint16_t normalSpeedR = 17;
  uint16_t fastSpeed = 20;
  uint16_t sensorReading[8] = {};

  /* Valid values are either:
   *  DARK_LINE  if your floor is lighter than your line
   *  LIGHT_LINE if your floor is darker than your line
   */
  uint8_t lineColor = DARK_LINE;

  /* Run this setup only once */
  if(isCalibrationComplete == false) {
    floorCalibration();
    isCalibrationComplete = true;
  }

  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);

  for (int i = 0; i < 8; i++){
      sensorReading[i] = sensorVal[i];
      Serial.print("Sensor ");
      Serial.print(i); Serial.print(" Value: ");
      Serial.println(sensorReading[i]);
      delay(10);
  }


  uint32_t linePos = getLinePosition(sensorCalVal,lineColor);
        delay(10);  Serial.println(linePos);

  if(lineFound == false){
    for(int i = 0; i < 8; i++){
      if(sensorVal[i] >= 2000){
        lineFound = true;
      }
    } 
    if(lineFound == true){
        disableMotor(BOTH_MOTORS);
        setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
        enableMotor(BOTH_MOTORS);
    } else{
       rotateCCW(90);   
    }

  }


  if(linePos > 0 && linePos < 2000) {
    
    setMotorSpeed(LEFT_MOTOR,0);
    setMotorSpeed(RIGHT_MOTOR,normalSpeedR - 5);
    
  } else if (linePos > 2000 && linePos < 3000){
    
    setMotorSpeed(LEFT_MOTOR,normalSpeedL - 5);
    setMotorSpeed(RIGHT_MOTOR,normalSpeedR);
    
  } else if(linePos > 3500 && linePos < 4500) {
    
    setMotorSpeed(LEFT_MOTOR,normalSpeedL);
    setMotorSpeed(RIGHT_MOTOR,normalSpeedR - 5);
    
  } else if(linePos > 4500) {
    
    setMotorSpeed(LEFT_MOTOR,normalSpeedL - 5 );
    setMotorSpeed(RIGHT_MOTOR, 0);
    
  }else if(linePos == 0) {
    
    setMotorSpeed(LEFT_MOTOR,0);
    setMotorSpeed(RIGHT_MOTOR,0);
    
  } else {
    
    setMotorSpeed(LEFT_MOTOR,normalSpeedL - 5);
    setMotorSpeed(RIGHT_MOTOR,normalSpeedR - 5);
    
  }  
}


//============================================================================
// The pulseCountRotateCCW() function calculates the total amount of encoder
// pulses that are needed to rotate a specific amount of degrees.
//============================================================================
int pulseCountRotate(float degree){
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

  int rotatePulse = pulseCountRotate(degree) - ERROR_OVERSHOOT_ROTATE;           // Calculates encoder pulses need to rotate degrees

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
  delay(500);    // Pauses before next action
}
