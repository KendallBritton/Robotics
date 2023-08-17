//*******************************************************************
// This program will allow the robot to find the closest object to  *
// the robot. The robot will then drive towards the object once     *
// detected. It will then stop as close as possible to the object   *
// without coming in contact with the object.                       *
//                                                                  *
//                                                                  *
//                                                                  *
// Kendall Britton, 2022-04-04                                      *
//*******************************************************************

#include <Servo.h> 
#include "SimpleRSLK.h"

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

#define LEFT_MOTOR_SPEED   31 //20 //31   //15           // Inital left motor speed for traveling striaght
#define RIGHT_MOTOR_SPEED  35 //24 //35   //17          // Inital right motor speed for traveling striaght

#define LEFT_MOTOR_SPEED_ROTATE 10    // Inital left motor speed for rotating
#define RIGHT_MOTOR_SPEED_ROTATE 10   // Inital right motor speed for rotating

#define ERROR_OVERSHOOT_STRAIGHT 16   // Error overshoot that makes encoder more accurate for traveling straight
#define ERROR_OVERSHOOT_ROTATE 18     // Error overshoot that makes encoder more accurate for rotating

#define ARRAY_SIZE  24                  // Allows for max range of 120 degree sweep
#define DISTANCE_FROM_WALL  0          // Distance to drive to from the wall
#define SENSOR_PLACEMENT_CORRECTION 4   // Distance from ultrasonic to front of robot

float wheel_diam = 6.99999;       // Wheel diameter of the TI RSLK
uint16_t gear_ratio = 120;        // Gear ratio of the TI RSLK
uint8_t pulses_per_motor = 3;     // Encoder pulse to motor turn ration for TI RSLK
float wheel_base = 13.99999;      // Wheel base distance of the TI RSLK

const int trigPin = 32;    // This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33;    // This is Port Pin 5.1 on the MSP432 Launchpad

int iter = 0;                       // This variable keeps tracks of how many iterations the servo has moved
float readingsPerRotate[9] = {};    // Array to filter the various sensor measurements
float distanceToObject[30] = {};    // Array to hold the filtered sensor measurements
int iterList[30] = {};              // This array holds the mapped servo iterations according to each ultrasonic reading
int count = 0;
int encoderValues[15];
int startIter = 29;
int endIter = 0;
int set1 = 0;
int set2 = -1;

void setRawMotorSpeed(uint8_t motorNum, uint8_t speed);
  
void setup() {    // put your setup code here, to run once:
   // initialize two digital pins as outputs.
   setupRSLK(); 
   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);
   pinMode(76, OUTPUT);  //RGB LED - P2.1 GREEN_LED
   pinMode(77, OUTPUT);  //RGB LED - P2.2 BLUE_LED 
   myservo.attach(38);  // attaches the servo on Port 2.4 to the servo object
   myservo.write(0);    // Send it to the default position
}

void loop() {
  delay(5000);

  int pos = 0;
  myservo.write(0);
  delay(500);

  for (int i = 0; i < 15; i){

    myservo.write(0);
    delay(500);
    pulseUltrasonicSensor();
    delay(500);
    sortAndFindMedian();
    delay(500);

    pulseUltrasonicSensor();
    delay(500);
    sortAndFindMedian();
    delay(500);
    myservo.write(180);
    delay(500);


  }

 // for(pos = 0; pos < 180;
  
}


//============================================================================
// The sortAndFindMedian() function sorts the distance recordings at a current
// iteration of sweep. 
//============================================================================
void sortAndFindMedian(){
  int temp;

  for(int i = 0; i < 9; i++){                         // Sorts the ultrasonic values in ascending order 
      for(int j = i + 1; j < 9; j++){
        if(readingsPerRotate[i] > readingsPerRotate[j]){
          temp = readingsPerRotate[i];
          readingsPerRotate[i] = readingsPerRotate[j];
          readingsPerRotate[j] = temp;
        }
      }  
  }

  distanceToObject[iter] = readingsPerRotate[4];    // Takes median and adds it to list 
  iterList[iter] = iter;                            // Assigns current iteration to list
  iter++;
}


//============================================================================
// The pulseUltrasonicSensor() function calculates/pulses the distance recording
// of the closest object.
//============================================================================
void pulseUltrasonicSensor(){
  float pulseLength, centimeters;

  
    for(int i = 0; i < 9; i++){

      digitalWrite(trigPin, LOW);                   // send low to get a clean pulse
      delayMicroseconds(10);                        // let it settle
      digitalWrite(trigPin, HIGH);                  // send high to trigger device
      delayMicroseconds(10);                        // let it settle
      digitalWrite(trigPin, LOW);                   // send low to get a clean pulse
      delayMicroseconds(10);                        // let it settle

      pulseLength = pulseIn(echoPin, HIGH);         // measure pulse coming back
      centimeters = pulseLength / 58;

      delayMicroseconds(10);

      readingsPerRotate[i] = centimeters;

  }
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
  delay(2000);    // Pauses before next action
}

//============================================================================
// The rotateCCW() function allows the robot to rotate a specific amount of
// degrees.
//============================================================================
void rotateCCWShort(float degree){

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
  //delay(500);    // Pauses before next action
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

  setRawMotorSpeed(RIGHT_MOTOR,RIGHT_MOTOR_SPEED);     // Set motor speed
  setRawMotorSpeed(LEFT_MOTOR,LEFT_MOTOR_SPEED);       // Set motor speed            
  

  while( (l_totalCount < straightPulse) || (r_totalCount < straightPulse) ) {         // Stop motors if they reach desired encoder count
    l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();

    
    if (r_totalCount >= straightPulse){
      disableMotor(RIGHT_MOTOR);
    }
    
    if (l_totalCount >= straightPulse){
      disableMotor(LEFT_MOTOR);
    }

   if (((l_totalCount + 1) < r_totalCount) && (l_motor_speed <= LEFT_MOTOR_SPEED + 2)){             // Adjusts speed if one motor is faster than the other
     setRawMotorSpeed(LEFT_MOTOR, l_motor_speed + 1);
    }

    if(((r_totalCount + 1) < l_totalCount)  && (l_motor_speed >= LEFT_MOTOR_SPEED - 2)){            // Adjusts speed if one motor is faster than the other
      setRawMotorSpeed(LEFT_MOTOR, l_motor_speed - 1);
   }
    
  }
  delay(500);      // Pauses before next action                    
}

//============================================================================
// The driveStraightCM() allows the robot to drive straight for a specificed
// distance in cm.
//============================================================================
//void driveStraightPulsesCM(int pulses){
  //uint16_t l_totalCount = 0;                                // Initializes left encoder count
  //uint16_t r_totalCount = 0;                                // Initializes right encoder count
  //uint16_t l_motor_speed = LEFT_MOTOR_SPEED;                // Initializes left motor speed
  //uint16_t r_motor_speed = RIGHT_MOTOR_SPEED;               // Initializes right motor speed

  //int straightPulse = pulses;     // Calculates encoder pulses need to drive distance

  //resetLeftEncoderCnt();        // Resets current encoder count
  //resetRightEncoderCnt();       // Resets current encoder count
  
  //setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);   // Cause the robot to drive forward 
  //enableMotor(BOTH_MOTORS);                           // Enable motors

  //setRawMotorSpeed(RIGHT_MOTOR,RIGHT_MOTOR_SPEED);     // Set motor speed
  //setRawMotorSpeed(LEFT_MOTOR,LEFT_MOTOR_SPEED);       // Set motor speed            
  

  //while( (l_totalCount < straightPulse) || (r_totalCount < straightPulse) ) {         // Stop motors if they reach desired encoder count
    //l_totalCount = getEncoderLeftCnt(); r_totalCount = getEncoderRightCnt();

    
    //if (r_totalCount >= straightPulse){
      //disableMotor(RIGHT_MOTOR);
    //}
    
    //if (l_totalCount >= straightPulse){
      //disableMotor(LEFT_MOTOR);
    //}

   //if (((l_totalCount + 1) < r_totalCount) && (l_motor_speed <= LEFT_MOTOR_SPEED + 2)){             // Adjusts speed if one motor is faster than the other
     //setRawMotorSpeed(LEFT_MOTOR, l_motor_speed + 1);
    //}

    //if(((r_totalCount + 1) < l_totalCount)  && (l_motor_speed >= LEFT_MOTOR_SPEED - 2)){            // Adjusts speed if one motor is faster than the other
     // setRawMotorSpeed(LEFT_MOTOR, l_motor_speed - 1);
   //}
    
  //}
  //delay(500);      // Pauses before next action                    
//}


void mapIterations(void){

      if ((startIter == 0) || (startIter == 1)){    // Start mapping
      set1 = 0;
    }

    else if ((startIter == 2) || (startIter == 3)){
      set1 = 1;
    }

    else if ((startIter == 4) || (startIter == 5)){
      set1 = 2;
    }

    else if ((startIter == 6) || (startIter == 7)){
      set1 = 3;
    }

    else if ((startIter == 8) || (startIter == 9)){
      set1 = 4;
    }

   else if ((startIter == 10) || (startIter == 11)){
      set1 = 5;
    }

   else if ((startIter == 12) || (startIter == 13)){
      set1 = 6;
    }

   else if ((startIter == 14) || (startIter == 15)){
      set1 = 7;
    }

   else if ((startIter == 16) || (startIter == 17)){
      set1 = 8;
    }

   else if ((startIter == 18) || (startIter == 19)){
      set1 = 9;
    }

   else if ((startIter == 20) || (startIter == 21)){
      set1 = 10;
    }

   else if ((startIter == 22) || (startIter == 23)){
      set1 = 11;
    }

   else if ((startIter == 24) || (startIter == 25)){
      set1 = 12;
    }

   else if ((startIter == 26) || (startIter == 27)){
      set1 = 13;
    }

   else if ((startIter == 28) || (startIter == 29)){
      set1 = 14;
    }


    if ((endIter == 0) || (endIter == 1)){      // End mapping
      set2 = 0;
    }

   else if ((endIter == 2) || (endIter == 3)){
      set2 = 1;
    }

   else if ((endIter == 4) || (endIter == 5)){
      set2 = 2;
    }

   else if ((endIter == 6) || (endIter == 7)){
      set2 = 3;
    }

   else if ((endIter == 8) || (endIter == 9)){
      set2 = 4;
    }

  else  if ((endIter == 10) || (endIter == 11)){
      set2 = 5;
    }

   else if ((endIter == 12) || (endIter == 13)){
      set2 = 6;
    }

   else if ((endIter == 14) || (endIter == 15)){
      set2 = 7;
    }

   else if ((endIter == 16) || (endIter == 17)){
      set2 = 8;
    }

   else if ((endIter == 18) || (endIter == 19)){
      set2 = 9;
    }

   else if ((endIter == 20) || (endIter == 21)){
      set2 = 10;
    }

   else if ((endIter == 22) || (endIter == 23)){
      set2 = 11;
    }

   else if ((endIter == 24) || (endIter == 25)){
      set2 = 12;
    }

   else if ((endIter == 26) || (endIter == 27)){
      set2 = 13;
    }

   else if ((endIter == 28) || (endIter == 29)){
      set2 = 14;
    }
}

void driveToBox(void){
  
  do{
     
    for (int i = startIter; i >= 0; i--){
      if( (distanceToObject[i - 2] < distanceToObject[i]) && (abs(distanceToObject[i - 2] - distanceToObject[i]) >= 10)){
            endIter = i - 2;
            break;
      }   
    }

   mapIterations();

    startIter = endIter;

    //driveStraightPulsesCM(encoderValues[set1] - encoderValues[set2]);

    delay(2500);
  }
 while((set1 - set2) != 0);
}
