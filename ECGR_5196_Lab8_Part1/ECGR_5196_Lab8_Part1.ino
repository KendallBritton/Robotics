//*******************************************************************
// This program will allow the robot to find objects on the left    *
// and right of the robot, while traveling a straight path.         *
// The robot will detect the objects on the first path travel. Then *
// on the path back, the robot will point towards the objects       *
// location when perpendicular to the object.                       *
//                                                                  *
//                                                                  *
// Kendall Britton, 2022-04-18                                      *
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

void setRawMotorSpeed(uint8_t motorNum, uint8_t speed);
  
void setup() {    // put your setup code here, to run once:
   // initialize two digital pins as outputs.
   setupRSLK(); 
   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);
   myservo.attach(38);  // attaches the servo on Port 2.4 to the servo object
   myservo.write(0);    // Send it to the default position
   Serial.begin(9600);
}

void loop() {    // put your main code here, to run repeatedly: 

  delay(5000);
  int j = 0;

   for(int i = 0; i < 15; i++){
      
      myservo.write(0);              // tell servo to go to position in variable 'pos'
      delay(500);
      pulseUltrasonicSensor();
      delay(500);
      sortAndFindMedian();
      delay(500);
      Serial.print("Set ");
      Serial.print(count);
      Serial.print(":      Left Reading: ");
      Serial.print(distanceToObject[j]);
      delay(3000);

      j++;
      
      myservo.write(180);              // tell servo to go to position in variable 'pos'
      delay(500);
      pulseUltrasonicSensor();
      delay(500);
      sortAndFindMedian();
      delay(500);
      Serial.print("      Right Reading: ");
      Serial.println(distanceToObject[j]);
      delay(500);

     // driveStraightCM(20);
      //if( (i < 6) || ((i > 7) && (i < 12)) || (i == 14) ){
        //rotateCCWShort(1);
      //}
      
      //delay(500);
      count++;
      j++;

   }
   

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


      readingsPerRotate[i] = centimeters;

      delayMicroseconds(10);
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
  delay(500);    // Pauses before next action
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
