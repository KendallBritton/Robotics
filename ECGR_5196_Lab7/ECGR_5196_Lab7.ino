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

#define LEFT_MOTOR_SPEED 15           // Inital left motor speed for traveling striaght
#define RIGHT_MOTOR_SPEED 17          // Inital right motor speed for traveling striaght

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
float distanceToObject[40] = {};    // Array to hold the filtered sensor measurements
int iterList[40] = {};              // This array holds the mapped servo iterations according to each ultrasonic reading
int set = -1;                       // The variable dictates which set of the sweep the object was detected in


//============================================================================
// The setup() funtion runs one time at the beginning of the Energia program 
//============================================================================
void setup() {
 setupRSLK();                // Set up all of the pins & functions needed to be used by the TI bot
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
 myservo.attach(38);         // attaches the servo on Port 2.4 to the servo object
 myservo.write(0);           // Send it to the default position
}

//============================================================================
// The loop() function runs after the setup() function completes in an 
// Energia program and will continue to run in a repeating loop until the 
// LaunchPad is reset or powered off 
//============================================================================
void loop() {


  delay(5000);         // Waits for 5 seconds

  int pos = 0;         // variable to store the servo position
  myservo.write(0);    // Send it to the default position

  delay(500);

  for(pos = 0; pos < 180; pos += 9){  // Performs first sweep to find object
     myservo.write(pos);              // tell servo to go to position in variable 'pos' 
     pulseUltrasonicSensor();
     delay(15);                       // waits 15ms for the servo to reach the position 
     sortAndFindMedian(); 
     delay(500);                      // waits 500ms for the servo to reach the position 
  }

  delay(500);                 // waits 500ms for the servo to reach the position 

  rotateCCW(180);             // Robot turns around to perform second sweep
  
  delay(500);                 // waits 500ms for the servo to reach the position 
  
  myservo.write(0);           // Send it to the default position

  delay(500);                 // waits 500ms for the servo to reach the position 

  for(pos = 0; pos < 180; pos += 9){  // Performs second sweep to find object
     myservo.write(pos);              // tell servo to go to position in variable 'pos'  
     pulseUltrasonicSensor();
     delay(15);                       // waits 15ms for the servo to reach the position 
     sortAndFindMedian(); 
     delay(500);                      // waits 500ms for the servo to reach the position 
  }

  sortFinalResults();     // Sorts the readings to find the closest object          

  delay(500);             // waits 500ms for the servo to reach the position 

  rotateCCW(90);          // Turns 90 degrees to set reference point for object

  delay(500);             // waits 500ms for the servo to reach the position 

  if(set == 0){       // Drives to object if it is in first sweep
    
      rotateCCW((180 - (iterList[0] * 9)));
      delay(500);
      driveStraightCM(distanceToObject[0] - 2);
      
  } else if (set == 1){   // Drives to object if it is in second sweep

      rotateCCW(180);
      delay(500);
      rotateCCW((360 - (iterList[0] * 9)));
      delay(500);
      driveStraightCM(distanceToObject[0] - 2);
      
  }


 delay(10000);
  
}


//============================================================================
// The sortFinalResults() function sorts the total distance recordings
// of the closest object. It also detects which sweep the object was found in.
//============================================================================
void sortFinalResults(){
  int temp;

  for(int i = 0; i < 40; i++){                         // Sorts the ultrasonic values in ascending order 
      for(int j = i + 1; j < 40; j++){
        if(distanceToObject[i] > distanceToObject[j]){
          temp = distanceToObject[i];
          distanceToObject[i] = distanceToObject[j];
          distanceToObject[j] = temp;

          temp = iterList[i];                         // Sorts the servo iteration values according to ultrasonic values
          iterList[i] = iterList[j];
          iterList[j] = temp;
        }
      }  
  }

  if((iterList[0] >= 0) && (iterList[0] <= 19)){    // Determines in object is in sweep 1
    set = 0;
  }

   if((iterList[0] >= 20) && (iterList[0] <= 39)){  // Determines in object is in sweep 2
    set = 1;
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
  float temp = (((degree * PI* wheel_base)/ 360) * (1 / (wheel_diam * PI)) * gear_ratio * pulses_per_motor) - ERROR_OVERSHOOT_ROTATE;
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
