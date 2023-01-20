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

#define LEFT_MOTOR_SPEED   31          // Inital left motor speed for traveling striaght
#define RIGHT_MOTOR_SPEED  35          // Inital right motor speed for traveling striaght

#define LEFT_MOTOR_SPEED_ROTATE 10    // Inital left motor speed for rotating
#define RIGHT_MOTOR_SPEED_ROTATE 10   // Inital right motor speed for rotating

#define ERROR_OVERSHOOT_STRAIGHT 16   // Error overshoot that makes encoder more accurate for traveling straight
#define ERROR_OVERSHOOT_ROTATE 18     // Error overshoot that makes encoder more accurate for rotating

float wheel_diam = 6.99999;       // Wheel diameter of the TI RSLK
uint16_t gear_ratio = 120;        // Gear ratio of the TI RSLK
uint8_t pulses_per_motor = 3;     // Encoder pulse to motor turn ration for TI RSLK
float wheel_base = 13.99999;      // Wheel base distance of the TI RSLK

const int trigPin = 32;    // This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33;    // This is Port Pin 5.1 on the MSP432 Launchpad

float readingsPerRotate[9] = {};         // Array to filter the various sensor measurements
float distanceToObjectLeft[15] = {};     // Array to hold the filtered sensor measurements
float distanceToObjectRight[15] = {};    // Array to hold the filtered sensor measurements
int prev = 0;                            // Variable that checks if the sensor has already identified an object
int prev2 = 0;                           // Variable that checks if the sensor has already identified an object

void setRawMotorSpeed(uint8_t motorNum, uint8_t speed);   // Initializes the raw motor speed function


//============================================================================
// The setup() funtion runs one time at the beginning of the Energia program 
//============================================================================
void setup() {  
   setupRSLK();                 // Set up all of the pins & functions needed to be used by the TI bot
   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);
   pinMode(76, OUTPUT);  //RGB LED - P2.1 GREEN_LED
   pinMode(77, OUTPUT);  //RGB LED - P2.2 BLUE_LED 
   myservo.attach(38);  // attaches the servo on Port 2.4 to the servo object
   myservo.write(0);    // Send it to the default position
}


//============================================================================
// The loop() function runs after the setup() function completes in an 
// Energia program and will continue to run in a repeating loop until the 
// LaunchPad is reset or powered off 
//============================================================================
void loop() {  
  
  delay(5000);        // Waits for 5 seconds
  int i = 0;          // Intializes variable for loop
  int j = 0;          // Intializes variable for loop

   for(i = 0; i < 15; i++){                                     // Performs first 3 meter path run 

      digitalWrite(77, HIGH);                                   // Light to indicate the ultrasonic is facing left
      myservo.write(0);                                         // tell servo to turn left (0 degrees)
      delay(800);                                               // waits 800 ms for the servo to reach the position
      pulseUltrasonicSensor();                                  // Pulses ultrasonic
      delay(500);                                               // waits 500 ms for the readings to process
      distanceToObjectLeft[i] = sortAndFindMedianLeft();        // Assigns value to array
      delay(500);                                               // waits 500 ms for the data to be stored
      digitalWrite(77, LOW);                                    // Turn off LED
      delay(500);

      digitalWrite(76, HIGH);                                   // Light to indicate the ultrasonic is facing right
      myservo.write(180);                                       // tell servo to turn right (180 degrees)
      delay(800);                                               // waits 800 ms for the servo to reach the position
      pulseUltrasonicSensor();                                  // Pulses ultrasonic
      delay(500);                                               // waits 500 ms for the readings to process
      distanceToObjectRight[i] = sortAndFindMedianRight();      // Assigns value to array
      delay(500);                                               // waits 500 ms for the data to be stored
      digitalWrite(76, LOW);                                    // Turn off LED
      delay(500);

      driveStraightCM(20);                                      // Drive straight for 20 cm
      
      if( (i < 6) || ((i > 7) && (i < 12)) || (i == 14) ){      // Start and stop straight correction
        rotateCCWShort(1);
      }
   }

      digitalWrite(77, HIGH);       // Light to indicate the ultrasonic is facing straight
      digitalWrite(76, HIGH);       // Light to indicate the ultrasonic is facing straight
      myservo.write(75);            // Tell the servo to turn straight ahead (90 degrees)
      delay(800);
      digitalWrite(77, LOW); 
      digitalWrite(76, LOW); 
   
   rotateCCW(180);                  // Turn around to repeat path

    for(j = 14; j >= 0; j--){       // Performs second 3 meter path run 
     
        if((((distanceToObjectLeft[j] < 110) && (distanceToObjectLeft[j] > 0))) && ((prev == 0) && (prev2 == 0))){    // Determines object location
          
          myservo.write(180);              // tell servo to turn right (180 degrees)
          
          if(prev == 1){    // Determines if around identified this object
            prev2++;
          }
          
          prev++;
          delay(3000);
          
        } else if((((distanceToObjectRight[j] < 110) && (distanceToObjectRight[j] > 0))) & ((prev == 0) && (prev2 == 0))){    // Determines object location
          
          myservo.write(0);              // tell servo to turn left (0 degrees)

         if(prev == 1){         // Determines if around identified this object
            prev2++;
          }
          
          prev++;
          delay(3000);
          
        }

        if ((prev > 0) || (prev2 > 0)){   // Resets servo position
          myservo.write(75);              // Tell the servo to turn straight ahead (90 degrees)
          delay(500); 
        }
        
        prev = 0;

        driveStraightCM(20);        // Drive straight for 20 cm
  
        if( (j > 8) || ((j < 7) && (j > 3)) || (j == 0) ){    // Start and stop straight correction
          rotateCCWShort(1);
        }
    }

delay(10000);

}


//============================================================================
// The sortAndFindMedianLeft() function sorts the distance recordings when
// the servo/ultrasonic is facing the left side of the robot. 
//============================================================================
float sortAndFindMedianLeft(){
  float temp;

  for(int i = 0; i < 9; i++){                         // Sorts the ultrasonic values in ascending order 
      for(int j = i + 1; j < 9; j++){
        if(readingsPerRotate[i] > readingsPerRotate[j]){
          temp = readingsPerRotate[i];
          readingsPerRotate[i] = readingsPerRotate[j];
          readingsPerRotate[j] = temp;
        }
      }  
  }
 
  return readingsPerRotate[4];    // Takes median and adds it to list 

}

//============================================================================
// The sortAndFindMedianRight() function sorts the distance recordings when
// the servo/ultrasonic is facing the right side of the robot  
//============================================================================
float sortAndFindMedianRight(){
  float temp;

  for(int i = 0; i < 9; i++){                         // Sorts the ultrasonic values in ascending order 
      for(int j = i + 1; j < 9; j++){
        if(readingsPerRotate[i] > readingsPerRotate[j]){
          temp = readingsPerRotate[i];
          readingsPerRotate[i] = readingsPerRotate[j];
          readingsPerRotate[j] = temp;
        }
      }  
  }
  
  return readingsPerRotate[4];    // Takes median and adds it to list 
  
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

      if (centimeters < 150){
        
        readingsPerRotate[i] = centimeters;
        
      } else {
        
        readingsPerRotate[i] = 0;
        
      }
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
// The rotateCCWShort() function allows the robot to rotate a specific smaller,
// more precise amount of degrees.
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
