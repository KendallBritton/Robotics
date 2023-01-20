//*******************************************************************
// This program will allow the robot to complete three parts that   *
// include localization, line following, and maze traversal. The    *
// robot will utilize the TOF sensor attached to a servo motor to   *
// successfully complete the lab.                                   *
//                                                                  *
//                                                                  *
//                                                                  *
// Kendall Britton, 2022-05-02                                      *
//*******************************************************************

#include <Servo.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" 
#include "SimpleRSLK.h"

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

SFEVL53L1X distanceSensor;

#define LEFT_MOTOR_SPEED   15           // Inital left motor speed for traveling striaght
#define RIGHT_MOTOR_SPEED  17           // Inital right motor speed for traveling striaght

#define LEFT_MOTOR_SPEED_ROTATE 10    // Inital left motor speed for rotating
#define RIGHT_MOTOR_SPEED_ROTATE 10   // Inital right motor speed for rotating

#define ERROR_OVERSHOOT_STRAIGHT 16   // Error overshoot that makes encoder more accurate for traveling straight
#define ERROR_OVERSHOOT_ROTATE 18     // Error overshoot that makes encoder more accurate for rotating

#define SENSOR_PLACEMENT_CORRECTION 70   // Distance from ultrasonic to front of robot
#define DISTANCE_FROM_WALL 70   

float wheel_diam = 6.99999;       // Wheel diameter of the TI RSLK
uint16_t gear_ratio = 120;        // Gear ratio of the TI RSLK
uint8_t pulses_per_motor = 3;     // Encoder pulse to motor turn ration for TI RSLK
float wheel_base = 13.99999;      // Wheel base distance of the TI RSLK

int readingsPerRotate[5] = {};         // Array to filter the various sensor measurements

int iter = 0;                       // This variable keeps tracks of how many iterations the servo has moved
float distanceToWall[40] = {};    // Array to hold the filtered sensor measurements
int iterList[40] = {};              // This array holds the mapped servo iterations according to each ultrasonic reading
int set = -1;                       // The variable dictates which set of the sweep the object was detected in

uint16_t sensorVal[LS_NUM_SENSORS];       // Variable that is utilized to store sensor valuies for line following 
uint16_t sensorCalVal[LS_NUM_SENSORS];    // Variable that stores the calibrated value of sensors for line following
uint16_t sensorMaxVal[LS_NUM_SENSORS];    // Variable that stores the max sensor value of sensors for line following
uint16_t sensorMinVal[LS_NUM_SENSORS];    // Variable that stores the min sensor value of sensors for ine following

uint16_t normalSpeedL = 15;       // Initial left motor speed for traveling striaght when line following
uint16_t normalSpeedR = 17;       // Initial right motor speed for traveling straight when line following 

uint8_t lineColor = DARK_LINE;    // DARK_LINE  if your floor is lighter than your line

uint32_t linePos;             // Variable that keeps track of line position when line following
bool lineFound = false;       // Variable that determines if the line is found or not

int turnLeftOrRight = -1;     // Variable that keeps track whether the robot has turned left or right when finding center of room

void setRawMotorSpeed(uint8_t motorNum, uint8_t speed);   // Initializes the raw motor speed function




//============================================================================
// The setup() funtion runs one time at the beginning of the Energia program 
//============================================================================
void setup() {
  setupRSLK();                                      // Set up all of the pins & functions needed to be used by the TI bot
  Wire.begin();                                     // Begin communication to distance sensor
  Serial.begin(9600);
  delay(1000);
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0)                  // Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  
  for (int i = 0; i < 40; i ++){
    distanceToWall[i] = 4000;
  }
  
  Serial.println("Sensor online!");
  myservo.attach(38);                               // attaches the servo on Port 2.4 to the servo object
  myservo.write(0);                                 // Send it to the default position
  clearMinMax(sensorMinVal,sensorMaxVal);
  distanceSensor.setIntermeasurementPeriod(200);    // default 100 ms
  distanceSensor.setDistanceModeLong();             // Set distance sensor to read up to 4000 mm
  setupWaitBtn(LP_LEFT_BTN);
  setupLed(RED_LED);
}


//============================================================================
// The loop() function runs after the setup() function completes in an 
// Energia program and will continue to run in a repeating loop until the 
// LaunchPad is reset or powered off 
//============================================================================
void loop() {

  String btnMsg = "Push left button on Launchpad to begin calibration.\n";      // Variable to setup left button 
  btnMsg += "Make sure the robot is on the floor away from the line.\n";
  

  
  delay(5000);        // Waits for 5 seconds

  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);           // Waits for left button press to start calibration

  delay(1000);

  calibrateFloor();                                     // Runs calibration for line position sensors

  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED);           // Waits for left button press to run rest of program

  delay(1000);

  orientPerpenToWall();                // Finds closest wall and orients robot perpendicular to wall

  travelToCenter();                    // Robot travels to the center of the room

  findLineToFollow();                  // Robot will rotate in 90 degree increments to find line to follow

  followLine();                        // Robot follows the line until it approaches "T"

  traverseMaze();                      // Robots now follows algorithm to traverse and exit maze
 
}


//============================================================================
// The traverseMaze() function allows the robot to traverse the maze by 
// either hugging the left or right wall depending on the initial turn
//============================================================================
void traverseMaze(){
  int initialLeftDistance, initialRightDistance, initialForwardDistance;      // Variables to dictate which direction the robot will initially turn
  int leftDistance, rightDistance, forwardDistance;                           // Variables to hold the distances of the robots surroundings
  int prevLeftDistance, prevRightDistance, prevForwardDistance;               // Variables to hold the previous distance of the robots surroundings

  int stuckLoop = 0;                         // Variable to dictate if the robot is stuck in a corner
  int turnON = 0;                            // Variable that dictates if the robot has turned a corner

  int distanceThreshold;                     // Threshold that the robot uses to stay away from the wall

  prevLeftDistance = 4000;                   // Predefines variables for better comparison upon start
  prevRightDistance = 4000;
  prevForwardDistance = 4000;
  
  int done = -1;                             // Variable to keep track if the maze is done
  int followLeftWallOrRightWall = -1;        // Variable to determine which wall to follow


  myservo.write(0);                                   // Pulses left and stores initial left value
  delay(500);
  pulseDistanceSensor();
  initialLeftDistance = sortAndFindMedianCenter();

  myservo.write(180);                                 // Pulses right and stores initial right value
  delay(500);
  pulseDistanceSensor();
  initialRightDistance = sortAndFindMedianCenter();

  myservo.write(72);                                  // Pulses straight and stores initial forward value
  delay(500);
  pulseDistanceSensor();
  initialForwardDistance = sortAndFindMedianCenter();

  distanceThreshold = initialForwardDistance - DISTANCE_FROM_WALL - SENSOR_PLACEMENT_CORRECTION;       // Calculates the distance threshold by staying 14 cm from wall

  if(initialLeftDistance > initialRightDistance){                                                      // If left distance is greater, turn left and follow right wall
    
    driveStraightCM(initialForwardDistance - DISTANCE_FROM_WALL - SENSOR_PLACEMENT_CORRECTION);
    rotateCCW(90);
    followLeftWallOrRightWall =  1;     
    
  } else if(initialLeftDistance < initialRightDistance){                                              // If right distance is greater, turn right and follow left wall
    
    driveStraightCM(initialForwardDistance - DISTANCE_FROM_WALL - SENSOR_PLACEMENT_CORRECTION);
    rotateCW(90);
    followLeftWallOrRightWall =  2;     
    
  }

while(done != 1){                                                       // Loops until course is complete
    if (followLeftWallOrRightWall == 1){                                // Follow Right Wall 
          myservo.write(180);                                           // Pulse and store right distance reading
          delay(500);
          pulseDistanceSensor();
          rightDistance = sortAndFindMedianCenter();

          myservo.write(72);                                            // Pulse and store forward distance reading
          delay(500);
          pulseDistanceSensor();
          forwardDistance = sortAndFindMedianCenter();

          stuckLoop++;                                                  // Increments stuck loop if action is repeat more than once


          if( (rightDistance > (prevRightDistance * 1.5)&& (rightDistance > distanceThreshold * 2.5))){         // Checks to see if a right turn is avaiable
            
            driveStraightCM(100);                                       // Drive striaght to clear wall
            rotateCW(90);                                               // Turn right to begin to pass wall 
            
            myservo.write(72);                                          // Pulse and store forward distance reading
            delay(500);
            pulseDistanceSensor();
            forwardDistance = sortAndFindMedianCenter();
            
            if (forwardDistance > 240){                                 // If distance is greater than threshold, travel two times previous right wall distance to clear wall
              driveStraightCM(2 * prevRightDistance);
            } else {
              driveStraightCM(50);                                      // Else drive slow along object
            }
            
            stuckLoop = 0;                            // Resets stuck loop
            turnON = 1;                               // Indicates that a turn happened 

            rotateCW(90);                             // Turn right to be parallel with wall
                   
            myservo.write(72);                        // Pulse and store forward distance reading
            delay(500);
            pulseDistanceSensor();
            forwardDistance = sortAndFindMedianCenter();

            if (forwardDistance < 240){               // If object is less than threshold, turn back left (obstacle)
              rotateCCW(90);
            }
            
            driveStraightCM(100);                     // Drives past obstacle
          }  
          
          if(forwardDistance > distanceThreshold){      // If able to move forward
            if(turnON == 1){

            } else {
              driveStraightCM(100);                     // Drive along wall

              myservo.write(0);                         // Pulse and store left distance reading
              delay(500);
              pulseDistanceSensor();
              leftDistance = sortAndFindMedianCenter();

              if(leftDistance < 150){                   // If too close to one side fix orientation of robot
                rotateCW(10);
              } else if (rightDistance < 150){
                rotateCCW(10);
              }
            }
            
            prevForwardDistance = forwardDistance;      // Updates previous readings
            prevRightDistance = rightDistance;
            prevLeftDistance = leftDistance;
            stuckLoop = 0;
            turnON = 0;
          } else if (stuckLoop > 1){                    // If stuck in corner, turn left
            rotateCCW(90);
            prevForwardDistance = forwardDistance;      // Updates previous readings
            prevRightDistance = rightDistance;
            prevLeftDistance = leftDistance;
            stuckLoop = 0;
            turnON = 0;
          }    
    }
    

    if (followLeftWallOrRightWall == 2){                // Follow Left Wall 
          myservo.write(0);                             // Pulse and store left distance reading
          delay(500);
          pulseDistanceSensor();
          leftDistance = sortAndFindMedianCenter();

          myservo.write(72);                            // Pulse and store forward distance reading
          delay(500);
          pulseDistanceSensor();
          forwardDistance = sortAndFindMedianCenter();

          stuckLoop++;                                  // Increments stuck loop if action is repeat more than once


          if( (leftDistance > (prevLeftDistance * 1.5)&& (leftDistance > distanceThreshold * 2.5))){        // Checks to see if a left turn is avaiable
            
            driveStraightCM(100);                                               // Drive striaght to clear wall
            rotateCCW(90);                                                      // Turn left to begin to pass wall
            
            myservo.write(72);                                                  // Pulse and store forward distance reading
            delay(500);
            pulseDistanceSensor();
            forwardDistance = sortAndFindMedianCenter();
            
            if (forwardDistance > 240){                                         // If distance is greater than threshold, travel two times previous left wall distance to clear wall
              driveStraightCM(2 * prevLeftDistance);
            } else {
              driveStraightCM(50);                                              // Else drive slow along object
            }
            
            stuckLoop = 0;                                                      // Resets stuck loop
            turnON = 1;                                                         // Indicates that a turn happened

            rotateCCW(90);                                                      // Turn right to be parallel with wall
                   
            myservo.write(72);                                                  // Pulse and store forward distance reading
            delay(500);
            pulseDistanceSensor();
            forwardDistance = sortAndFindMedianCenter();

            if (forwardDistance < 240){                                     // If object is less than threshold, turn back right (obstacle)
              rotateCW(90);
            }
            
            driveStraightCM(100);                                           // Drives past obstacle
          }  
          
          if(forwardDistance > distanceThreshold){                          // If able to move forward
            if(turnON == 1){

            } else {
              driveStraightCM(100);                                         // Drive along wall

              myservo.write(180);                                           // Pulse and store right distance reading
              delay(500);
              pulseDistanceSensor();
              rightDistance = sortAndFindMedianCenter();

              if(leftDistance < 150){                                       // If too close to one side fix orientation of robot
                rotateCW(10);
              } else if (rightDistance < 150){
                rotateCCW(10);
              }
            }
            
            prevForwardDistance = forwardDistance;                          // Updates previous readings
            prevRightDistance = rightDistance;
            prevLeftDistance = leftDistance;
            stuckLoop = 0;
            turnON = 0;
          } else if (stuckLoop > 1){                                        // If stuck in corner, turn left
            
            rotateCCW(90);
            prevForwardDistance = forwardDistance;                          // Updates previous readings
            prevRightDistance = rightDistance;
            prevLeftDistance = leftDistance;
            stuckLoop = 0;
            turnON = 0;
          }
    }   
  }
  
  myservo.write(72);
  delay(500);
    
}


//============================================================================
// The followLine() function allows the robot to follow the line using 
// the line position sensors on the bottom of the robots
//============================================================================
void followLine(){
    while(linePos != 0){                  // Travels the line unless position is zero
      
      readLineSensor(sensorVal);                                                        // Reads sensor values
      readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
    
      linePos = getLinePosition(sensorCalVal,lineColor);          // Update line position
            delay(10); 
  
      if(linePos > 0 && linePos < 2000) {                         // If too far left speed up right stop left
        
        setMotorSpeed(LEFT_MOTOR,0);
        setMotorSpeed(RIGHT_MOTOR,normalSpeedR + 5);
        
      } else if (linePos > 2000 && linePos < 2500){              // If too far left speed up right and slow down left
        
        setMotorSpeed(LEFT_MOTOR,normalSpeedL - 8);
        setMotorSpeed(RIGHT_MOTOR,normalSpeedR + 3);
        
      } else if(linePos > 4000 && linePos < 4500) {              // If too far right speed up left and slow down right
        
        setMotorSpeed(LEFT_MOTOR,normalSpeedL + 3);
        setMotorSpeed(RIGHT_MOTOR,normalSpeedR - 8);
        
      } else if(linePos > 4500) {
        
        setMotorSpeed(LEFT_MOTOR,normalSpeedL + 5);             // If too far right speed up left stop right
        setMotorSpeed(RIGHT_MOTOR,0);
        
      } else if(linePos == 0) {                                 // If position = 0, stop motors
        
        setMotorSpeed(LEFT_MOTOR,0);
        setMotorSpeed(RIGHT_MOTOR,0);      
    
      }else {                                                   // Normal traversal speed
        
        setMotorSpeed(LEFT_MOTOR,normalSpeedL - 5);
        setMotorSpeed(RIGHT_MOTOR,normalSpeedR - 5);
        
      }
  }

  delay(500);

  rotateCCW(20);
}


//============================================================================
// The findLineToFollow() function allows the robot to find line using 
// the line position sensors on the bottom of the robots
//============================================================================
void findLineToFollow(){
  
  int count = 0;                        // Variable to see if line was ever found in first rotation

  if(turnLeftOrRight == 0){             // Determines which was to turn to find line based on previous turn
    rotateCW(90);  
  } else if (turnLeftOrRight == 1){
    rotateCCW(90);
  }

  
    
    while (lineFound == false){         // Runs until line is found
    
      readLineSensor(sensorVal);                                                        // Reads sensor values
      readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
      linePos = getLinePosition(sensorCalVal,lineColor);                                // Update line position
  
      for(int i = 0; i < 8; i++){           // Searches for dark line
        if((sensorVal[i] >= 2000)){
          lineFound = true;
        }
      }
      
      if(lineFound == true){                                    // If line is found 
        disableMotor(BOTH_MOTORS);                              // Fix motor setting
        setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
        enableMotor(BOTH_MOTORS);
      } else{                                                   // If not found, rotate again 
         if(turnLeftOrRight == 0){
            rotateCW(90);  
          } else if (turnLeftOrRight == 1){
            rotateCCW(90);
          } 
         count++;
         if (count == 4){               // If not found in one rotation, find center again
            orientPerpenToWall();
            travelToCenter();
            turnLeftOrRight = -1;
            count = 0;
         }
      } 

  }
}


//============================================================================
// The calibrateFloor() function allows the robot to calibrate the sensor 
// values for line poition sensors
//============================================================================
void calibrateFloor(){

  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);       // Set both motors direction forward
  enableMotor(BOTH_MOTORS);                               // Enable both motors
  setMotorSpeed(BOTH_MOTORS,20);                          // Set both motors speed 20
  
  for(int x = 0;x<100;x++){                                 // Records sensors measurements
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
  }

  disableMotor(BOTH_MOTORS);        // Disables motors
  
}


//============================================================================
// The travelToCenter() function allows the robot to calculate its position 
// within its surroundings and travel to the center of the room
//============================================================================
void travelToCenter(){
  int distanceXLeft, distanceXRight, distanceYFront, distanceYBack, driveX, driveY;     // Variables used to store distance measurements

  myservo.write(72);                                // Turn servo straight
  delay(500);

  pulseDistanceSensor();                            // Pulse and store front distance measurement 
  delay(10);
  distanceYFront = sortAndFindMedianCenter();

  rotateCCW(180);                                  // Turn around
    
  pulseDistanceSensor();                           // Pulse and store back distance measurement
  delay(10);
  distanceYBack = sortAndFindMedianCenter();

  driveY = abs(distanceYBack + distanceYFront)/2;     // Centerpoint in vertical axis

  myservo.write(0);                                   // Pulse and store left distance measurement 
  delay(500);
  pulseDistanceSensor();
  delay(10);
  distanceXLeft = sortAndFindMedianCenter();
  
  myservo.write(180);                                 // Pulse and store right distance measurement 
  delay(500);
  pulseDistanceSensor();
  delay(10);
  distanceXRight = sortAndFindMedianCenter();

  driveX = abs(distanceXRight + distanceXLeft)/2;     // Centerpoint in horizontal axis
  
  driveStraightCM( (driveY - distanceYFront ));       // Drive to centerpoint in vertical axis
  

  if (distanceXRight > distanceXLeft){                // Turns towards larger side 
    rotateCW(90);
    turnLeftOrRight = 0;
    driveStraightCM( (driveX - distanceXLeft));       // Drive to centerpoint in horizontal axis
  }
  
  if (distanceXRight < distanceXLeft){                // Turns towards larger side 
    rotateCCW(90);
    turnLeftOrRight = 1;
    driveStraightCM( (driveX - distanceXRight));      // Drive to centerpoint in horizontal axis
  }
  
}


//============================================================================
// The orientPerpenToWall() function allows the robot to calculate its position 
// alongside the wall to orient itself to be perpendicular to the wall
//============================================================================
void orientPerpenToWall(){
  int pos = 0;         // variable to store the servo position
  myservo.write(0);    // Send it to the default position

  delay(500);


  for(pos = 0; pos < 180; pos += 9){  // Performs first sweep to find object
     myservo.write(pos);              // tell servo to go to position in variable 'pos' 
     pulseDistanceSensor();
     delay(10);                       // waits 15ms for the servo to reach the position 
     sortAndFindMedian(); 
     delay(500);                      // waits 500ms for the servo to reach the position 
  }         

   rotateCCW(180);             // Robot turns around to perform second sweep

   myservo.write(0);           // Send it to the default position
   delay(500);

   for(pos = 0; pos < 180; pos += 9){  // Performs second sweep to find object
   myservo.write(pos);              // tell servo to go to position in variable 'pos'  
   pulseDistanceSensor();
   delay(10);                       // waits 15ms for the servo to reach the position 
   sortAndFindMedian(); 
   delay(500);                      // waits 500ms for the servo to reach the position 
  }
  
  sortFinalResults();     // Sorts the readings to find the closest object  

  delay(10); 
  
  rotateCCW(90);          // Turns 90 degrees to set reference point for object

  if(set == 0){       // Drives to object if it is in first sweep
    
      rotateCCW((180 - (iterList[0] * 9)));
      
  } else if (set == 1){   // Drives to object if it is in second sweep

      rotateCCW(180);
      rotateCCW((360 - (iterList[0] * 9)));
  }

  rotateCW(25);

  
}


//============================================================================
// The sortFinalResults() function sorts the total distance recordings
// of the closest wall. It also detects which sweep the object was found in.
//============================================================================
void sortFinalResults(){
  int temp;

  for(int i = 0; i < 40; i++){                         // Sorts the ultrasonic values in ascending order 
      for(int j = i + 1; j < 40; j++){
        if(distanceToWall[i] > distanceToWall[j]){
          temp = distanceToWall[i];
          distanceToWall[i] = distanceToWall[j];
          distanceToWall[j] = temp;

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
// The sortAndFindMedian() function sorts the distance recordings when
// the servo/ultrasonic is facing the right side of the robot  
//============================================================================
void sortAndFindMedian(){
  float temp;

  for(int i = 0; i < 5; i++){                         // Sorts the ultrasonic values in ascending order 
      for(int j = i + 1; j < 5; j++){
        if(readingsPerRotate[i] > readingsPerRotate[j]){
          temp = readingsPerRotate[i];
          readingsPerRotate[i] = readingsPerRotate[j];
          readingsPerRotate[j] = temp;
        }
      }  
  }

  distanceToWall[iter] = readingsPerRotate[2];    // Takes median and adds it to list
  iterList[iter] = iter;                            // Assigns current iteration to list
  iter++;
    
}

//============================================================================
// The sortAndFindMedianCenter() function sorts the distance recordings when
// the servo/ultrasonic is facing the right side of the robot  
//============================================================================
float sortAndFindMedianCenter(){
  float temp;

  for(int i = 0; i < 5; i++){                         // Sorts the ultrasonic values in ascending order 
      for(int j = i + 1; j < 5; j++){
        if(readingsPerRotate[i] > readingsPerRotate[j]){
          temp = readingsPerRotate[i];
          readingsPerRotate[i] = readingsPerRotate[j];
          readingsPerRotate[j] = temp;
        }
      }  
  }

   return readingsPerRotate[2];    // Takes median and adds it to list
    
}


//============================================================================
// The pulseDistanceSensor() function calculates/pulses the distance recording
// of the closest object.
//============================================================================
void pulseDistanceSensor(){
  float pulseLength;
  int millimeters;

    for(int i = 0; i < 5; i++){

      distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      while (!distanceSensor.checkForDataReady())
      {
        delay(10);
      }
      int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
      distanceSensor.clearInterrupt();
      distanceSensor.stopRanging();
      millimeters = distance;

      //Serial.print("Sensor Reading:  ");
      //Serial.println(centimeters);

      //delay(100);

      if (millimeters < 4000){
        
        readingsPerRotate[i] = millimeters;
        
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
  float temp = ((distanceCM * 0.1)  * (1 / (wheel_diam * PI)) * gear_ratio * pulses_per_motor) - ERROR_OVERSHOOT_STRAIGHT;
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
  delay(500);    // Pauses before next action
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

   if ((l_totalCount + 1) < r_totalCount){             // Adjusts speed if one motor is faster than the other
     setMotorSpeed(LEFT_MOTOR, l_motor_speed + 1);
    }

    if((r_totalCount + 1) < l_totalCount){            // Adjusts speed if one motor is faster than the other
      setMotorSpeed(LEFT_MOTOR, l_motor_speed - 1);
   }
    
  }
  delay(500);      // Pauses before next action                    
}
