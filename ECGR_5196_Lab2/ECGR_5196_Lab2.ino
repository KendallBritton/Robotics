//*******************************************************************
// This program will repeatedly light the RGB LED in accordance     *
// to the specific pattern that is outlined in the Lab 2 file.      *
// The LED will update every 0.5 second. The pattern is as follows: *
//                                                                  *
// Pattern: Off, Red, Blue, Green, Red & Green, Blue & Green,       *
//          Red & Blue, Red & Blue & Green, Off (repeat)            *
//                                                                  *
// Kendall Britton, 2022-01-31                                      *
//*******************************************************************

#define RED 75    // Define RED of the tri-color LED as pin 75
#define GREEN 76  // Define GREEN of the tri-color LED as pin 76
#define BLUE 77   // Define BLUE of the tri-color LED as pin 77
  
// the setup routine runs once when you press reset:
void setup() {                
  // initialize the various digital pins as outputs
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT); 
  pinMode(BLUE, OUTPUT);      
}

// the loop routine runs over and over again forever:
void loop() {

  // Turns off all the LEDs (Stage 1)
  digitalWrite(RED, LOW);   
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW); 
  delay(500);               // wait for half a second

  // Turns on only the Red LED (Stage 2)
  digitalWrite(RED, HIGH);   
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW); 
  delay(500);               // wait for half a second

  // Turns on only the Blue LED (Stage 3)
  digitalWrite(RED, LOW); 
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, HIGH); 
  delay(500);               // wait for half a second

  // Turns on only the Green LED (Stage 4)
  digitalWrite(RED, LOW);   
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW); 
  delay(500);               // wait for half a second

  // Turns on only the Red and Green LEDs (Stage 5)
  digitalWrite(RED, HIGH);   
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, LOW); 
  delay(500);               // wait for half a second

  // Turns on only the Blue and Green LEDs (Stage 6)
  digitalWrite(RED, LOW);   
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH); 
  delay(500);               // wait for half a second

  // Turns on only the Red and Blue LEDs (Stage 7)
  digitalWrite(RED, HIGH);   
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, HIGH); 
  delay(500);               // wait for half a second

  // Turns on all LEDs (Stage 8)
  digitalWrite(RED, HIGH);   
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH); 
  delay(500);               // wait for half a second
}
