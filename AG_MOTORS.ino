#include <EEPROM.h>                 // Safe Positons
#include "Astro_Motors.h"           // Added .h and .cpp for cleaning up the code
#include <DualVNH5019MotorShield.h> // Motor Shield Built-In Library

DualVNH5019MotorShield md;

// ===== Encoder Pins ===== //
const int pinA                    = 3;      // CLK - Interrupt pin (Channel A)
const int pinB                    = 5;      // DT  - Regular digital pin (Channel B)
volatile int prevA                = LOW;

// ===== Constants ===== //
const long SAFE_POS_COUNTS        = 0;      // Encoder count for home/safe zone
const long HOME_OFFSET_COUNTS     = 150;    // Small offset to ensure it's settled
const long countsPerRev           = 100;     // Adjust for the motor rotaion
const int m3DownSpeed             = 100;    // Speed for M3 to lower
const int m3UpSpeed               = 100;    // Speed for M3 to raise
const int m4Speed                 = 200;    // Speed for M4 to move the belt

const int EEPROM_ADDR_POS         = 0;      // Where encoder count is stored
const int SAFE_SPEED              = 150;    // Speed to reach home
const unsigned long MOTOR_TIMEOUT = 1000;
volatile long position            = 0;      // Motor Position


// ===== REFERENCE SPEED TABLE (-400 to +400, with Duty Percentage) ===== //
/*
  -400: Full (MAX) Power Reverse      (100%)
  -300: High Reverse Speed            (75%)
  -200: Medium Reverse Speed          (50%)
  -100: Slow Reverse Speed            (25%)
   -50: Extremely Slow Reverse Speed  (12.5%)
     0: No Speed (IDLE)               (0%)
    50: Extremely Slow Forward Speed  (12.5%)
   100: Slow Forward Speed            (25%)
   200: Medium Forward Speed          (50%)
   300: High Forward Speed            (75%)
   400: Full (MAX) Power Forward      (100%)
*/

// ===== Function Declarations ===== //
void updateEncoder();                           // Encoder Interrupt Service Routine
bool checkMotorFaults();                        // Checks if any motors are faulty
void savedEncoderPos();                         // Saves The Encoder Position To EEPROM
long loadEncoderPos();                          // Load The Encoder Position From EEPROM
long getPosition();                             // 
bool waitForRun();                              // Waits for input activation, then asks after sequence
bool moveM3Down(long targetCount, int speed);   // Move Motor Down (M3)
bool moveM3Up(long targetCount, int speed);     // Move Motor Up (M3)
void runM4Cont(int speed);                      // Run Motor 4 Continuously
void stopMotor(int motorNumber);                // Stop Individual Motor        
bool goToSafePos();                             // Returns The Motors To The Safe Position
void clearSerialInput();                        // Clears the serial input

// ====== Function Definitions ====== //

void updateEncoder(){
  int currentA = digitalRead(pinA);
  int currentB = digitalRead(pinB);

  // Detect state changes on both edges
  if (currentA != prevA) {
    if (currentA == HIGH) {
      // Rising edge
      if (currentB == LOW) position++;
      else position--;
    } else {
      // Falling edge
      if (currentB == HIGH) position++;
      else position--;
    }
  }
  prevA = currentA;
}

bool checkMotorFaults(){
  if (md.getM1Fault()){
    Serial.println("ERROR: M4 fault detected!");
    stopMotor(3);
    stopMotor(4);
    return true;
  }

  if (md.getM2Fault()){
    Serial.println("ERROR: M3 fault detected!");
    stopMotor(3);
    stopMotor(4);
    return true;
  }
  return false;
}

void savedEncoderPos(){
  noInterrupts();
  long tempPos = position;
  interrupts();
  EEPROM.put(EEPROM_ADDR_POS, tempPos);
  Serial.print("Saved Encoder Position to EEPROM: ");
  Serial.println(tempPos);
}

long loadEncoderPos(){
  long savedPos;
  EEPROM.get(EEPROM_ADDR_POS, savedPos);
  Serial.print("Loaded Encoder Position from EEPROM: ");
  Serial.println(savedPos);
  return savedPos;
}

long getPosition(){
  noInterrupts();   // Stops all interrupts temporarily
  long pos = position;  // Safely read the value
  interrupts();   // Re-enable interrupts
  return pos;
}

bool waitForRun(){
  clearSerialInput();
  Serial.println("Press [SPACE] to start or 'S' to stop.");

  while (true){
    if (Serial.available() > 0){
      char input = Serial.read();

      if (input == ' '){
        Serial.println("Starting the sequence...");
        return true;
      } else if (input == 'S' || input == 's'){
        Serial.println("System idle.");
        delay(5000);
        return false;
      }
    }
  }
}

bool moveM3Down(long targetCount, int speed){
  position = 0;
  md.setM2Speed(-speed);
  Serial.println("M3 Lowering...");

  unsigned long start = millis();
  unsigned long lastPrint = 0;

  while (abs(getPosition()) < targetCount){
    // Check for timeout
    if (millis() - start > MOTOR_TIMEOUT){
      Serial.println("ERROR: M3 timeout!");
      stopMotor(3);
      return false;
    }

    // Check motor falut
    if (checkMotorFaults()){
      return false;
    }

    // Print every 100ms
    if (millis() - lastPrint > 100){
      Serial.print("Encoder Count: ");
      Serial.println(getPosition());
      lastPrint = millis();
    }
    delay(10);
  }

  stopMotor(3);
  Serial.println("M3 Reached Lower Position.");
  return true;
}

bool moveM3Up(long targetCount, int speed){
  position = 0;
  md.setM2Speed(speed);
  Serial.println("M3 Raising...");

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  while (abs(getPosition()) < targetCount){
    // Check for timeout
    if (millis() - startTime > MOTOR_TIMEOUT){
      Serial.println("ERROR: M3 up timeout!");
      stopMotor(3);
      return false;
    }

    // Check for motor faults
    if (checkMotorFaults()){
      return false;
    }

    // Print every 100ms instead of 10ms to reduce spam
    if (millis() - lastPrintTime > 100){
      Serial.print("Encoder Count: ");
      Serial.println(getPosition());
      lastPrintTime = millis();
    }
    delay(10);
  }

  stopMotor(3);
  Serial.println("M3 Returned To Start Position.");
  return true;
}

void runM4Cont(int speed){
  md.setM1Speed(speed);
  Serial.print("Motor 4 running continuously at: ");
  Serial.println(speed);
}

void stopMotor(int motorNum){
  if (motorNum == 1){
    // Add changes for M1
  } else if (motorNum == 2){
    // Add chnages for M2
  } else if (motorNum == 3){
    md.setM2Speed(0);
    savedEncoderPos();
    Serial.println("Motor 3 stopped and position saved.");
  } else if (motorNum == 4){
    md.setM1Speed(0);
    Serial.println("Motor 4 stopped.");
  } else {
    Serial.println("Invalid motor number, use either 1 to 4.");
  }
}

bool goToSafePos(){
  long savedPos = loadEncoderPos();
  position = savedPos;

  Serial.println("Moving M3 to safe position...");

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  if (getPosition() > SAFE_POS_COUNTS){
    md.setM2Speed(-SAFE_SPEED);
    
    while (getPosition() > SAFE_POS_COUNTS + HOME_OFFSET_COUNTS){
      // Check for timeout
      if (millis() - startTime > MOTOR_TIMEOUT){
        Serial.println("ERROR: Safe homing timeout!");
        md.setM2Speed(0);
        return false;
      }

      // Check for motor faults
      if (checkMotorFaults()){
        return false;
      }

      // Print every 100ms to reduce spam
      if (millis() - lastPrintTime > 100){
        Serial.print("Safe Homing... Encoder: ");
        Serial.println(getPosition());
        lastPrintTime = millis();
      }
      delay(10);
    }
    
  } else if (getPosition() < SAFE_POS_COUNTS){
    md.setM2Speed(SAFE_SPEED);
    
    while (getPosition() < SAFE_POS_COUNTS - HOME_OFFSET_COUNTS){
      // Check for timeout
      if (millis() - startTime > MOTOR_TIMEOUT){
        Serial.println("ERROR: Safe homing timeout!");
        md.setM2Speed(0);
        return false;
      }

      // Check for motor faults
      if (checkMotorFaults()){
        return false;
      }

      // Print every 100ms to reduce spam
      if (millis() - lastPrintTime > 100){
        Serial.print("Safe Homing. Encoder Count: ");
        Serial.println(getPosition());
        lastPrintTime = millis();
      }
      delay(10);
    }
  }

  md.setM2Speed(0);
  position = SAFE_POS_COUNTS;
  savedEncoderPos();
  Serial.println("Gentlemen, We Have Landed In The Safe Position");
  return true;
}

void clearSerialInput(){
  while (Serial.available() > 0){
    Serial.read();
  }
}

// ===== Full Sequence in Action ===== //
void setup(){
  Serial.begin(115200);
  md.init();    // Initialize Motor Driver

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE);

  Serial.println("=== Test Sequence: M3 Down, M4 Run, M3 Up, Stop ===");
  delay(1000);
}

void loop(){
  // If STOP was entered, it does nothing
  if (!waitForRun()){
    delay(250);
    return;
  }

  if (!goToSafePos()){
    Serial.println("ERROR: FAiled to reach safe posiiton. Aborting...");
    while(true);
  }

  delay(1000);

  position = 0;  // Reset encoder position

  // M3 lowers down
  if (!moveM3Down(countsPerRev, m3DownSpeed)){
    Serial.println("ERROR: M3 down movement failed. Aborting.");
    stopMotor(3);
    stopMotor(4);
    savedEncoderPos();  // Save position on error
    while(true);  // Halt on error
  }

  delay(2000);  // 2 sec

  // M4 runs continuously for 5 seconds
  runM4Cont(m4Speed);
  delay(5000);  // 5 sec
  stopMotor(4);
  delay(2000);  // 2 sec

  // M3 raises back up
  if (!moveM3Up(countsPerRev, m3UpSpeed)){
    Serial.println("ERROR: M3 up movement failed. Aborting.");
    stopMotor(3);
    stopMotor(4);
    savedEncoderPos();  // Save position on error
    while(true);  // Halt on error
  }

  delay(2000);  // 2 sec

  // Stop all motors
  stopMotor(3);
  stopMotor(4);

  savedEncoderPos();

  Serial.println("=== SEQUENCE COMPLETE ===");
  delay(500);
}


// FOR PEROSNAL //
/*
Updating GITHUB:
  - git status
  - git add .
  - git commit -m "Enter any comments"
  - git push
*/

