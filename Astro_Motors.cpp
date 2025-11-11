/**
Astro_Motors.cpp - Implementation File for AstroGlass/Sunburn's Motors

To make it simple, (for me), the header file declares all constants, variables, and functions used in to motor control 
system. It provides the interface between both the main program and the motor control implementation.

Motor Assignments:
  M1 - Plunger
  M2 - Platform
  M3 - Conveyor Lift
  M4 - Belt
[Motors used: 4x Studia Maverick 7 PPR with a 61:1 gear ratio (854 per revolution)]
[Motor Drivers used: 2x DualVNH5019 Motor Shield]
*/

#include "Astro_Motors.h"

// ===== SPEED REFERENCE TABLE (-400 to +400) ===== //
/*
  -400: Full Reverse      (100%)
  -300: High Reverse      (75%)
  -200: Medium Reverse    (50%)
  -100: Slow Reverse      (25%)
     0: Stopped           (0%)
   100: Slow Forward      (25%)
   200: Medium Forward    (50%)
   300: High Forward      (75%)
   400: Full Forward      (100%)
*/

// ===== HARDWARE CONFIGURATION ===== //
const int pinA = 2;                       // CLK - Interrupt 0 (Channel A)
const int pinB = 3;                       // DT  - Interrupt 1 (Channel B)
volatile int prevA = LOW;

// ===== ENCODER & MOTOR PARAMETERS ===== //
const long SAFE_POS_COUNTS = 0;           // Encoder count for safe zone
const long countsPerRev    = 854;         // Total Motor Counts
const long CONST_90_DEG    = 214;         // 90-degree movement

// ===== MOTOR SPEEDS ===== //
const int m3DownSpeed     = 100;          // M3 speed to lower
const int m3UpSpeed       = 100;          // M3 speed to raise
const int m4Speed         = 100;          // M4 speed
const int SAFE_SPEED      = 150;          // Speed to reach home position

// ===== TIMING PARAMETERS ===== //
const unsigned long MOTOR_TIMEOUT_MOVE = 15000;       // 15 seconds for normal moves
const unsigned long MOTOR_TIMEOUT_HOME = 30000;       // 30 seconds for homing
const unsigned long DELAY_AFTER_DOWN   = 2000;        // 2 seconds
const unsigned long M4_RUN_TIME        = 3000;        // 3 seconds
const unsigned long DELAY_AFTER_M4     = 2000;        // 2 seconds
const unsigned long DELAY_AFTER_UP     = 2000;        // 2 seconds
const unsigned long SEQUENCE_PAUSE     = 500;         // 0.5 seconds

// ===== EEPROM CONFIGURATION ===== //
const int EEPROM_ADDR_POS = 0;            // Where encoder count is stored

// ===== MOTOR POSITION ===== //
volatile long position = 0;               // Current motor position

// ===== MOTOR SHIELD ===== //
DualVNH5019MotorShield md_main;
DualVNH5019MotorShield md_external;

// ===== FUNCTION IMPLEMENTATIONS ===== //
// Initialize motor system
void initializeMotors(){
  md_main.init();
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE);
}

// Encoder ISR. Called automatically when ecoder Channel A changes states
void updateEncoder(){
  int currentA = digitalRead(pinA);
  int currentB = digitalRead(pinB);

  // Detet state changes and determine direction
  if (currentA != prevA){
    if (currentA == HIGH){
      if (currentB == LOW) position++;
      else position--;
    } else {
      if (currentB == HIGH) position++;
      else position--;
    }
  }
  prevA = currentA;
}

// Check for motor faults
bool checkMotorFaults(){
  if (md_main.getM1Fault()){
    Serial.println("ERROR: M4 fault detected!");
    stopMotor(Motor::M3);
    stopMotor(Motor::M4);
    return true;
  }

  if (md_main.getM2Fault()){
    Serial.println("ERROR: M3 fault detected!");
    stopMotor(Motor::M3);
    stopMotor(Motor::M4);
    return true;
  }
  return false;
}

// Save encoder position to EEPROM
void savedEncoderPos(){
  noInterrupts();
  long tempPos = position;
  interrupts();
  EEPROM.put(EEPROM_ADDR_POS, tempPos);
  Serial.print("Saved Encoder Position to EEPROM: ");
  Serial.println(tempPos);
}

// Load encoder position from EEPROM
long loadEncoderPos(){
  long savedPos;
  EEPROM.get(EEPROM_ADDR_POS, savedPos);
  Serial.print("Loaded Encoder Position from EEPROM: ");
  Serial.println(savedPos);
  return savedPos;
}

// Get current encoder position safely
long getPosition(){
  noInterrupts();
  long pos = position;
  interrupts();
  return pos;
}

// Waits for the user input (START or 'S')
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
      } else if (input == 'PPOII'){
        Serial.println("Special thanks to my family and friends and the team for the effort on the 'Middle Man'! <3");
        delay(5000);
        return false;
      }
    }
  }
}

// Moves M3 to the target position
bool moveM3ToPos(long targetCount, int speed, const char* direction){
  position = 0;
  md_main.setM2Speed(speed);
  Serial.print("M3 ");
  Serial.print(direction);
  Serial.println("...");

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  while (abs(getPosition()) < targetCount){
    // Check for timeout
    if (millis() - startTime > MOTOR_TIMEOUT_MOVE){
      Serial.print("ERROR: M3 ");
      Serial.print(direction);
      Serial.println(" timeout!");
      stopMotor(Motor::M3);
      return false;
    }

    // Check for motor faults
    if (checkMotorFaults()){
      return false;
    }

    // Print progress every 100ms
    if (millis() - lastPrintTime > 100){
      Serial.print("Encoder Count: ");
      Serial.println(getPosition());
      lastPrintTime = millis();
    }

    delay(10);
  }

  stopMotor(Motor::M3);
  Serial.print("M3 ");
  Serial.print(direction);
  Serial.println(" complete.");
  return true;
}

// Continuously run M4 for the belt
void runM4Cont(int speed){
  md_main.setM1Speed(speed);
  Serial.print("Motor 4 running continuously at: ");
  Serial.println(speed);
}

// Stops individual motors
void stopMotor(int motorNum){
  if (motorNum == Motor::M1){
    // Add changes for M1
  } else if (motorNum == Motor::M2){
    // Add changes for M2
  } else if (motorNum == Motor::M3){
    md_main.setM2Speed(0);
    savedEncoderPos();
    Serial.println("Motor 3 stopped and position saved.");
  } else if (motorNum == Motor::M4){
    md_main.setM1Speed(0);
    Serial.println("Motor 4 stopped.");
  } else {
    Serial.println("Invalid motor number, use either 1 to 4.");
  }
}

// Return to the safe position
bool goToSafePos(){
  long savedPos = loadEncoderPos();
  position = savedPos;

  Serial.println("Moving M3 to safe position...");

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  if (getPosition() > SAFE_POS_COUNTS){
    md_main.setM2Speed(SAFE_SPEED);
    
    while (getPosition() > SAFE_POS_COUNTS){
      if (millis() - startTime > MOTOR_TIMEOUT_HOME){
        Serial.println("ERROR: Safe homing timeout!");
        md_main.setM2Speed(0);
        return false;
      }

      if (checkMotorFaults()){
        return false;
      }

      if (millis() - lastPrintTime > 100){
        // Serial.print("Safe Homing... Encoder: ");
        // Serial.println(getPosition());
        lastPrintTime = millis();
      }
      delay(10);
    }
    
  } else if (getPosition() < SAFE_POS_COUNTS){
    md_main.setM2Speed(-SAFE_SPEED);
    
    while (getPosition() < SAFE_POS_COUNTS) {
      if (millis() - startTime > MOTOR_TIMEOUT_HOME){
        Serial.println("ERROR: Safe homing timeout!");
        md_main.setM2Speed(0);
        return false;
      }

      if (checkMotorFaults()){
        return false;
      }

      if (millis() - lastPrintTime > 100){
        // Serial.print("Safe Homing. Encoder Count: ");
        // Serial.println(getPosition());
        lastPrintTime = millis();
      }
      delay(10);
    }
  }

  md_main.setM2Speed(0);
  position = SAFE_POS_COUNTS;
  savedEncoderPos();
  Serial.println("Gentlemen, We Have Landed In The Safe Position");
  return true;
}

// Emergency stop procedure
void emergencyStop(){
  Serial.println("!! EMERGENCY STOP - RETURNING TO SAFE POSITION !!");

  md_main.setM2Speed(0);
  md_main.setM1Speed(0);

  delay(500);

  if (goToSafePos()){
    Serial.println("Successfully returned to safe position.");
  } else {
    Serial.println("WARNING: Could not return to safe position!");
  }

  Serial.println("System halted. Please reset to continue.");
  while(true);
}

// Clear serial input buffer
void clearSerialInput(){
  while (Serial.available() > 0){
    Serial.read();
  }

}