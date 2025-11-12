#include <EEPROM.h>
#include <DualVNH5019MotorShield.h>

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
const int pinA     = 2;                       // CLK - Interrupt pin (Channel A)
const int pinB     = 3;                       // DT  - Regular digital pin (Channel B)
volatile int prevA = LOW;

// ===== ENCODER & MOTOR PARAMETERS ===== //
const long SAFE_POS_COUNTS = 0;               // Encoder count for home/safe zone
const long countsPerRev    = 854;             // Maverick: 7 PPR × 2 edges × 61:1 gear
const long CONST_90_DEG    = 214;             // 90-degree movement (1/4 revolution)

// ===== MOTOR SPEEDS ===== //
const int m3Speed     = 100;                  // Speed for M3 to raise
const int m4Speed     = 200;                  // Speed for M4 belt movement
const int SAFE_SPEED  = 150;                  // Speed to reach home position

// ===== TIMING PARAMETERS ===== //
const unsigned long MOTOR_TIMEOUT_MOVE = 5000;    // 5 seconds for normal moves
const unsigned long MOTOR_TIMEOUT_HOME = 15000;   // 15 seconds for homing
const unsigned long DELAY_AFTER_DOWN   = 2000;    // 2 seconds
const unsigned long M4_RUN_TIME        = 3000;    // 3 seconds
const unsigned long DELAY_AFTER_M4     = 2000;    // 2 seconds
const unsigned long DELAY_AFTER_UP     = 2000;    // 2 seconds
const unsigned long SEQUENCE_PAUSE     = 500;     // 0.5 seconds

// ===== EEPROM CONFIGURATION ===== //
const int EEPROM_ADDR_POS = 0;                // Where encoder count is stored

// ===== MOTOR POSITION ===== //
volatile long position = 0;                   // Current motor position

// ===== MOTOR SHIELD ===== //
DualVNH5019MotorShield md;

// ===== FUNCTION DECLARATIONS ===== //
void initializeMotors();
void updateEncoder();
bool checkMotorFaults();
void savedEncoderPos();
long loadEncoderPos();
long getPosition();
bool waitForRun();
bool moveM3ToPos(long targetCount, int speed, const char* direction);
void runM4Cont(int speed);
void stopMotor(int motorNum);
bool goToSafePos();
void emergencyStop();
void clearSerialInput();

// ===== SETUP ===== //
void setup() {
  Serial.begin(115200);
  initializeMotors();

  Serial.println("=== Test Sequence: M3 Down, M4 Run, M3 Up, Stop ===");
  
  long currPos = loadEncoderPos();
  position = currPos;

  if (getPosition() < SAFE_POS_COUNTS) {
    Serial.println("Belt below safe position - raising to safe position...");
    if (!goToSafePos()) {
      Serial.println("ERROR: Failed to reach the safe position. Aborting...");
      while(true);
    }
  } else if (getPosition() > SAFE_POS_COUNTS) {
    Serial.println("WARNING: Belt is above safe position!");
    Serial.println("Manually lower belt to safe position and press RESET.");
    while(true);
  } else {
    Serial.println("Already in safe position");
  }
  
  delay(1000);
}

// ===== FUNCTION IMPLEMENTATIONS ===== //
void initializeMotors(){
  md.init();
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE);
}

void updateEncoder(){
  int currentA = digitalRead(pinA);
  int currentB = digitalRead(pinB);

  if (currentA != prevA) {
    if (currentA == HIGH) {
      if (currentB == LOW) position++;
      else position--;
    } else {
      if (currentB == HIGH) position++;
      else position--;
    }
  }
  prevA = currentA;
}

bool checkMotorFaults(){
  if (md.getM1Fault()) {
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
  noInterrupts();
  long pos = position;
  interrupts();
  return pos;
}

bool waitForRun(){
  clearSerialInput();
  Serial.println("Press [SPACE] to start or 'S' to stop.");

  while (true) {
    if (Serial.available() > 0){
      char input = Serial.read();

      if (input == ' ') {
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

bool moveM3ToPos(long targetCount, int speed, const char* direction){
  // Reset position to 0 for relative movement from current position
  position = 0;
  md.setM2Speed(speed);
  Serial.print("M3 ");
  Serial.print(direction);
  Serial.println("...");

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  while (abs(getPosition()) < targetCount){
    if (millis() - startTime > MOTOR_TIMEOUT_MOVE){
      Serial.print("ERROR: M3 ");
      Serial.print(direction);
      Serial.println(" timeout!");
      stopMotor(3);
      return false;
    }

    if (checkMotorFaults()){
      return false;
    }

    if (millis() - lastPrintTime > 100){
      Serial.print("Encoder Count: ");
      Serial.println(getPosition());
      lastPrintTime = millis();
    }
    delay(10);
  }

  stopMotor(3);
  Serial.print("M3 ");
  Serial.print(direction);
  Serial.println(" complete.");
  return true;
}

void runM4Cont(int speed){
  md.setM1Speed(speed);
  Serial.print("Motor 4 running continuously at: ");
  Serial.println(speed);
}

void stopMotor(int motorNum){
  if (motorNum == 1) {
    // Add changes for M1
  } else if (motorNum == 2){
    // Add changes for M2
  } else if (motorNum == 3){
    md.setM2Speed(0);
    savedEncoderPos();
    Serial.println("Motor 3 stopped and position saved.");
  } else if (motorNum == 4){
    md.setM1Speed(0);
    Serial.println("Motor 4 stopped.");
  } else {
    Serial.println("Invalid motor number dude, use either 1 to 4.");
  }
}

bool goToSafePos(){
  long savedPos = loadEncoderPos();
  position = savedPos;

  Serial.println("Moving M3 to safe position...");

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  if (getPosition() > SAFE_POS_COUNTS){
    md.setM2Speed(-SAFE_SPEED);  // Negative speed to move down
    
    while (getPosition() > SAFE_POS_COUNTS){
      if (millis() - startTime > MOTOR_TIMEOUT_HOME){
        Serial.println("ERROR: Safe homing timeout!");
        md.setM2Speed(0);
        return false;
      }

      if (checkMotorFaults()){
        return false;
      }

      if (millis() - lastPrintTime > 100){
        Serial.print("Safe Homing... Encoder: ");
        Serial.println(getPosition());
        lastPrintTime = millis();
      }
      delay(10);
    }
    
  } else if (getPosition() < SAFE_POS_COUNTS){
    md.setM2Speed(SAFE_SPEED);  // Positive speed to move up
    
    while (getPosition() < SAFE_POS_COUNTS){
      if (millis() - startTime > MOTOR_TIMEOUT_HOME){
        Serial.println("ERROR: Safe homing timeout!");
        md.setM2Speed(0);
        return false;
      }

      if (checkMotorFaults()){
        return false;
      }

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
  Serial.println("Ladies and Gentlemen, We Have Landed In The Safe Position");
  return true;
}

void emergencyStop(){
  Serial.println("!! EMERGENCY STOP - RETURNING TO SAFE POSITION !!");

  md.setM2Speed(0);
  md.setM1Speed(0);

  delay(500);

  if (goToSafePos()){
    Serial.println("Successfully returned to safe position.");
  } else {
    Serial.println("WARNING: Could not return to safe position!");
  }

  Serial.println("System halted. Please reset to continue.");
  while(true);
}

void clearSerialInput(){
  while (Serial.available() > 0){
    Serial.read();
  }
}

// ===== MAIN LOOP ===== //
void loop() {
  if (!waitForRun()) {
    delay(250);
    return;
  }

  position = 0;

  // M3 lowers down
  if (!moveM3ToPos(CONST_90_DEG, -m3Speed, "Lowering")) {
    emergencyStop();
  }

  delay(DELAY_AFTER_DOWN);

  // M4 runs continuously
  runM4Cont(m4Speed);
  delay(M4_RUN_TIME);
  stopMotor(4);
  delay(DELAY_AFTER_M4);

  // M3 raises back up
  if (!moveM3ToPos(CONST_90_DEG, m3Speed, "Raising")) {
    emergencyStop();
  }

  delay(DELAY_AFTER_UP);

  // Ensure M4 is stopped
  stopMotor(4);

  savedEncoderPos();

  Serial.println("=== SEQUENCE COMPLETE ===");
  delay(SEQUENCE_PAUSE);
}

// ===== FOR PERSONAL ===== //
/*
Updating GITHUB:
  - git status
  - git add .
  - git commit -m "Enter any comments"
  - git push
*/
