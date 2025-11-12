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
const int m3PinA     = 2;                       // CLK - Interrupt pin (Channel A)
const int m3PinB     = 3;                       // DT  - Regular digital pin (Channel B)
volatile int m3PrevA = LOW;

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
const int M3_EEPROM_ADDR_POS = 0;                // Where encoder count is stored

// ===== MOTOR POSITION ===== //
volatile long m3Position = 0;                   // Current motor position

// ===== MOTOR SHIELD ===== //
DualVNH5019MotorShield md;

// ===== FUNCTION DECLARATIONS ===== //
void initializeMotors();
void updateM3Encoder();
bool checkMotorFaults();
void savedM3EncoderPos();
long loadM3EncoderPos();
long getM3Position();
bool waitForRun();
bool moveM3ToPos(long targetCount, int speed, const char* direction);
void runM4Cont(int speed);
void stopMotor(int motorNum);
bool m3GoToSafePos();
void emergencyStop();
void clearSerialInput();

// ===== FUNCTION IMPLEMENTATIONS ===== //
void initializeMotors(){
  md.init();
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
}

void updateM3Encoder(){
  int currentA = digitalRead(m3PinA);
  int currentB = digitalRead(m3PinB);

  if (currentA != m3PrevA) {
    if (currentA == HIGH) {
      if (currentB == LOW) m3Position++;
      else m3Position--;
    } else {
      if (currentB == HIGH) m3Position++;
      else m3Position--;
    }
  }
  m3PrevA = currentA;
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

void savedM3EncoderPos(){
  noInterrupts();
  long tempPos = m3Position;
  interrupts();
  EEPROM.put(M3_EEPROM_ADDR_POS, tempPos);
  Serial.print("Saved Encoder Position to EEPROM: ");
  Serial.println(tempPos);
}

long loadM3EncoderPos(){
  long savedPos;
  EEPROM.get(M3_EEPROM_ADDR_POS, savedPos);
  Serial.print("Loaded Encoder Position from EEPROM: ");
  Serial.println(savedPos);
  return savedPos;
}

long getM3Position(){
  noInterrupts();
  long pos = m3Position;
  interrupts();
  return pos;
}

bool waitForRun(){
  clearSerialInput();
  Serial.println("Press [SPACE] to start or 'S' to stop.");
  
  unsigned long startWaitTime = millis();
  const unsigned long WAIT_TIMEOUT = 60000;  // 60 second timeout

  while (true) {
    // Check for timeout
    if (millis() - startWaitTime > WAIT_TIMEOUT) {
      Serial.println("Wait timeout - system idle.");
      delay(5000);
      return false;
    }
    
    if (Serial.available() > 0){
      char input = Serial.read();

      if (input == ' ') {
        Serial.println("Starting the sequence...");
        return true;
      } else if (input == 'S' || input == 's'){
        Serial.println("System idle.");
        delay(5000);
        return false;
      } else if (input == 'P'){
        Serial.println("I, Pedro P. Ortiz II, designed this code to work the Middle Man.");
        delay(5000);
        return false;
      }
    }
  }
}

bool moveM3ToPos(long targetCount, int speed, const char* direction){
  // Reset position to 0 for relative movement from current position
  m3Position = 0;
  md.setM2Speed(speed);
  Serial.print("M3 ");
  Serial.print(direction);
  Serial.println("...");

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  while (abs(getM3Position()) < targetCount){
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
      Serial.println(getM3Position());
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
    savedM3EncoderPos();
    Serial.println("Motor 3 stopped and position saved.");
  } else if (motorNum == 4){
    md.setM1Speed(0);
    Serial.println("Motor 4 stopped.");
  } else {
    Serial.println("Invalid motor number dude, use either 1 to 4.");
  }
}

bool m3GoToSafePos(){
  long savedPos = loadM3EncoderPos();
  m3Position = savedPos;

  Serial.println("Moving M3 to safe position...");

  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  if (getM3Position() > SAFE_POS_COUNTS){
    md.setM2Speed(-SAFE_SPEED);  // Negative speed to move down
    
    while (getM3Position() > SAFE_POS_COUNTS){
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
        Serial.println(getM3Position());
        lastPrintTime = millis();
      }
      delay(10);
    }
    
  } else if (getM3Position() < SAFE_POS_COUNTS){
    md.setM2Speed(SAFE_SPEED);  // Positive speed to move up
    
    while (getM3Position() < SAFE_POS_COUNTS){
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
        Serial.println(getM3Position());
        lastPrintTime = millis();
      }
      delay(10);
    }
  }

  md.setM2Speed(0);
  m3Position = SAFE_POS_COUNTS;
  savedM3EncoderPos();
  Serial.println("Ladies and Gentlemen, We Have Landed In The Safe Position");
  return true;
}

void emergencyStop(){
  Serial.println("!! EMERGENCY STOP - RETURNING TO SAFE POSITION !!");

  md.setM2Speed(0);
  md.setM1Speed(0);

  delay(500);

  if (m3GoToSafePos()){
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

// ===== SETUP ===== //
void setup() {
  Serial.begin(115200);
  initializeMotors();

  Serial.println("=== Test Sequence: M3 Down, M4 Run, M3 Up, Stop ===");
  
  long currPos = loadM3EncoderPos();
  m3Position = currPos;

  if (getM3Position() < SAFE_POS_COUNTS) {
    Serial.println("Belt below safe position - raising to safe position...");
    if (!m3GoToSafePos()) {
      Serial.println("ERROR: Failed to reach the safe position. Aborting...");
      while(true);
    }
  } else if (getM3Position() > SAFE_POS_COUNTS) {
    Serial.println("WARNING: Belt is above safe position!");
    Serial.println("Manually lower belt to safe position and press RESET.");
    while(true);
  } else {
    Serial.println("Already in safe position");
  }
  
  delay(1000);
}

// ===== MAIN LOOP ===== //
void loop() {
  if (!waitForRun()) {
    delay(250);
    return;
  }

  m3Position = 0;

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

  savedM3EncoderPos();

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
