/*
=============================================================== //
   PROJECT:   AstroGlass, The  Middle Man, Control System
   PLATFORM:  Arduino MEGA 2560
   DRIVER:    x2 DualVNH5019 Motor Shield
   MOTOR:     Maverick 12V DC Gear Motor w/Encoder (61:1)
   AUTHOR:    Pedro Ortiz
   VERSION:   v0.30
=============================================================== //
*/

#include <EEPROM.h>
#include <Arduino.h>
#include <DualVNH5019MotorShield.h>

// =========== SPEED REFERENCE TABLE (-400 to +400) =========== //
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

// ==================== SHIELD 2 CONTROL PINS ================= //
// M1 Plunger Motor
const int M1_INA = 30;                  // M1 Direction A
const int M1_INB = 31;                  // M1 Direction B
const int M1_PWM = 12;                  // M1 PWM

// M2 Platform Motor
const int M2_INA = 0;                   // M2 Direction A
const int M2_INB = 0;                   // M2 Direction B
const int M2_PWM = 0;                   // M2 PWM

// ======================= ENCODER PINS ======================= //
// M1 Encoder
const int m1PinA     = 18;
const int m1PinB     = 19;
volatile int m1PrevA = LOW;

// M2 Encoder
const int m2PinA     = 0;
const int m2PinB     = 0;
volatile int m2PrevA = LOW;

// M3 Encoder
const int m3PinA     = 2;              // Encoder Channel A, Blue
const int m3PinB     = 3;              // Encoder Channel B, Yellow
volatile int m3PrevA = LOW;

// ============== MOTOR-SPECIFIC PARAMETERS =================== //
// M1 Plunger Parameters
const long M1_EXTEND_COUNTS = 200;     // How far plunger extends (adjust after testing)
const long M1_HOLD_TIME     = 2000;    // Hold time in milliseconds (2 seconds)
const int m1Speed           = 25;

// M2 Platform Parameters
// COMING SOON

// M3 Conveyor Parameters
const long SAFE_POS_COUNTS   = 0;             // Encoder count for home/safe zone
const long CONST_90_DEG      = 214;           // 90-degree movement (1/4 revolution)
const long M3_MAX_VALID_POS  = 500;           // Maximum valid encoder position
const long M3_MIN_VALID_POS  = -500;          // Minimum valid encoder position
const int m3Speed            = 75;            // Speed for M3 to raise
const int SAFE_SPEED         = 150;           // Speed to reach home position
const int M3_EEPROM_ADDR_POS = 0;             // Where encoder count is stored

// M4 Belt Parameters
const int m4Speed = 300;                      // Speed for M4 belt movement

// Extra Parameters
const long countsPerRev = 854;                // Maverick: 7 PPR × 2 edges × 61:1 gear

// ===================== TIMING PARAMETERS ==================== //
const unsigned long MOTOR_TIMEOUT_MOVE  = 15000;   // 15 seconds for normal moves
const unsigned long MOTOR_TIMEOUT_HOME  = 15000;   // 15 seconds for homing
const unsigned long M1_ACTIVATION_DELAY = 10000;   // 10 seconds before M1 activates
const unsigned long M4_TOTAL_RUN_TIME   = 30000;   // 30 seconds for M4 to run
const unsigned long SEQUENCE_PAUSE      = 500;     // 0.5 seconds

// ================ EEPROM CONFIGURATION ====================== //
const int EEPROM_SYSTEM_STATE_ADDR = 4;      // Address for system state flag
const byte SYSTEM_RUNNING         = 0xAA;   // Magic byte: System running
const byte SYSTEM_SAFE            = 0x55;   // Magic byte: System shutdown

// ==================== MOTOR POSITIONS ======================= //
volatile long m1Position = 0;       // M1 Encoder Position
volatile long m2Position = 0;       // M2 Encoder Position
volatile long m3Position = 0;       // M3 Encoder Position

// ===== MOTOR SHIELD ===== //
DualVNH5019MotorShield md;

// =================== FUNCTION DECLARATIONS ================== //
// System Functions
void initializeMotors();
bool checkMotorFaults();
bool validateAndFixEEPROM();
bool waitForRun();
void clearSerialInput();
void emergencyStop();
void stopMotor(int motorNum);
bool allMotorsToSafePos();

// M1 Functions
void updateM1Encoder();
long getM1Position();
void setM1Direction(int dir);
bool runM1Sequence();

// M2 Functions
void updateM2Encoder();

// M3 Functions
void updateM3Encoder();
long getM3Position();
void savedM3EncoderPos();
long loadM3EncoderPos();
bool moveM3ToPos(long targetCount, int speed, const char* direction);

// M4 Functions
void runM4Cont(int speed);
void clearM4Fault();

// =================== SYSTEM FUNCTIONS ======================= //
void initializeMotors(){
  // Initialize Shield 1 (M3 & M4)
  md.init();
  
  // Initialize M1 control pins (Shield 2)
  pinMode(M1_INA, OUTPUT);
  pinMode(M1_INB, OUTPUT);
  digitalWrite(M1_INA, LOW);
  digitalWrite(M1_INB, LOW);
  
  // Initialize M1 encoder
  pinMode(m1PinA, INPUT_PULLUP);
  pinMode(m1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m1PinA), updateM1Encoder, CHANGE);
  
  // Initialize M3 encoder
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
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

bool validateAndFixEEPROM(){
  Serial.println("Validating EEPROM data...");
  
  long savedPos;
  EEPROM.get(M3_EEPROM_ADDR_POS, savedPos);
  
  Serial.print("EEPROM value: ");
  Serial.println(savedPos);
  
  // Check if value is within reasonable range
  if (savedPos < M3_MIN_VALID_POS || savedPos > M3_MAX_VALID_POS){
    Serial.println("WARNING: EEPROM data corrupted!");
    Serial.print("Invalid value detected: ");
    Serial.println(savedPos);
    Serial.println("Auto-resetting to 0...");
    
    long zeroValue = 0;
    EEPROM.put(M3_EEPROM_ADDR_POS, zeroValue);
    
    // Verify write
    long verifyValue;
    EEPROM.get(M3_EEPROM_ADDR_POS, verifyValue);
    
    if (verifyValue == 0) {
      Serial.println("SUCCESS: EEPROM reset to 0");
      Serial.println("");
      return true;
    } else {
      Serial.println("ERROR: EEPROM reset failed!");
      return false;
    }
  } else {
    Serial.println("EEPROM data valid.");
    Serial.println("");
    return true;
  }
}

bool checkEmergencyStop(){
  byte systemState;
  EEPROM.get(EEPROM_SYSTEM_STATE_ADDR, systemState);

  if (systemState == SYSTEM_RUNNING){
    Serial.println("!! EMERGENCEY STOP DETECTED !!");
    Serial.println("System was powered off during operation");
    Serial.println("Performing safety reset...");
    return true;
  }

  return false;
}

void setSystemState(byte state){
  EEPROM.put(EEPROM_SYSTEM_STATE_ADDR, state);
}

bool waitForRun(){
  clearSerialInput();
  Serial.println("Commands: [SPACE] = Start | 1-4 = Test Motor | [S] = Stop.");
  
  unsigned long startWaitTime = millis();
  const unsigned long WAIT_TIMEOUT = 60000;  // 60 second timeout

  while (true) {
    // Check for timeout
    if (millis() - startWaitTime > WAIT_TIMEOUT) {
      Serial.println("Timeout - system idle.");
      delay(5000);
      return false;
    }
    
    if (Serial.available() > 0){
      char input = Serial.read();

      if (input == ' ') {
        Serial.println("Starting full sequence...");
        return true;
      } else if (input == 'S' || input == 's'){
        Serial.println("System idle.");
        delay(5000);
        return false;
      } else if (input == 'P'){
        Serial.println("I, Pedro P. Ortiz II, designed this code to work the Middle Man.");
        delay(5000);
        return false;
      } else if (input == '1'){
        testM1();
        return false;
      } else if (input == '2'){
        //testM2();
        return false;
      } else if (input == '3'){
        testM3();
        return false;
      } else if (input == '4'){
        testM4();
        return false;
      }
    }
  }
}

void clearSerialInput(){
  while (Serial.available() > 0){
    Serial.read();
  }
}

bool allMotorsToSafePos(){
  bool allSuccess = true;
  
  Serial.println("Moving motors to safe positions...");
  
  // ===== M1 - PLUNGER (Future Implementation) ===== //
  // Placeholder: Will retract plunger to home position
  Serial.println("M1: Not yet implemented");
  
  // ===== M2 - PLATFORM (Future Implementation) ===== //
  // Placeholder: Will move platform to home position
  Serial.println("M2: Not yet implemented");
  
  // ===== M3 - CONVEYOR ===== //
  Serial.println("M3: Moving to safe position...");
  
  long savedPos = loadM3EncoderPos();
  m3Position = savedPos;
  
  unsigned long startTime = millis();

  if (getM3Position() > SAFE_POS_COUNTS){
    md.setM2Speed(SAFE_SPEED);
    
    while (getM3Position() > SAFE_POS_COUNTS){
      if (millis() - startTime > MOTOR_TIMEOUT_HOME){
        Serial.println("M3: Timeout!");
        md.setM2Speed(0);
        allSuccess = false;
        break;
      }
      if (checkMotorFaults()){
        allSuccess = false;
        break;
      }
      delay(10);
    }
    
  } else if (getM3Position() < SAFE_POS_COUNTS){
    md.setM2Speed(-SAFE_SPEED);
    
    while (getM3Position() < SAFE_POS_COUNTS){
      if (millis() - startTime > MOTOR_TIMEOUT_HOME){
        Serial.println("M3: Timeout!");
        md.setM2Speed(0);
        allSuccess = false;
        break;
      }
      if (checkMotorFaults()){
        allSuccess = false;
        break;
      }
      delay(10);
    }
  }

  md.setM2Speed(0);
  m3Position = SAFE_POS_COUNTS;
  savedM3EncoderPos();
  Serial.println("M3: Safe");
  
  // ===== M4 - BELT ===== //
  Serial.println("M4: Stopping...");
  md.setM1Speed(0);
  Serial.println("M4: Safe");
  
  if (allSuccess){
    Serial.println("All motors safe");
  } else {
    Serial.println("WARNING: Some motors may not be safe!");
  }
  
  return allSuccess;
}

void emergencyStop(){
  Serial.println("!! EMERGENCY STOP - RETURNING TO SAFE POSITION !!");

  // Immediately stop all motors
  setM1Direction(0);
  md.setM2Speed(0);
  md.setM1Speed(0);
  
  delay(500);

  // Move all motors to safe positions
  if (allMotorsToSafePos()){
    Serial.println("System secured successfully.");
  } else {
    Serial.println("WARNING: System may not be fully secured!");
  }

  Serial.println("");
  Serial.println("System halted. Press RESET to restart.");
  while(true);  // Halt until manual reset
}

void stopMotor(int motorNum){
  if (motorNum == 1) {
    setM1Direction(0);
    Serial.println("Motor 1 (Plunger) stopped.");

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

// M1 FUNCTIONS
void updateM1Encoder(){
  int currentA = digitalRead(m1PinA);
  int currentB = digitalRead(m1PinB);

  if (currentA != m1PrevA) {
    if (currentA == HIGH) {
      if (currentB == LOW) m1Position++;
      else m1Position--;
    } else {
      if (currentB == HIGH) m1Position++;
      else m1Position--;
    }
  }
  m1PrevA = currentA;
}

long getM1Position(){
  noInterrupts();
  long pos = m1Position;
  interrupts();
  return pos;
}

void setM1Direction(int dir){
  if (dir > 0){         // 1 = Extend
    digitalWrite(M1_INA, HIGH);
    digitalWrite(M1_INB, LOW);
  } else if (dir < 0){  // -1 = Retract
    digitalWrite(M1_INA, LOW);
    digitalWrite(M1_INB, HIGH);
  } else {              // 0 =Stop
    digitalWrite(M1_INA, LOW);
    digitalWrite(M1_INB, LOW);
  }
}

bool runM1Sequence(){
  Serial.println("");
  Serial.println("=== M1 PLUNGER SEQUENCE ===");
  
  // Reset position
  m1Position = 0;
  
  // STEP 1: Extend plunger
  Serial.print("Extending plunger (target: ");
  Serial.print(M1_EXTEND_COUNTS);
  Serial.println(" counts)...");
  
  setM1Direction(1);  // Extend
  
  unsigned long startTime = millis();
  while (abs(getM1Position()) < M1_EXTEND_COUNTS) {
    if (millis() - startTime > 10000) {
      Serial.println("ERROR: M1 extend timeout!");
      setM1Direction(0);
      return false;
    }
    
    // Print progress
    if (millis() % 200 == 0) {
      Serial.print("  Count: ");
      Serial.println(getM1Position());
    }
    delay(10);
  }
  
  setM1Direction(0);  // Stop
  Serial.print("Extension complete. Final count: ");
  Serial.println(getM1Position());
  Serial.println("");
  
  // STEP 2: Hold compression
  Serial.print("Holding compression for ");
  Serial.print(M1_HOLD_TIME / 1000);
  Serial.println(" seconds...");
  delay(M1_HOLD_TIME);
  Serial.println("Hold complete.");
  Serial.println("");
  
  // STEP 3: Retract plunger
  Serial.print("Retracting plunger (target: ");
  Serial.print(M1_EXTEND_COUNTS);
  Serial.println(" counts)...");
  
  m1Position = 0;  // Reset for return movement
  setM1Direction(-1);  // Retract
  
  startTime = millis();
  while (abs(getM1Position()) < M1_EXTEND_COUNTS) {
    if (millis() - startTime > 10000) {
      Serial.println("ERROR: M1 retract timeout!");
      setM1Direction(0);
      return false;
    }
    
    // Print progress
    if (millis() % 200 == 0) {
      Serial.print("  Count: ");
      Serial.println(getM1Position());
    }
    delay(10);
  }
  
  setM1Direction(0);  // Stop
  Serial.print("Retraction complete. Final count: ");
  Serial.println(getM1Position());
  Serial.println("");
  
  Serial.println("=== M1 PLUNGER SEQUENCE COMPLETE ===");
  return true;
}

void testM1(){
  Serial.println("Testing M1 (3 seconds)...");
  m1Position = 0;
  
  setM1Direction(1);
  unsigned long startTime = millis();
  
  while (millis() - startTime < 3000) {
    Serial.print("M1 Count: ");
    Serial.println(getM1Position());
    delay(500);
  }
  
  setM1Direction(0);
  Serial.println("M1 test complete");
}

// M2 FUNCTIONS
void updateM2Encoder(){}

// M3 FUNCTIONS
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

long getM3Position(){
  noInterrupts();
  long pos = m3Position;
  interrupts();
  return pos;
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

void testM3(){
  Serial.println("Testing M3 (3 seconds)...");
  m3Position = 0;
  
  md.setM2Speed(m3Speed);
  unsigned long startTime = millis();
  
  while (millis() - startTime < 3000) {
    Serial.print("M3 Count: ");
    Serial.println(getM3Position());
    delay(500);
  }
  
  md.setM2Speed(0);
  Serial.println("M3 test complete");
}

// M4 FUNCTIONS
void runM4Cont(int speed){
  md.setM1Speed(speed);
  Serial.print("Motor 4 running continuously at: ");
  Serial.println(speed);
}

void testM4(){
  Serial.println("Testing M4 (3 seconds)...");
  
  md.setM1Speed(m4Speed);
  delay(3000);
  md.setM1Speed(0);
  
  Serial.println("M4 test complete");
}

// bool m3GoToSafePos(){
//   long savedPos = loadM3EncoderPos();
//   m3Position = savedPos;

//   Serial.println("Moving M3 to safe position...");

//   unsigned long startTime = millis();
//   unsigned long lastPrintTime = 0;

//   if (getM3Position() > SAFE_POS_COUNTS){
//     md.setM2Speed(SAFE_SPEED);  // Negative speed to move down
    
//     while (getM3Position() > SAFE_POS_COUNTS){
//       if (millis() - startTime > MOTOR_TIMEOUT_HOME){
//         Serial.println("ERROR: Safe homing timeout!");
//         md.setM2Speed(0);
//         return false;
//       }

//       if (checkMotorFaults()){
//         return false;
//       }

//       if (millis() - lastPrintTime > 100){
//         Serial.print("Safe Homing... Encoder: ");
//         Serial.println(getM3Position());
//         lastPrintTime = millis();
//       }
//       delay(10);
//     }
    
//   } else if (getM3Position() < SAFE_POS_COUNTS){
//     md.setM2Speed(-SAFE_SPEED);  // Positive speed to move up
    
//     while (getM3Position() < SAFE_POS_COUNTS){
//       if (millis() - startTime > MOTOR_TIMEOUT_HOME){
//         Serial.println("ERROR: Safe homing timeout!");
//         md.setM2Speed(0);
//         return false;
//       }

//       if (checkMotorFaults()){
//         return false;
//       }

//       if (millis() - lastPrintTime > 100){
//         Serial.print("Safe Homing. Encoder Count: ");
//         Serial.println(getM3Position());
//         lastPrintTime = millis();
//       }
//       delay(10);
//     }
//   }

//   md.setM2Speed(0);
//   m3Position = SAFE_POS_COUNTS;
//   savedM3EncoderPos();
//   Serial.println("Ladies and Gentlemen, We Have Landed In The Safe Position");
//   return true;
// }

// ========================= SETUP ============================ //
void setup(){
  Serial.begin(115200);
  delay(500);
  
  Serial.println("AstroGlass Control System v0.25");

  bool emergencyDetected = checkEmergencyStop();
  
  if (!validateAndFixEEPROM()){
    Serial.println("FATAL: EEPROM error!");
    while(true);
  }
  
  initializeMotors();
  
  if (!allMotorsToSafePos()){
    Serial.println("ERROR: Failed to reach safe positions!");
    Serial.println("Manually position motors and press RESET");
    while(true);
  }

  if (emergencyDetected){
    Serial.println("Emergency Recovery Complete.");
    Serial.println("All motors reset to safe positions.");
  }
  
  Serial.println("System ready");
  delay(1000);
}

// ======================== MAIN LOOP ========================= //
void loop() {
  if (!waitForRun()) {
    delay(250);
    return;
  }

  // CLEAR ANY FAULTS FROM PREVIOUS RUN
  // Serial.println("Checking for residual faults...");
  // if (md.getM1Fault()) {
  //   Serial.println("M4 fault detected from previous run!");
  //   clearM4Fault();
  // }
  // if (md.getM2Fault()) {
  //   Serial.println("M3 fault detected from previous run!");
  //   md.setM2Speed(0);
  //   delay(100);
  // }

  Serial.println("Starting Sequence...");
  m3Position = 0;

  // Step One: M3 Lowers
  if (!moveM3ToPos(CONST_90_DEG, -m3Speed, "Lowering")){
    emergencyStop();
  }

  // Step Two: M4 Activates
  delay(3000);
  Serial.println("M4: Starting Belt");
  runM4Cont(m4Speed);

  // Step Three: Wait 10 seconds (Pour), then M1 activates
  delay(M1_ACTIVATION_DELAY);
  Serial.println("M1: Starting Plunger");

  if (!runM1Sequence()){
    Serial.println("M1 sequence failed");
    emergencyStop();
  }

  // Step Four: M2 Lowers
  Serial.println("M2: Lowering Platform (COMING SOON)");

  // Step Five: M4 still running
  unsigned long remainingTime = M4_TOTAL_RUN_TIME - M1_ACTIVATION_DELAY - 5000;
  Serial.print("M4: Running for ");
  Serial.print(remainingTime / 1000);
  Serial.println(" more seconds...");
  delay(remainingTime);

  // Step Six: Stop M4
  Serial.println("M4 Stopping");
  stopMotor(4);

  // Step Seven: M3 Rises
  if (!moveM3ToPos(CONST_90_DEG, m3Speed, "Raising")){
    emergencyStop();
  }

  // Reset Position
  m3Position = 0;
  EEPROM.put(M3_EEPROM_ADDR_POS, 0);
  setSystemState(SYSTEM_SAFE);

  Serial.println("== Sequence Complete ==");
  Serial.println("");

  delay(SEQUENCE_PAUSE);

  // M3 lowers down
  // if (!moveM3ToPos(CONST_90_DEG, -m3Speed, "Lowering")) {
  //   emergencyStop();
  // }

  // delay(DELAY_AFTER_DOWN);

  // // M4 runs continuously
  // Serial.println("Starting M4 belt...");
  
  // // Final check before running
  // if (md.getM1Fault()) {
  //   Serial.println("ERROR: M4 fault present before start!");
  //   clearM4Fault();
  //   delay(500);
  // }
  
  // runM4Cont(m4Speed);
  // delay(M4_RUN_TIME);
  // stopMotor(4);
  
  // // Check for fault after run
  // if (md.getM1Fault()) {
  //   Serial.println("WARNING: M4 fault occurred during run!");
  //   Serial.println("Possible causes:");
  //   Serial.println("- Motor drawing too much current");
  //   Serial.println("- Motor stalled or jammed");
  //   Serial.println("- Loose wiring");
  // }
  
  // delay(DELAY_AFTER_M4);

  // // M3 raises back up
  // if (!moveM3ToPos(CONST_90_DEG, m3Speed, "Raising")) {
  //   emergencyStop();
  // }

  // delay(DELAY_AFTER_UP);

  // // Ensure M4 is stopped
  // stopMotor(4);

  // // ===== STEP 4: RUN M1 PLUNGER SEQUENCE ===== //
  // Serial.println("");
  // Serial.println("=== STEP 4: M1 PLUNGER ===");
  
  // if (!runM1Sequence()) {
  //   Serial.println("ERROR: M1 plunger sequence failed!");
  //   emergencyStop();
  // }
  
  // Serial.println("M1 plunger sequence complete.");
  // Serial.println("");

  // // CLEAR FAULTS AT END OF SEQUENCE
  // delay(500);
  // if (md.getM1Fault()) {
  //   Serial.println("Clearing M4 fault at end of sequence...");
  //   clearM4Fault();
  // }

  // // Reset position and save to EEPROM
  // m3Position = 0;
  // EEPROM.put(M3_EEPROM_ADDR_POS, 0);

  // Serial.println("=== SEQUENCE COMPLETE ===");
  // Serial.println("");
  
  // delay(SEQUENCE_PAUSE);
}
