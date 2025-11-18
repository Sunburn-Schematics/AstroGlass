// ============================================================= //
// PROJECT:   AstroGlass, The Middle Man, Control System
// PLATFORM:  Arduino MEGA 2560
// DRIVER:    x2 DualVNH5019 Motor Shield
// MOTOR:     x4 Maverick 12V DC Gear Motor w/Encoder (61:1)
// AUTHOR:    Pedro Ortiz
// VERSION:   v1.3.1
// ============================================================= //

#include <EEPROM.h>
#include <Arduino.h>
#include <DualVNH5019MotorShield.h>

// =================== SPEED REFERENCE CHART ================== //
// M1 and M2 Speed Chart (PWM)
/*
    0: Stopped            (0%)
   64: Slow               (25%)
  128: Medium             (50%)
  192: High               (75%)
  255: Full               (100%)
*/

// M3 and M4 Speed Chart (Shield)
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
const int M1_PWM = 12;                  // M1 PWM Speed Control

// M2 Platform Motor
const int M2_INA = 7;                   // M2 Direction A
const int M2_INB = 8;                   // M2 Direction B
const int M2_PWM = 11;                  // M2 PWM Speed Control

// ================= EMERGENCY STOP BUTTON ====================
const int EMERGENCY_STOP_PIN = 21;      // Hardware emergency stop

// ======================= ENCODER PINS ======================= //
// M1 Plunger Encoder
const int m1PinA     = 18;              // Encoder Channel A (interrupt)
const int m1PinB     = 19;              // Encoder Channel B (interrupt)
volatile int m1PrevA = LOW;

// M2 Platform Encoder
const int m2PinA     = 20;              // Encoder Channel A (interrupt)
const int m2PinB     = 22;              // Encoder Channel B (non-interrupt)
volatile int m2PrevA = LOW;

// M3 Conveyor Encoder
const int m3PinA     = 2;               // Encoder Channel A (interrupt)
const int m3PinB     = 3;               // Encoder Channel B (interrupt)
volatile int m3PrevA = LOW;

// ============== MOTOR-SPECIFIC PARAMETERS =================== //
// M1 Plunger Parameters
const long M1_EXTEND_COUNTS = 50;      // Extension distance in encoder counts
const long M1_HOLD_TIME     = 1000;     // Compression hold time (milliseconds)
const int m1Speed           = 64;       // PWM speed (0-255)

// M2 Platform Parameters
const long M2_LOWER_COUNTS = 50;        // Lowering distance in encoder counts
const int m2Speed          = 64;        // PWM speed (0-255)

// M3 Conveyor Parameters
const long SAFE_POS_COUNTS   = 0;       // Home/safe position
const long CONST_90_DEG      = 214;     // 90-degree movement (1/4 revolution)
const long M3_MAX_VALID_POS  = 500;     // Maximum valid EEPROM position
const long M3_MIN_VALID_POS  = -500;    // Minimum valid EEPROM position
const int m3Speed            = 200;
const int SAFE_SPEED         = 150;     // Speed for returning to safe position

// M4 Belt Parameters
const int m4Speed = 200;

// Extra Parameters
const long countsPerRev = 854;          // Maverick: 7 PPR × 2 edges × 61:1 gear

// ===================== TIMING PARAMETERS ==================== //
const unsigned long MOTOR_TIMEOUT_MOVE  = 15000;   // M3 movement timeout (15s)
const unsigned long MOTOR_TIMEOUT_HOME  = 15000;   // M3 homing timeout (15s)
const unsigned long M1_ACTIVATION_DELAY = 10000;   // Unused legacy parameter
const unsigned long M4_TOTAL_RUN_TIME   = 5000;    // M4 total run duration (5s)
const unsigned long SEQUENCE_PAUSE      = 500;     // Pause between sequences (0.5s)

// ================ EEPROM CONFIGURATION ====================== //
const int M1_EEPROM_ADDR_POS       = 0;      // EEPROM storage address
const int M2_EEPROM_ADDR_POS       = 8;      // EEPROM storage address
const int M3_EEPROM_ADDR_POS       = 16;      // EEPROM storage address
const int EEPROM_SYSTEM_STATE_ADDR = 24;      // System state flag address
const byte SYSTEM_RUNNING          = 0xAA;   // Magic byte: system active
const byte SYSTEM_SAFE             = 0x55;   // Magic byte: system safe

// ==================== MOTOR POSITIONS ======================= //
volatile long m1Position = 0;       // M1 current encoder position
volatile long m2Position = 0;       // M2 current encoder position
volatile long m3Position = 0;       // M3 current encoder position

// =================== EMERGENCY STOP FLAG ==================== //
volatile bool emergencyStopTriggered = false;

// ===== MOTOR SHIELD OBJECT =====
DualVNH5019MotorShield md;

// =================== FUNCTION DECLARATIONS ================== //
// System Functions
void initializeMotors();
void clearMotorFaults();
bool checkMotorFaults();
bool validateAndFixEEPROM();
bool waitForRun();
void clearSerialInput();
void emergencyStop();
void emergencyStopISR();
void stopMotor(int motorNum);
void stopAllMotors();
bool allMotorsToSafePos();
bool checkEmergencyStop();
void setSystemState(byte state);
void printProgressBar(int step, int totalSteps, const char* stepName);

// M1 Functions
void updateM1Encoder();
long getM1Position();
void savedM1EncoderPos();
long loadM1EncoderPos();
void setM1Direction(int dir);
bool runM1Sequence();
void testM1();

// M2 Functions
void updateM2Encoder();
long getM2Position();
void savedM2EncoderPos();
long loadM2EncoderPos();
void setM2Direction(int dir);
bool runM2Sequence();
void testM2();

// M3 Functions
void updateM3Encoder();
long getM3Position();
void savedM3EncoderPos();
long loadM3EncoderPos();
bool moveM3ToPos(long targetCount, int speed, const char* direction);
void testM3();

// M4 Functions
void runM4Cont(int speed);
void testM4();

// =================== SYSTEM FUNCTIONS ======================= //
// Initialize all motors and encoders
void initializeMotors(){
  // Initialize Shield 1 (M3 & M4)
  md.init();
  
  // Initialize M1 control pins
  pinMode(M1_INA, OUTPUT);
  pinMode(M1_INB, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  digitalWrite(M1_INA, LOW);
  digitalWrite(M1_INB, LOW);
  analogWrite(M1_PWM, 0);
  
  // Initialize M1 encoder with both interrupt pins
  pinMode(m1PinA, INPUT_PULLUP);
  pinMode(m1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m1PinA), updateM1Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m1PinB), updateM1Encoder, CHANGE);

  // Initialize M2 control pins
  pinMode(M2_INA, OUTPUT);
  pinMode(M2_INB, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  digitalWrite(M2_INA, LOW);
  digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);
  
  // Initialize M2 encoder (pin 22 is not an interrupt pin)
  pinMode(m2PinA, INPUT_PULLUP);
  pinMode(m2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m2PinA), updateM2Encoder, CHANGE);
  
  // Initialize M3 encoder with both interrupt pins
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
  
  // Initialize emergency stop button
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStopISR, FALLING);
}

// Clear motor faults on startup
void clearMotorFaults(){
  Serial.println("Clearing motor faults...");
  
  // Stop all shield motors
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(100);
  
  // Reinitialize motor shield
  md.init();
  delay(500);
  
  // Re-attach M3 encoder interrupts
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
  
  Serial.println("Motor faults cleared");
}

// Emergency stop interrupt service routine
void emergencyStopISR(){
  emergencyStopTriggered = true;
}

// Check for motor faults on the shield
bool checkMotorFaults(){
  if (md.getM1Fault()){
    Serial.println("ERROR: M4 fault detected!");
    return true;
  }

  if (md.getM2Fault()){
    Serial.println("ERROR: M3 fault detected!");
    return true;
  }
  return false;
}

// Validate EEPROM data and reset if corrupted
bool validateAndFixEEPROM(){
  Serial.println("Validating EEPROM data...");
  bool allValid = true;
  
  // Check M1 position
  long savedM1Pos;
  EEPROM.get(M1_EEPROM_ADDR_POS, savedM1Pos);
  Serial.print("M1 EEPROM value: ");
  Serial.println(savedM1Pos);
  
  if (savedM1Pos < -500 || savedM1Pos > 500){
    Serial.println("WARNING: M1 EEPROM corrupted - resetting to 0");
    long zeroValue = 0;
    EEPROM.put(M1_EEPROM_ADDR_POS, zeroValue);
    allValid = false;
  }
  
  // Check M2 position
  long savedM2Pos;
  EEPROM.get(M2_EEPROM_ADDR_POS, savedM2Pos);
  Serial.print("M2 EEPROM value: ");
  Serial.println(savedM2Pos);
  
  if (savedM2Pos < -500 || savedM2Pos > 500){
    Serial.println("WARNING: M2 EEPROM corrupted - resetting to 0");
    long zeroValue = 0;
    EEPROM.put(M2_EEPROM_ADDR_POS, zeroValue);
    allValid = false;
  }
  
  // Check M3 position
  long savedM3Pos;
  EEPROM.get(M3_EEPROM_ADDR_POS, savedM3Pos);
  Serial.print("M3 EEPROM value: ");
  Serial.println(savedM3Pos);
  
  if (savedM3Pos < M3_MIN_VALID_POS || savedM3Pos > M3_MAX_VALID_POS){
    Serial.println("WARNING: M3 EEPROM corrupted - resetting to 0");
    long zeroValue = 0;
    EEPROM.put(M3_EEPROM_ADDR_POS, zeroValue);
    allValid = false;
  }
  
  if (allValid){
    Serial.println("All EEPROM data valid");
  } else {
    Serial.println("EEPROM corrupted - all positions reset to 0");
  }
  
  Serial.println("");
  return true;
}

// Check if system was powered off during operation
bool checkEmergencyStop(){
  byte systemState;
  EEPROM.get(EEPROM_SYSTEM_STATE_ADDR, systemState);

  if (systemState == SYSTEM_RUNNING){
    Serial.println("!! EMERGENCY STOP DETECTED !!");
    Serial.println("System was powered off during operation");
    return true;
  }

  return false;
}

// Set system state in EEPROM
void setSystemState(byte state){
  EEPROM.put(EEPROM_SYSTEM_STATE_ADDR, state);
}

// Wait for user input to start sequence or test motors
bool waitForRun(){
  clearSerialInput();
  Serial.println("Astro Motor Commands: [SPACE] = Start | [1] - [4] = Test Motor | [S] = Stop");
  
  unsigned long startWaitTime = millis();
  const unsigned long WAIT_TIMEOUT = 120000;  // 120 second timeout

  while (true){
    // Check for timeout
    if (millis() - startWaitTime > WAIT_TIMEOUT){
      Serial.println("Timeout - system idle");
      delay(5000);
      return false;
    }
    
    // Check for serial input
    if (Serial.available() > 0){
      char input = Serial.read();

      if (input == ' ') {
        Serial.println("Starting full sequence...");
        return true;
      } else if (input == 'S' || input == 's'){
        Serial.println("System Idle");
        delay(5000);
        return false;
      } else if (input == 'P'){
        Serial.println("Pedro P. Ortiz II, designed this code to work the Middle Man.");
        delay(5000);
        return false;
      } else if (input == '1'){
        testM1();
        return false;
      } else if (input == '2'){
        testM2();
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

// Clear serial input buffer
void clearSerialInput(){
  while (Serial.available() > 0){
    Serial.read();
  }
}

// Stop all motors immediately
void stopAllMotors(){
  setM1Direction(0);      // Stop M1 plunger
  setM2Direction(0);      // Stop M2 platform
  md.setM2Speed(0);       // Stop M3 conveyor
  md.setM1Speed(0);       // Stop M4 belt
}

// Move all motors to safe positions
bool allMotorsToSafePos(){
  bool allSuccess = true;
  
  Serial.println("Moving motors to safe positions...");
  
  // M1 - Plunger: Return to home position (fully retracted)
  Serial.println("M1: Moving to safe position...");
  
  long savedM1Pos = loadM1EncoderPos();
  m1Position = savedM1Pos;
  
  unsigned long startTime = millis();
  const long M1_HOME_POSITION = 0;  // Fully retracted
  
  // If position is extended (positive), retract
  if (getM1Position() > M1_HOME_POSITION){
    setM1Direction(-1);  // Retract
    
    while (getM1Position() > M1_HOME_POSITION){
      if (millis() - startTime > 10000){  // 10 second timeout
        Serial.println("M1: Timeout!");
        setM1Direction(0);
        allSuccess = false;
        break;
      }
      delay(10);
    }
    
  // If position is somehow negative, extend to home
  } else if (getM1Position() < M1_HOME_POSITION){
    setM1Direction(1);  // Extend
    
    while (getM1Position() < M1_HOME_POSITION){
      if (millis() - startTime > 10000){
        Serial.println("M1: Timeout!");
        setM1Direction(0);
        allSuccess = false;
        break;
      }
      delay(10);
    }
  }
  
  // Stop M1 and save position
  setM1Direction(0);
  m1Position = M1_HOME_POSITION;
  savedM1EncoderPos();
  Serial.println("M1: Safe");
  
  // M2 - Platform: Return to home position (fully raised)
  Serial.println("M2: Moving to safe position...");
  
  long savedM2Pos = loadM2EncoderPos();
  m2Position = savedM2Pos;
  
  startTime = millis();
  const long M2_HOME_POSITION = 0;  // Fully raised
  
  // If position is lowered (positive), raise
  if (getM2Position() > M2_HOME_POSITION){
    setM2Direction(-1);  // Raise
    
    while (getM2Position() > M2_HOME_POSITION){
      if (millis() - startTime > 10000){  // 10 second timeout
        Serial.println("M2: Timeout!");
        setM2Direction(0);
        allSuccess = false;
        break;
      }
      delay(10);
    }
    
  // If position is somehow negative, lower to home
  } else if (getM2Position() < M2_HOME_POSITION){
    setM2Direction(1);  // Lower
    
    while (getM2Position() < M2_HOME_POSITION){
      if (millis() - startTime > 10000){
        Serial.println("M2: Timeout!");
        setM2Direction(0);
        allSuccess = false;
        break;
      }
      delay(10);
    }
  }
  
  // Stop M2 and save position
  setM2Direction(0);
  m2Position = M2_HOME_POSITION;
  savedM2EncoderPos();
  Serial.println("M2: Safe");
  
  // M3 - Conveyor: Return to home position
  Serial.println("M3: Moving to safe position...");
  
  long savedPos = loadM3EncoderPos();
  m3Position = savedPos;
  
  startTime = millis();

  // If position is above safe position, lower
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
    
  // If position is below safe position, raise
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

  // Stop M3 and save position
  md.setM2Speed(0);
  m3Position = SAFE_POS_COUNTS;
  savedM3EncoderPos();
  Serial.println("M3: Safe");
  
  // M4 - Belt: Stop in place
  Serial.println("M4: Stopping belt");
  md.setM1Speed(0);
  Serial.println("M4: Safe");
  
  if (allSuccess){
    Serial.println("All motors safe");
  } else {
    Serial.println("WARNING: Some motors may not be safe!");
  }
  
  return allSuccess;
}

// Emergency stop: halt all motors and return to safe positions
void emergencyStop(){
  Serial.println("!! EMERGENCY STOP !!");

  // Immediately stop all motors
  stopAllMotors();
  delay(500);

  // Move all motors to safe positions
  if (allMotorsToSafePos()){
    Serial.println("System secured successfully");
  } else {
    Serial.println("WARNING: System may not be fully secured!");
  }

  // Halt system until manual reset
  Serial.println("");
  Serial.println("System halted. Press RESET to restart.");
  while(true);  // Infinite loop
}

// Stop individual motor by number
void stopMotor(int motorNum){
  if (motorNum == 1) {
    setM1Direction(0);
    savedM1EncoderPos();
    Serial.println("M1 stopped");
  } else if (motorNum == 2){
    setM2Direction(0);
    savedM2EncoderPos();
    Serial.println("M2 stopped");
  } else if (motorNum == 3){
    md.setM2Speed(0);
    savedM3EncoderPos();
    Serial.println("M3 stopped");
  } else if (motorNum == 4){
    md.setM1Speed(0);
    Serial.println("M4 stopped");
  } else {
    Serial.println("Invalid motor number");
  }
}

// Display progress bar in serial monitor
void printProgressBar(int step, int totalSteps, const char* stepName){
  int barWidth = 30; // Width of the bar
  float progress = (float)step / totalSteps;
  int pos = barWidth * progress;

  Serial.print("[");
  for (int i = 0; i < barWidth; i++){
    if (i < pos) Serial.print("=");
    else if (i == pos) Serial.print(">");
    else Serial.print(" ");
  }
  Serial.print("] ");

  int percent = progress * 100;
  Serial.print(percent);
  Serial.print("% - ");
  Serial.println(stepName);
}

// =================== M1 PLUNGER FUNCTIONS =================== //
// M1 encoder interrupt service routine (full quadrature)
void updateM1Encoder(){
  int currentA = digitalRead(m1PinA);
  int currentB = digitalRead(m1PinB);

  // Detect edge on channel A
  if (currentA != m1PrevA){
    // Determine direction based on channel B state
    if (currentA == HIGH){
      if (currentB == LOW) m1Position++;
      else m1Position--;
    } else {
      if (currentB == HIGH) m1Position++;
      else m1Position--;
    }
    m1PrevA = currentA;
  }
}

// Get M1 encoder position (thread-safe)
long getM1Position(){
  noInterrupts();
  long pos = m1Position;
  interrupts();
  return pos;
}

// Save M1 encoder position to EEPROM
void savedM1EncoderPos(){
  noInterrupts();
  long tempPos = m1Position;
  interrupts();
  EEPROM.put(M1_EEPROM_ADDR_POS, tempPos);
}

// Load M1 encoder position from EEPROM
long loadM1EncoderPos(){
  long savedPos;
  EEPROM.get(M1_EEPROM_ADDR_POS, savedPos);
  return savedPos;
}

// Set M1 direction and speed
void setM1Direction(int dir){
  if (dir > 0){         // Extend
    digitalWrite(M1_INA, HIGH);
    digitalWrite(M1_INB, LOW);
    analogWrite(M1_PWM, m1Speed);
  } else if (dir < 0){  // Retract
    digitalWrite(M1_INA, LOW);
    digitalWrite(M1_INB, HIGH);
    analogWrite(M1_PWM, m1Speed);
  } else {              // Stop
    digitalWrite(M1_INA, LOW);
    digitalWrite(M1_INB, LOW);
    analogWrite(M1_PWM, 0);
  }
}

// Run M1 plunger sequence: extend, hold, retract
bool runM1Sequence(){
  Serial.println("");
  Serial.println("=== M1 PLUNGER SEQUENCE ===");
  
  m1Position = 0;  // Reset encoder position
  
  // STEP 1: Extend plunger
  Serial.println("Extending plunger until resistance...");
  setM1Direction(1);
  
  long lastPosition = 0;
  unsigned long lastMovementTime = millis();
  const unsigned long STALL_TIMEOUT = 200;
  const long MIN_MOVEMENT = 2;

  unsigned long startTime = millis();
  while (true){
    if (emergencyStopTriggered) emergencyStop();

    // Check for overall timeout (safety caution)
    if (millis() - startTime > 10000){
      Serial.println("ERROR: M1 extend timeout!");
      setM1Direction(0);
      return false;
    }

    long currentPosition = getM1Position();

    // Check if motor is still moving
    if (abs(currentPosition - lastPosition) > MIN_MOVEMENT){
      lastPosition = currentPosition;
      lastMovementTime = millis();
    }

    // If no movement for STALL_TIMEOUT, we got resistance
    if (millis() - lastMovementTime > STALL_TIMEOUT){
      Serial.println("Resistance detected - Plunger stalled");
      Serial.print("Position at stall: ");
      Serial.println(currentPosition);
      break;
    }
    
    delay(10);
  }
    // Step 2: Hold compression with force
    Serial.print("Compressing for ");
    Serial.print(M1_HOLD_TIME / 1000);
    Serial.println(" seconds...");
    // Motor stays powered, applying force
    delay(M1_HOLD_TIME);

    setM1Direction(0);
    Serial.println("Compression Complete.");

    // Step 3: Retract plunger to start position
    Serial.println("Retracting plunger...");
    long extendedPosition = getM1Position();
    setM1Direction(-1);

    startTime = millis();
    while (abs(getM1Position()) > 10){  // Retract to ~0 (with some tolerance)
      if (emergencyStopTriggered) emergencyStop();

      if (millis() - startTime > 10000){
        Serial.println("ERROR: M1 retract timeout!");
        setM1Direction(0);
        return false;
      }
      delay(10);
    }

    setM1Direction(0);
    savedM1EncoderPos();
    Serial.println("=== M1 SEQUENCE COMPLETE ===");
    return true;
}

// Test M1 motor - Full rotation forward and reverse
void testM1(){
  Serial.println("Testing M1 - Full Rotation Test");
  Serial.println("=================================");
  m1Position = 0;
  
  // FORWARD: Full rotation
  Serial.println("FORWARD: One full rotation...");
  setM1Direction(1);
  
  unsigned long startTime = millis();
  while (abs(getM1Position()) < countsPerRev){
    if (millis() % 500 == 0) {  // Print every 500ms
      Serial.print("  M1 Count: ");
      Serial.println(getM1Position());
    }
    delay(10);
  }
  
  setM1Direction(0);
  Serial.print("Forward complete. Final count: ");
  Serial.println(getM1Position());
  Serial.println("");
  
  delay(1000);  // Pause between directions
  
  // REVERSE: Full rotation back
  Serial.println("REVERSE: One full rotation...");
  m1Position = 0;  // Reset counter
  setM1Direction(-1);
  
  startTime = millis();
  while (abs(getM1Position()) < countsPerRev){
    if (millis() % 500 == 0){  // Print every 500ms
      Serial.print("  M1 Count: ");
      Serial.println(getM1Position());
    }
    delay(10);
  }
  
  setM1Direction(0);
  Serial.print("Reverse complete. Final count: ");
  Serial.println(getM1Position());
  Serial.println("");
  Serial.println("=== M1 TEST COMPLETE ===");
}

// =================== M2 PLATFORM FUNCTIONS ================== //
// M2 encoder interrupt service routine (partial quadrature)
void updateM2Encoder(){
  int currentA = digitalRead(m2PinA);
  int currentB = digitalRead(m2PinB);

  // Detect edge on channel A
  if (currentA != m2PrevA){
    // Determine direction based on channel B state
    if (currentA == HIGH){
      if (currentB == LOW) m2Position++;
      else m2Position--;
    } else {
      if (currentB == HIGH) m2Position++;
      else m2Position--;
    }
    m2PrevA = currentA;
  }
}

// Get M2 encoder position (thread-safe)
long getM2Position(){
  noInterrupts();
  long pos = m2Position;
  interrupts();
  return pos;
}

// Save M2 encoder position to EEPROM
void savedM2EncoderPos(){
  noInterrupts();
  long tempPos = m2Position;
  interrupts();
  EEPROM.put(M2_EEPROM_ADDR_POS, tempPos);
}

// Load M2 encoder position from EEPROM
long loadM2EncoderPos(){
  long savedPos;
  EEPROM.get(M2_EEPROM_ADDR_POS, savedPos);
  return savedPos;
}

// Set M2 direction and speed
void setM2Direction(int dir){
  if (dir > 0){         // Lower platform
    digitalWrite(M2_INA, HIGH);
    digitalWrite(M2_INB, LOW);
    analogWrite(M2_PWM, m2Speed);
  } else if (dir < 0){  // Raise platform
    digitalWrite(M2_INA, LOW);
    digitalWrite(M2_INB, HIGH);
    analogWrite(M2_PWM, m2Speed);
  } else {              // Stop
    digitalWrite(M2_INA, LOW);
    digitalWrite(M2_INB, LOW);
    analogWrite(M2_PWM, 0);
  }
}

// Run M2 platform sequence: lower, trigger M4, wait, raise
bool runM2Sequence(){
  Serial.println("");
  Serial.println("=== M2 PLATFORM SEQUENCE ===");
  
  m2Position = 0;  // Reset encoder position
  
  // STEP 1: Lower platform to halfway point (triggers M4)
  Serial.println("M2: Lowering to halfway point...");
  setM2Direction(1);

  long m2HalfwayPoint = M2_LOWER_COUNTS / 2;  // Calculate halfway dynamically
  Serial.print("Halfway point: ");
  Serial.println(m2HalfwayPoint);

  unsigned long startTime = millis();
  while (abs(getM2Position()) < m2HalfwayPoint){
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > 10000){
      Serial.println("ERROR: M2 halfway timeout!");
      setM2Direction(0);
      return false;
    }
    delay(10);
  }
  
  // STEP 2: Start M4 belt at halfway point
  Serial.println("M2: Reached halfway - Starting M4 belt");
  
  // Always clear faults before M4
  Serial.println("Clearing faults before M4...");
  md.setM1Speed(0);
  delay(100);
  md.init();
  delay(500);
  
  // Re-attach M3 encoder after reinit
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
  delay(500);
  
  runM4Cont(m4Speed);
  unsigned long m4StartTime = millis();
  
  // STEP 3: Continue lowering to full position
  Serial.println("M2: Continuing to full lower position...");
  
  startTime = millis();
  while (abs(getM2Position()) < M2_LOWER_COUNTS){
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > 10000){
      Serial.println("ERROR: M2 full lower timeout!");
      setM2Direction(0);
      stopMotor(4);
      return false;
    }
    delay(10);
  }
  
  setM2Direction(0);
  Serial.print("M2: Fully lowered. Final count: ");
  Serial.println(getM2Position());
  
  // STEP 4: Wait 1 second at bottom (M4 still running)
  Serial.println("M2: Waiting 1 second...");
  delay(1000);
  
  // STEP 5: Raise platform back to start position
  Serial.println("M2: Raising back to start position...");
  setM2Direction(-1);
  
  startTime = millis();
  unsigned long lastPrintTime = 0;
  
  while (abs(getM2Position()) > 5){  // Allow 5-count tolerance
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > 10000){
      Serial.println("ERROR: M2 raise timeout!");
      setM2Direction(0);
      stopMotor(4);
      return false;
    }
    
    // Position monitoring removed for cleaner output
    lastPrintTime = millis();
    
    delay(10);
  }
  
  setM2Direction(0);
  savedM2EncoderPos();
  Serial.println("M2: Back at start position");
  
  // STEP 6: Keep M4 running until 5 seconds total
  unsigned long m4ElapsedTime = millis() - m4StartTime;
  if (m4ElapsedTime < M4_TOTAL_RUN_TIME){
    unsigned long remainingTime = M4_TOTAL_RUN_TIME - m4ElapsedTime;
    Serial.print("M4: Running for ");
    Serial.print(remainingTime / 1000);
    Serial.println(" more seconds...");
    delay(remainingTime);
  }
  
  // STEP 7: Stop M4
  Serial.println("M4: Stopping");
  stopMotor(4);
  
  Serial.println("=== M2 PLATFORM SEQUENCE COMPLETE ===");
  return true;
}

// Test M2 motor - Full rotation forward and reverse
void testM2(){
  Serial.println("Testing M2 - Full Rotation Test");
  Serial.println("=================================");
  m2Position = 0;
  
  // FORWARD: Full rotation (lowering)
  Serial.println("LOWERING: One full rotation...");
  setM2Direction(1);
  
  unsigned long startTime = millis();
  while (abs(getM2Position()) < countsPerRev){
    if (millis() % 500 == 0) {  // Print every 500ms
      Serial.print("  M2 Count: ");
      Serial.println(getM2Position());
    }
    delay(10);
  }
  
  setM2Direction(0);
  Serial.print("Lowering complete. Final count: ");
  Serial.println(getM2Position());
  Serial.println("");
  
  delay(1000);  // Pause between directions
  
  // REVERSE: Full rotation back (raising)
  Serial.println("RAISING: One full rotation...");
  m2Position = 0;  // Reset counter
  setM2Direction(-1);
  
  startTime = millis();
  while (abs(getM2Position()) < countsPerRev){
    if (millis() % 500 == 0){  // Print every 500ms
      Serial.print("  M2 Count: ");
      Serial.println(getM2Position());
    }
    delay(10);
  }
  
  setM2Direction(0);
  Serial.print("Raising complete. Final count: ");
  Serial.println(getM2Position());
  Serial.println("");
  Serial.println("=== M2 TEST COMPLETE ===");
}

// =================== M3 CONVEYOR FUNCTIONS ==================//
// M3 encoder interrupt service routine (full quadrature)
void updateM3Encoder(){
  int currentA = digitalRead(m3PinA);
  int currentB = digitalRead(m3PinB);

  // Detect edge on channel A
  if (currentA != m3PrevA){
    // Determine direction based on channel B state
    if (currentA == HIGH){
      if (currentB == LOW) m3Position++;
      else m3Position--;
    } else {
      if (currentB == HIGH) m3Position++;
      else m3Position--;
    }
    m3PrevA = currentA;
  }
}

// Get M3 encoder position (thread-safe)
long getM3Position(){
  noInterrupts();
  long pos = m3Position;
  interrupts();
  return pos;
}

// Save M3 encoder position to EEPROM
void savedM3EncoderPos(){
  noInterrupts();
  long tempPos = m3Position;
  interrupts();
  EEPROM.put(M3_EEPROM_ADDR_POS, tempPos);
}

// Load M3 encoder position from EEPROM
long loadM3EncoderPos(){
  long savedPos;
  EEPROM.get(M3_EEPROM_ADDR_POS, savedPos);
  return savedPos;
}

// Move M3 to target position at specified speed
bool moveM3ToPos(long targetCount, int speed, const char* direction){
  // Clear any faults before starting
  Serial.println("Preparing M3...");
  md.setM2Speed(0);
  delay(100);
  
  if (md.getM2Fault()){
    Serial.println("Clearing M3 fault...");
    md.init();
    delay(500);
    
    // Re-attach M3 encoder after reinit
    pinMode(m3PinA, INPUT_PULLUP);
    pinMode(m3PinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
    delay(500);
  }
  
  m3Position = 0;  // Reset for relative movement
  
  Serial.print("M3 ");
  Serial.print(direction);
  Serial.println("...");

  md.setM2Speed(speed);
  
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;

  while (abs(getM3Position()) < targetCount){
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > MOTOR_TIMEOUT_MOVE){
      Serial.print("ERROR: M3 ");
      Serial.print(direction);
      Serial.println(" timeout!");
      stopMotor(3);
      return false;
    }

    if (checkMotorFaults()){
      Serial.println("Fault during M3 movement!");
      stopMotor(3);
      return false;
    }

    // Position monitoring removed for cleaner output
    lastPrintTime = millis();
    delay(10);
  }

  stopMotor(3);
  Serial.print("M3 ");
  Serial.print(direction);
  Serial.println(" complete");
  return true;
}

// Test M3 motor - Full rotation forward and reverse
void testM3(){
  Serial.println("Testing M3 - Full Rotation Test");
  Serial.println("=================================");
  
  // Clear faults before testing
  Serial.println("Clearing faults...");
  md.setM2Speed(0);
  delay(100);
  
  if (md.getM2Fault() || md.getM1Fault()){
    Serial.println("Fault detected - reinitializing...");
    md.init();
    delay(500);
    
    // Re-attach M3 encoder
    pinMode(m3PinA, INPUT_PULLUP);
    pinMode(m3PinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
    delay(500);
  }
  
  m3Position = 0;
  
  // FORWARD: Full rotation
  Serial.println("FORWARD: One full rotation...");
  md.setM2Speed(m3Speed);
  
  unsigned long startTime = millis();
  while (abs(getM3Position()) < countsPerRev){
    if (millis() % 500 == 0){  // Print every 500ms
      Serial.print("  M3 Count: ");
      Serial.println(getM3Position());
    }
    delay(10);
  }
  
  md.setM2Speed(0);
  Serial.print("Forward complete. Final count: ");
  Serial.println(getM3Position());
  Serial.println("");
  
  delay(1000);  // Pause between directions
  
  // REVERSE: Full rotation back
  Serial.println("REVERSE: One full rotation...");
  m3Position = 0;  // Reset counter
  md.setM2Speed(-m3Speed);
  
  startTime = millis();
  while (abs(getM3Position()) < countsPerRev){
    if (millis() % 500 == 0){  // Print every 500ms
      Serial.print("  M3 Count: ");
      Serial.println(getM3Position());
    }
    delay(10);
  }
  
  md.setM2Speed(0);
  Serial.print("Reverse complete. Final count: ");
  Serial.println(getM3Position());
  Serial.println("");
  Serial.println("=== M3 TEST COMPLETE ===");
}

// =================== M4 BELT FUNCTIONS ====================== //
// Run M4 belt continuously at specified speed
void runM4Cont(int speed){
  md.setM1Speed(speed);
  Serial.print("M4 running at speed: ");
  Serial.println(speed);
}

// Test M4 motor - Forward and reverse (USING M2 OUTPUT FOR TESTING)
void testM4(){
  Serial.println("Testing M4 - Forward/Reverse Test");
  Serial.println("==================================");
  
  // Always clear faults before testing
  Serial.println("Clearing faults...");
  md.setM1Speed(0);
  delay(100);
  
  // Always reinitialize to ensure clean state
  md.init();
  delay(500);
  
  // Re-attach M3 encoder (since md.init() clears it)
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
  delay(500);
  
  // FORWARD: 3 seconds
  Serial.println("FORWARD: Running for 3 seconds...");
  md.setM1Speed(m4Speed);
  delay(3000);
  md.setM1Speed(0);
  Serial.println("Forward complete.");
  Serial.println("");
  
  delay(1000);  // Pause between directions
  
  // REVERSE: 3 seconds
  Serial.println("REVERSE: Running for 3 seconds...");
  md.setM1Speed(-m4Speed);
  delay(3000);
  md.setM1Speed(0);
  Serial.println("Reverse complete.");
  Serial.println("");
  
  Serial.println("=== M4 TEST COMPLETE ===");
}

// ========================= SETUP ============================ //
void setup(){
  Serial.begin(115200);
  delay(500);
  
  Serial.println("AstroGlass Control System v1.0.7");

  // Check if system was stopped during operation
  bool emergencyDetected = checkEmergencyStop();
  
  // Validate EEPROM data
  if (!validateAndFixEEPROM()){
    Serial.println("FATAL: EEPROM error!");
    while(true);
  }
  
  // Initialize all motors and encoders
  initializeMotors();
  
  // Clear any latched faults from previous runs
  clearMotorFaults();
  
  // Move all motors to safe positions
  if (!allMotorsToSafePos()){
    Serial.println("ERROR: Failed to reach safe positions!");
    Serial.println("Manually position motors and press RESET");
    while(true);
  }

  // Report emergency recovery if needed
  if (emergencyDetected){
    Serial.println("Emergency recovery complete");
  }
  
  Serial.println("System ready");
  delay(1000);
}

// ======================== MAIN LOOP ========================= //
void loop() {
  // Check for emergency stop
  if (emergencyStopTriggered){
    emergencyStop();
  }
  
  // Wait for user command
  if (!waitForRun()){
    delay(250);
    return;
  }

  Serial.println("");
  Serial.println("=== STARTING SEQUENCE ===");
  setSystemState(SYSTEM_RUNNING);

  // Startup delay before full sequence start
  Serial.println("Sequence beings in five seconds...");
  delay(5000);

  const int TOTAL_STEPS = 7;

  // STEP 1: M3 lowers 90 degrees
  printProgressBar(1, TOTAL_STEPS, "M3 Lowering");
  if (!moveM3ToPos(CONST_90_DEG, -m3Speed, "Lowering")){
    Serial.println("Sequence aborted: M3 lowering failed");
    setSystemState(SYSTEM_SAFE);
    return;
  }

  // Clear M3 fault after lowering
  Serial.println("Clearing M3 fault after lowering...");
  md.setM2Speed(0);
  delay(500);

  // STEP 2: Wait 3 seconds
  printProgressBar(2, TOTAL_STEPS, "Pause");
  delay(3000);
  
  // STEP 3: M1 plunger sequence (extend, hold, retract)
  printProgressBar(3, TOTAL_STEPS, "SQUEEEEZE");
  if (!runM1Sequence()){
    Serial.println("Sequence aborted: M1 failed");
    setSystemState(SYSTEM_SAFE);
    return;
  }

  // STEP 4: Wait 1 second before M2 activates
  printProgressBar(4, TOTAL_STEPS, "Hol' on now");
  Serial.println("Waiting 2 seconds before M2 activates...");
  delay(2000);

  // STEP 5: M2 platform + M4 belt (coordinated sequence)
  printProgressBar(5, TOTAL_STEPS, "Let's bring 'er home");
  if (!runM2Sequence()){
    Serial.println("Sequence aborted: M2 failed");
    setSystemState(SYSTEM_SAFE);
    return;
  }

  // STEP 6: Wait 1 second
  printProgressBar(6, TOTAL_STEPS, "Stop. Hold up");
  delay(1000);

  // STEP 7: M3 raises back to home position
  printProgressBar(7, TOTAL_STEPS, "Let's go home");
  if (!moveM3ToPos(CONST_90_DEG, m3Speed, "Raising")){
    Serial.println("Sequence aborted: M3 raising failed");
    setSystemState(SYSTEM_SAFE);
    return;
  }

  // Reset all motor positions and mark system as safe
  m1Position = 0;
  EEPROM.put(M1_EEPROM_ADDR_POS, 0);
  
  m2Position = 0;
  EEPROM.put(M2_EEPROM_ADDR_POS, 0);
  
  m3Position = 0;
  EEPROM.put(M3_EEPROM_ADDR_POS, 0);
  
  setSystemState(SYSTEM_SAFE);

  printProgressBar(7, TOTAL_STEPS, "HOORAY! We did it!");
  Serial.println("=== SEQUENCE COMPLETE ===");
  Serial.println("");
  delay(SEQUENCE_PAUSE);
}



