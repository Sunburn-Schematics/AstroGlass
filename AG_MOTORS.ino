// ============================================================= //
// PROJECT:   AstroGlass, The Middle Man, Control System
// PLATFORM:  Arduino MEGA 2560
// DRIVER:    x2 DualVNH5019 Motor Shield
// MOTOR:     x4 Maverick 12V DC Gear Motor w/Encoder (61:1)
// AUTHOR:    Pedro Ortiz
// VERSION:   v1.6.3
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

// ======================== MOTOR SPEEDS ====================== //
const int m1Speed = 64;     // PWM speed (0-255)
const int m2Speed = 64;     // PWM speed (0-255)
const int m3Speed = 100;    // Shield speed (-400-400)
const int m4Speed = 100;    // Shield speed (-400-400)

// ==================== SHIELD 2 CONTROL PINS ================= //
// M1 Plunger Motor
const int M1_INA = 30;      // M1 Direction A
const int M1_INB = 31;      // M1 Direction B
const int M1_PWM = 11;      // M1 PWM Speed Control

// M2 Platform Motor
const int M2_INA = 7;       // M2 Direction A
const int M2_INB = 8;       // M2 Direction B
const int M2_PWM = 12;      // M2 PWM Speed Control

// ======================= ENCODER PINS ======================= //
// M1 Plunger Encoder
const int m1PinA     = 18;      // Encoder Channel A (interrupt)
const int m1PinB     = 19;      // Encoder Channel B (interrupt)
volatile int m1PrevA = LOW;
volatile int m1PrevB = LOW;

// M2 Platform Encoder
const int m2PinA     = 20;      // Encoder Channel A (interrupt)
const int m2PinB     = 21;      // Encoder Channel B (interrupt)
volatile int m2PrevA = LOW;
volatile int m2PrevB = LOW;

// M3 Conveyor Encoder
const int m3PinA     = 2;       // Encoder Channel A (interrupt)
const int m3PinB     = 3;       // Encoder Channel B (interrupt)
volatile int m3PrevA = LOW;
volatile int m3PrevB = LOW;

// ============== MOTOR-SPECIFIC PARAMETERS =================== //
// M1 Plunger Parameters
const int M1_POSITION_TOLERANCE = 50;        // Acceptable position error (counts)
const long M1_EXTEND_COUNTS     = 44408;     // Extension distance: 52 rotations
const long M1_HOLD_TIME         = 1000;      // Compression hold time (milliseconds)

// M2 Platform Parameters
const int M2_POSITION_TOLERANCE = 50;        // Acceptable position error (counts)
const long M2_LOWER_COUNTS      = 78568;     // Lowering distance: 92 rotations

// M3 Conveyor Parameters
const int M3_POSITION_TOLERANCE = 10;        // Acceptable position error (counts)
const long SAFE_POS_COUNTS      = 0;         // Home/safe position
const long CONST_90_DEG         = 214;       // 90-degree movement (1/4 revolution)
const long M3_MAX_VALID_POS     = 500;       // Maximum valid EEPROM position
const long M3_MIN_VALID_POS     = -500;      // Minimum valid EEPROM position
const int SAFE_SPEED            = 150;       // Speed for returning to safe position

// Extra Parameters
const long countsPerRev = 854;          // Maverick: 7 PPR × 2 edges × 61:1 gear

// ===================== TIMING PARAMETERS ==================== //
const unsigned long MOTOR_TIMEOUT_MOVE  = 15000;   // M3 movement timeout (15s)
const unsigned long MOTOR_TIMEOUT_HOME  = 15000;   // M3 homing timeout (15s)
const unsigned long M1_TIMEOUT          = 75000;   // M1 movement timeout (75s)
const unsigned long M2_TIMEOUT          = 120000;  // M2 movement timeout (120s)
const unsigned long M4_TOTAL_RUN_TIME   = 7000;    // M4 total run duration (7s)

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

// ================= MOTOR SHIELD OBJECT ====================== //
DualVNH5019MotorShield md;

// =================== FUNCTION DECLARATIONS ================== //
// System Functions
void initializeMotors();
void clearMotorFaults();
bool checkMotorFaults();
bool validateAndFixEEPROM();
void clearSerialInput();
void emergencyStop();
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
bool moveM1ToPosition(long targetPosition, unsigned long timeout);
void testM1();

// M2 Functions
void updateM2Encoder();
long getM2Position();
void savedM2EncoderPos();
long loadM2EncoderPos();
void setM2Direction(int dir);
bool runM2Sequence();
bool moveM2ToPosition(long targetPosition, unsigned long timeout);
void testM2();

// M3 Functions
void updateM3Encoder();
long getM3Position();
void savedM3EncoderPos();
long loadM3EncoderPos();
void brakeM3();
bool moveM3ToPosition(long targetCount, unsigned long timeout);
void testM3();

// M4 Functions
void runM4Cont(int speed);
void testM4();

// =================== SYSTEM FUNCTIONS ======================= //
// Initialize all motors and encoders
void initializeMotors(){
  // Initialize M1 control pins
  pinMode(M1_INA, OUTPUT);
  pinMode(M1_INB, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  digitalWrite(M1_INA, LOW);
  digitalWrite(M1_INB, LOW);
  analogWrite(M1_PWM, 0);

  // Initialize M2 control pins
  pinMode(M2_INA, OUTPUT);
  pinMode(M2_INB, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  digitalWrite(M2_INA, LOW);
  digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);

  delay(200);

  // Initialize Shield 1 (M3 & M4)
  md.init();
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(100);
  
  // Initialize M1 encoder with both interrupt pins
  pinMode(m1PinA, INPUT_PULLUP);
  pinMode(m1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m1PinA), updateM1Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m1PinB), updateM1Encoder, CHANGE);
  
  // Initialize M2 encoder (pin 22 is not an interrupt pin)
  pinMode(m2PinA, INPUT_PULLUP);
  pinMode(m2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m2PinA), updateM2Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m2PinB), updateM2Encoder, CHANGE);
  
  // Initialize M3 encoder with both interrupt pins
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);

  // Final verification - ensure all motors stopped
  digitalWrite(M1_INA, LOW);
  digitalWrite(M1_INB, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_INA, LOW);
  digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);
  md.setM1Speed(0);
  md.setM2Speed(0);
}

// Clear motor faults on startup
void clearMotorFaults(){
  Serial.println("Clearing motor faults...");
  
  // Stop all shield motors
  digitalWrite(M1_INA, LOW);
  digitalWrite(M1_INB, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_INA, LOW);
  digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(100);
  
  // Reinitialize motor shield
  md.init();
  delay(500);

  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(100);
  
  // Re-attach M3 encoder interrupts
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);

  // Verify M1/M2 still stopped (redundant safety)
  digitalWrite(M1_INA, LOW);
  digitalWrite(M1_INB, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_INA, LOW);
  digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);
  delay(100);
  
  Serial.println("Motor faults cleared");
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
  
  // Check M1 position (valid range: -50,000 to +50,000)
  long savedM1Pos;
  EEPROM.get(M1_EEPROM_ADDR_POS, savedM1Pos);
  Serial.print("M1 EEPROM value: ");
  Serial.println(savedM1Pos);
  
  if (savedM1Pos < -50000 || savedM1Pos > 50000){
    Serial.println("WARNING: M1 EEPROM corrupted - resetting to 0");
    long zeroValue = 0;
    EEPROM.put(M1_EEPROM_ADDR_POS, zeroValue);
    allValid = false;
  }
  
  // Check M2 position (valid range: -85,000 to +85,000)
  long savedM2Pos;
  EEPROM.get(M2_EEPROM_ADDR_POS, savedM2Pos);
  Serial.print("M2 EEPROM value: ");
  Serial.println(savedM2Pos);
  
  if (savedM2Pos < -85000 || savedM2Pos > 85000){
    Serial.println("WARNING: M2 EEPROM corrupted - resetting to 0");
    long zeroValue = 0;
    EEPROM.put(M2_EEPROM_ADDR_POS, zeroValue);
    allValid = false;
  }
  
  // Check M3 position (same as before)
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
  
  const long M1_HOME_POSITION = 0;  // Fully retracted
  
  if (!moveM1ToPosition(M1_HOME_POSITION, M1_TIMEOUT)){
    Serial.println("M1: Failed to reach safe position!");
    allSuccess = false;
  } else {
    m1Position = M1_HOME_POSITION;
    savedM1EncoderPos();
    Serial.println("M1: Safe");
  }
  
  // M2 - Platform: Return to home position (fully raised)
  Serial.println("M2: Moving to safe position...");
  
  long savedM2Pos = loadM2EncoderPos();
  m2Position = savedM2Pos;
  
  const long M2_HOME_POSITION = 0;  // Fully raised
  
  if (!moveM2ToPosition(M2_HOME_POSITION, M2_TIMEOUT)){
    Serial.println("M2: Failed to reach safe position!");
    allSuccess = false;
  } else {
    m2Position = M2_HOME_POSITION;
    savedM2EncoderPos();
    Serial.println("M2: Safe");
  }
  
  // M3 - Conveyor: Return to home position
  Serial.println("M3: Moving to safe position...");
  
  long savedPos = loadM3EncoderPos();
  m3Position = savedPos;
  
  unsigned long startTime = millis();

  // Closed-loop control for M3
  while (abs(getM3Position() - SAFE_POS_COUNTS) > M3_POSITION_TOLERANCE){
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
    
    long error = SAFE_POS_COUNTS - getM3Position();
    
    if (abs(error) <= M3_POSITION_TOLERANCE){
      md.setM2Speed(0);
      break;
    } else if (error > 0){
      md.setM2Speed(SAFE_SPEED);
    } else {
      md.setM2Speed(-SAFE_SPEED);
    }
    
    delay(10);
  }

  brakeM3();

  // Only set position if we actually reached target
  if (abs(getM3Position() - SAFE_POS_COUNTS) <= M3_POSITION_TOLERANCE){
    m3Position = SAFE_POS_COUNTS;
    savedM3EncoderPos();
    Serial.println("M3: Safe");
  } else {
    Serial.println("M3: WARNING - Not at safe position!");
    savedM3EncoderPos();  // Save actual position, not ideal position
  }
  
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
    brakeM3();
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
    if (currentA == HIGH){
      if (currentB == LOW) m1Position++;
      else m1Position--;
    } else {
      if (currentB == HIGH) m1Position++;
      else m1Position--;
    }
    m1PrevA = currentA;
  }

  // Detect edge on channel B
  if (currentB != m1PrevB){
    if (currentB == HIGH){
      if (currentA == HIGH) m1Position++;
      else m1Position--;
    } else {
      if (currentA == LOW) m1Position++;
      else m1Position--;
    }
    m1PrevB = currentB;
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
    analogWrite(M1_PWM, 255);
  }
}

// Run M1 plunger sequence: extend, hold, retract
bool runM1Sequence(){
  Serial.println("");
  Serial.println("=== M1 PLUNGER SEQUENCE ===");
  
  m1Position = 0;  // Reset encoder position

  // STEP 0: Apply brake to M2 to prevent movement during compression
  setM2Direction(0);
  analogWrite(M2_PWM, 255);
  delay(100);
  
  // STEP 1: Extend plunger
  Serial.println("Extending plunger until resistance...");
  setM1Direction(1);
  
  long lastPosition = 0;
  unsigned long lastMovementTime = millis();
  unsigned long lastPrintTime = millis();
  const unsigned long STALL_TIMEOUT = 200;
  const long MIN_MOVEMENT = 2;
  const long MAX_EXTENSION_LIMIT = M1_EXTEND_COUNTS; // Safety limit

  unsigned long startTime = millis();
  while (true){
    // Check for overall timeout (safety caution)
    if (millis() - startTime > M1_TIMEOUT){
      Serial.println("ERROR: M1 extend timeout!");
      setM1Direction(0);
      analogWrite(M2_PWM, 0);
      return false;
    }

    long currentPosition = getM1Position();

    if (abs(currentPosition) >= MAX_EXTENSION_LIMIT){
      Serial.println("MAXIMUM Extension limit reached");
      Serial.print("Position at limit: ");
      Serial.println(currentPosition);
      break;
    }

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

    // Print progress every 5 seconds
    if (millis() - lastPrintTime > 5000){
      Serial.print("M1 extending... Position: ");
      Serial.println(getM1Position());
      lastPrintTime = millis();
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
    setM1Direction(-1);

    lastPosition = getM1Position();
    lastMovementTime = millis();
    startTime = millis();

    while (abs(getM1Position()) > M1_POSITION_TOLERANCE){  // Retract to ~0 (with some tolerance)
      if (millis() - startTime > M1_TIMEOUT){
        Serial.println("ERROR: M1 retract timeout!");
        setM1Direction(0);
        analogWrite(M2_PWM, 0);
        return false;
      }

      long currentPosition = getM1Position();

      if (abs(currentPosition - lastPosition) > MIN_MOVEMENT){
        lastPosition = currentPosition;
        lastMovementTime = millis();
      }

      // Stall detection during retraction
      if (millis() - lastMovementTime > STALL_TIMEOUT){
        Serial.println("WARNING: M1 stalled during retraction!");
        Serial.print("Stuck at position: ");
        Serial.println(currentPosition);
        setM1Direction(0);
        return false;
      }
      
      delay(10);
    }
    
    setM1Direction(0);
    analogWrite(M2_PWM, 0);

    savedM1EncoderPos();
    Serial.println("=== M1 SEQUENCE COMPLETE ===");
    return true;
}

// Moving m1 to the target location with a closed loop controller
bool moveM1ToPosition(long targetPosition, unsigned long timeout){
  Serial.print("M1: Moving to position ");
  Serial.println(targetPosition);

  unsigned long startTime = millis();
  long lastPosition = getM1Position();
  unsigned long lastMovementTime = millis();
  const unsigned long STALL_TIMEOUT = 500;  // 500ms no movement = stalled
  const long MIN_MOVEMENT = 5;  // Minimum movement to reset stall timer

  while (abs(getM1Position() - targetPosition) > M1_POSITION_TOLERANCE){
    // Checks for timeout
    if (millis() - startTime > timeout){
      Serial.println("ERROR: M1 position timeout!");
      setM1Direction(0);
      return false;
    }

    long currentPosition = getM1Position();

    // Check if motor is moving
    if (abs(currentPosition - lastPosition) > MIN_MOVEMENT){
      lastPosition = currentPosition;
      lastMovementTime = millis();
    }

    // Stall detection
    if (millis() - lastMovementTime > STALL_TIMEOUT){
      Serial.println("ERROR: M1 stalled during homing!");
      Serial.print("Stuck at position: ");
      Serial.println(currentPosition);
      setM1Direction(0);
      return false;
    }

    // Calculate the error
    long error = targetPosition - getM1Position();

    // P-Controller
    if (abs(error) <= M1_POSITION_TOLERANCE){
      setM1Direction(0);
      break;
    } else if (error > 0){
      setM1Direction(1);  // Extend
    } else {
      setM1Direction(-1); // Retract
    }

    delay(10);
  }

  setM1Direction(0);
  Serial.print("M1: Reached position");
  Serial.println(getM1Position());
  return true;
}

// Test M1 motor - Full rotation forward and reverse
void testM1(){
  Serial.println("Testing M1 - Rotation Test");
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
// M2 encoder interrupt service routine (full quadrature)
void updateM2Encoder(){
  int currentA = digitalRead(m2PinA);
  int currentB = digitalRead(m2PinB);

  // Detect edge on channel A
  if (currentA != m2PrevA){
    if (currentA == HIGH){
      if (currentB == LOW) m2Position++;
      else m2Position--;
    } else {
      if (currentB == HIGH) m2Position++;
      else m2Position--;
    }
    m2PrevA = currentA;
  }

  // Detect edge on channel B
  if (currentB != m2PrevB){
    if (currentB == HIGH){
      if (currentA == HIGH) m2Position++;
      else m2Position--;
    } else {
      if (currentA == LOW) m2Position++;
      else m2Position--;
    }
    m2PrevB = currentB;
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
    analogWrite(M2_PWM, 255);
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
  const long MAX_LOWER_LIMIT = M2_LOWER_COUNTS;

  Serial.print("Halfway point: ");
  Serial.println(m2HalfwayPoint);
  Serial.print("Maximum lower limit: ");
  Serial.println(MAX_LOWER_LIMIT);

  unsigned long startTime = millis();
  while (abs(getM2Position()) < m2HalfwayPoint){
    if (millis() - startTime > M2_TIMEOUT / 2){
      Serial.println("ERROR: M2 halfway timeout!");
      setM2Direction(0);
      return false;
    }

    if (abs(getM2Position()) >= MAX_LOWER_LIMIT){
      Serial.println("ERROR: M2 exceeded maximum lower limit at halfway!");
      Serial.print("Position at limit: ");
      Serial.println(getM2Position());
      setM2Direction(0);
      return false;
    }

    delay(10);
  }
  
  // STEP 2: Start M4 belt at halfway point
  Serial.println("M2: Reached halfway - Starting M4 belt");
  
  // CRITICAL: Lock M3 before starting M4
  Serial.println("Locking M3 conveyor...");
  md.setM2Speed(0);
  md.setM2Brake(400);
  delay(200);

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

  Serial.println("Re-locking M3 after init...");
  md.setM2Brake(400);
  delay(200);
  
  runM4Cont(m4Speed);
  unsigned long m4StartTime = millis();
  
  // STEP 3: Continue lowering to full position
  Serial.println("M2: Continuing to full lower position...");
  
  startTime = millis();
  while (abs(getM2Position()) < M2_LOWER_COUNTS){
    if (millis() - startTime > M2_TIMEOUT / 2){
      Serial.println("ERROR: M2 full lower timeout!");
      setM2Direction(0);
      stopMotor(4);
      return false;
    }

    // SAFETY CHECK
    if (abs(getM2Position()) >= MAX_LOWER_LIMIT){
      Serial.println("M2 reached maximum lower limit");
      Serial.print("Position at limit: ");
      Serial.println(getM2Position());
      setM2Direction(0);
      break;
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
  
  while (abs(getM2Position()) > M2_POSITION_TOLERANCE){  // Allow tolerance
    if (millis() - startTime > M2_TIMEOUT){
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
  
  // STEP 6: Keep M4 running until 7 seconds total
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

  md.setM2Brake(400);
  
  Serial.println("=== M2 PLATFORM SEQUENCE COMPLETE ===");
  return true;
}

// Moving M2 to the target position with a closed loop controller
bool moveM2ToPosition(long targetPosition, unsigned long timeout){
  Serial.print("M2: Moving to position ");
  Serial.println(targetPosition);

  unsigned long startTime = millis();
  long lastPosition = getM2Position();
  unsigned long lastMovementTime = millis();
  const unsigned long STALL_TIMEOUT = 500;
  const long MIN_MOVEMENT = 5;

  while (abs(getM2Position() - targetPosition) > M2_POSITION_TOLERANCE){
    // Checks for timeout
    if (millis() - startTime > timeout){
      Serial.println("ERROR: M2 position timeout!");
      setM2Direction(0);
      return false;
    }

    long currentPosition = getM2Position();

    // Stall detection
    if (abs(currentPosition - lastPosition) > MIN_MOVEMENT){
      lastPosition = currentPosition;
      lastMovementTime = millis();
    }

    if (millis() - lastMovementTime > STALL_TIMEOUT){
      Serial.println("ERROR: M2 stalled during homing!");
      Serial.print("Stuck at position: ");
      Serial.println(currentPosition);
      setM2Direction(0);
      return false;
    }

    // Calculate the error
    long error = targetPosition - getM2Position();

    // P-Controller
    if (abs(error) <= M2_POSITION_TOLERANCE){
      setM2Direction(0);
      break;
    } else if (error > 0){
      setM2Direction(1);  // Lower
    } else {
      setM2Direction(-1); // Raise
    }

    delay(10);
  }

  setM2Direction(0);
  Serial.print("M2: Reached position");
  Serial.println(getM2Position());
  return true;
}

// Test M2 motor - Full rotation forward and reverse
void testM2(){
  Serial.println("Testing M2 - Rotation Test");
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
    if (currentA == HIGH){
      if (currentB == LOW) m3Position++;
      else m3Position--;
    } else {
      if (currentB == HIGH) m3Position++;
      else m3Position--;
    }
    m3PrevA = currentA;
  }

  // Detect edge on channel B
  if (currentB != m3PrevB){
    if (currentB == HIGH){
      if (currentA == HIGH) m3Position++;
      else m3Position--;
    } else {
      if (currentA == LOW) m3Position++;
      else m3Position--;
    }
    m3PrevB = currentB;
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

void brakeM3(){
  md.setM2Brake(400);   // Maximum brake force
}

// Move M3 to target position with closed-loop control
bool moveM3ToPosition(long targetCount, unsigned long timeout){
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
  
  // m3Position = 0;  // Reset for relative movement
  long startPosition = getM3Position();
  long targetPosition = startPosition + targetCount;
  
  Serial.print("M3: Moving from ");
  Serial.print(startPosition);
  Serial.print(" to ");
  Serial.println(targetPosition);

  unsigned long startTime = millis();
  
  // Closed-loop position control
  while (abs(getM3Position() - targetPosition) > M3_POSITION_TOLERANCE){
    // Check for timeout
    if (millis() - startTime > MOTOR_TIMEOUT_MOVE){
      Serial.print("ERROR: M3 position timeout");
      stopMotor(3);
      return false;
    }

    if (checkMotorFaults()){
      Serial.println("Fault during M3 movement!");
      stopMotor(3);
      return false;
    }

    // Calculate error and adjust speed
    long error = targetPosition - getM3Position();
    int adjustedSpeed;
    
    // Slow down as we approach target (proportional control)
    if (abs(error) < 50){
      // Within 50 counts - slow speed
      adjustedSpeed = SAFE_SPEED;
    } else {
      // Far from target - full speed
      adjustedSpeed = m3Speed;
    }
    
    // Set direction based on error
    if (abs(error) <= M3_POSITION_TOLERANCE){
      md.setM2Speed(0);
      break;
    } else if (error > 0){
      md.setM2Speed(adjustedSpeed);
    } else {
      md.setM2Speed(-adjustedSpeed);
    }
    
    delay(10);
  }

  brakeM3();
  savedM3EncoderPos();
  Serial.print("M3: Reached position");
  Serial.println(getM3Position());
  return true;
}

// Test M3 motor - Full rotation forward and reverse
void testM3(){
  Serial.println("Testing M3 - 90 Degree Test");
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
  Serial.println("FORWARD: 90 Degrees...");
  md.setM2Speed(m3Speed);
  
  unsigned long startTime = millis();
  while (abs(getM3Position()) < CONST_90_DEG){
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
  Serial.println("REVERSE: 90 Degrees...");
  m3Position = 0;  // Reset counter
  md.setM2Speed(-m3Speed);
  
  startTime = millis();
  while (abs(getM3Position()) < CONST_90_DEG){
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

// Test M4 motor - Belt forward test
void testM4(){
  Serial.println("Testing M4 - Forward Test");
  Serial.println("==================================");
  
  Serial.println("Applying M3 brake...");
  md.setM2Speed(0);
  md.setM2Brake(400);
  delay(200);

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

  // Re-Apply Brake
  md.setM2Brake(400);
  delay(200);
  
  // FORWARD: 3 seconds
  Serial.println("FORWARD: Running for 3 seconds...");
  md.setM1Speed(m4Speed);
  delay(3000);
  md.setM1Speed(0);
  Serial.println("Forward complete.");
  Serial.println("");

  // Keep M3 brake engaged
  md.setM2Brake(400);
  
  Serial.println("=== M4 TEST COMPLETE ===");
}

// ========================= SETUP ============================ //
void setup(){
  Serial.begin(115200);
  delay(500);
  
  Serial.println("AstroGlass Control System v1.6.2");

  // IMMEDIATELY disable ALL motors
  pinMode(M1_INA, OUTPUT);
  pinMode(M1_INB, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  digitalWrite(M1_INA, LOW);
  digitalWrite(M1_INB, LOW);
  analogWrite(M1_PWM, 0);
  
  pinMode(M2_INA, OUTPUT);
  pinMode(M2_INB, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  digitalWrite(M2_INA, LOW);
  digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);
  delay(200);

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

  // Verify motors are still stopped (redundant safety)
  digitalWrite(M1_INA, LOW);
  digitalWrite(M1_INB, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_INA, LOW);
  digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);
  md.setM1Speed(0);
  md.setM2Speed(0);
  delay(100);
  
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
void loop(){
  clearSerialInput();

  Serial.println("");
  Serial.println("========================================");
  Serial.println("   MANUAL SEQUENCE CONTROL MODE");
  Serial.println("========================================");
  Serial.println("Motor Control Keys:");
  Serial.println("  [1] - M1 Plunger Sequence");
  Serial.println("  [2] - M2 Platform Sequence");
  Serial.println("  [3] - M3 Lower to 90°");
  Serial.println("  [4] - M3 Raise up 90°");
  Serial.println("  [5] - M4 Belt Run (7 seconds)");
  Serial.println("");
  Serial.println("Position Controls:");
  Serial.println("  [H] - Home All Motors");
  Serial.println("  [R] - Reset All Positions to Zero");
  Serial.println("");
  Serial.println("Testing:");
  Serial.println("  [T1] - Test M1");
  Serial.println("  [T2] - Test M2");
  Serial.println("  [T3] - Test M3");
  Serial.println("  [T4] - Test M4");
  Serial.println("");
  Serial.println("System:");
  Serial.println("  [S] - Show Current Positions");
  Serial.println("  [?] - Team Credits");
  Serial.println("========================================");

  // Wait for the input
  while (true){
    if (Serial.available() > 0){
      char input = Serial.read();
      clearSerialInput();

      // ========== MOTOR CONTROL COMMANDS ========== //
      if (input == '1'){
        Serial.println("");
        Serial.println(">>> RUNNING M1 <<<");
        setSystemState(SYSTEM_RUNNING);

        printProgressBar(0, 1, "M1 Activated");

        if (runM1Sequence()){
          printProgressBar(1, 1, "M1 Complete");
          Serial.println("M1 Sequence Complete");
        } else {
          Serial.println("M1 Sequence Failed");
        }

        setSystemState(SYSTEM_SAFE);
        break;

      } else if (input == '2'){
        // M2 Platform Sequence
        Serial.println("");
        Serial.println(">>> RUNNING M2 <<<");
        setSystemState(SYSTEM_RUNNING);

        printProgressBar(0, 1, "Starting M2");

        if (runM2Sequence()){
          printProgressBar(1, 1, "M2 Complete");
          Serial.println("M2 Sequence Complete");
        } else {
          Serial.println("M2 Sequence Failed");
        }

        setSystemState(SYSTEM_SAFE);
        break;

      } else if (input == '3'){
        // M3 Lower 90 degrees
        Serial.println("");
        Serial.println(">>> LOWERING M3 90 DEGREES <<<");
        setSystemState(SYSTEM_RUNNING);

        printProgressBar(0, 1, "Lowering M3");

        if (moveM3ToPosition(-CONST_90_DEG, MOTOR_TIMEOUT_MOVE)){
          printProgressBar(1, 1, "M3 Lowered");
          Serial.println("M3 Lowering Complete");
        } else {
          Serial.println("M3 Lowering Failed");
        }

        setSystemState(SYSTEM_SAFE);
        break;

      } else if (input == '4'){
        // M3 Raise 90 degrees
        Serial.println("");
        Serial.println(">>> RAISING M3 90 DEGREES <<<");
        setSystemState(SYSTEM_RUNNING);

        printProgressBar(0, 1, "Raising M3");

        if (moveM3ToPosition(CONST_90_DEG, MOTOR_TIMEOUT_MOVE)){
          printProgressBar(1, 1, "M3 Raised");
          Serial.println("M3 Raising Complete");
        } else {
          Serial.println("M3 Raising Failed");
        }

        setSystemState(SYSTEM_SAFE);
        break;

      } else if (input == '5'){
        // M4 Belt Run
        Serial.println("");
        Serial.println(">>> RUNNING M4 BELT FOR 7 SECONDS <<<");
        setSystemState(SYSTEM_RUNNING);

        printProgressBar(0, 1, "Starting M4");

        // Brake M3
        Serial.println("Locking M3 conveyor...");
        md.setM2Speed(0);
        md.setM2Brake(400);
        delay(200);

        // Clear faults before M4
        Serial.println("Clearing faults before M4...");
        md.setM1Speed(0);
        delay(100);
        md.init();
        delay(500);

        // Re-attach M3 encoder
        pinMode(m3PinA, INPUT_PULLUP);
        pinMode(m3PinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
        attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
        delay(500);

        // RE-APPLY M3 BRAKE after reinit
        md.setM2Brake(400);
        delay(200);

        runM4Cont(m4Speed);
        delay(M4_TOTAL_RUN_TIME);
        md.setM1Speed(0);

        md.setM2Brake(400);

        printProgressBar(1, 1, "M4 Complete");
        Serial.println("M4 Belt Complete");
        setSystemState(SYSTEM_SAFE);
        break;

      // ========== POSITION CONTROL COMMANDS ========== //
      } else if (input == 'H' || input == 'h'){
        // Home All Motors
        Serial.println("");
        Serial.println(">>> HOMING ALL MOTORS <<<");
        setSystemState(SYSTEM_RUNNING);

        printProgressBar(0, 4, "Homing M1");

        if (allMotorsToSafePos()){
          printProgressBar(4, 4, "All Motors Homed");
          Serial.println("All Motors Homed");
        } else {
          Serial.println("Homing Failed!");
        }

        setSystemState(SYSTEM_SAFE);
        break;

      } else if (input == 'R' || input == 'r'){
        // Reset All Positions
        Serial.println("");
        Serial.println(">>> RESETTING ALL POSITIONS TO ZERO <<<");

        m1Position = 0;
        savedM1EncoderPos();
        Serial.println("M1 position reset to 0");

        m2Position = 0;
        savedM2EncoderPos();
        Serial.println("M2 position reset to 0");

        m3Position = 0;
        savedM3EncoderPos();
        Serial.println("M3 position reset to 0");

        Serial.println("All Positions Reset");
        break;

      // ========== SYSTEM INFO COMMANDS ========== //
      } else if (input == 'S' || input == 's'){
        // Show Current Positions
        Serial.println("");
        Serial.println("========================================");
        Serial.println("     CURRENT MOTOR POSITIONS");
        Serial.println("========================================");
        Serial.print("M1 (Plunger):  ");
        Serial.print(getM1Position());
        Serial.println(" counts");

        Serial.print("M2 (Platform): ");
        Serial.print(getM2Position());
        Serial.println(" counts");

        Serial.print("M3 (Conveyor): ");
        Serial.print(getM3Position());
        Serial.println(" counts");

        Serial.println("========================================");
        Serial.println("");
        break;

      // ========== TEST COMMANDS ========== //
      } else if (input == 'T' || input == 't'){
        Serial.println("Waiting for test number (1-4)...");

        unsigned long testWaitStart = millis();
        while (millis() - testWaitStart < 10000){  // 10 second timeout
          if (Serial.available() > 0){
            char testNum = Serial.read();
            clearSerialInput();

            if (testNum == '1'){
              Serial.println("");
              Serial.println(">>> TESTING M1 - FULL ROTATION <<<");
              testM1();
              break;
            } else if (testNum == '2'){
              Serial.println("");
              Serial.println(">>> TESTING M2 - FULL ROTATION <<<");
              testM2();
              break;
            } else if (testNum == '3'){
              Serial.println("");
              Serial.println(">>> TESTING M3 - 90 DEGREES <<<");
              testM3();
              break;
            } else if (testNum == '4'){
              Serial.println("");
              Serial.println(">>> TESTING M4 - BELT <<<");
              testM4();
              break;
            } else {
              Serial.println("Invalid test number. Use 1, 2, 3, or 4");
              break;
            }
          }
          delay(10);
        }

        if (millis() - testWaitStart >= 10000){
          Serial.println("Test command timeout. Please try again.");
        }
        break;

      // ========== CREDITS ========== //
      } else if (input == '?'){
        Serial.println("");
        Serial.println("========================================");
        Serial.println("           TEAM CREDITS");
        Serial.println("========================================");
        Serial.println("Thanks to: Jordan, Robby, Aidan,");
        Serial.println("           Charles, and Brad");
        Serial.println("========================================");
        Serial.println("");
        break;

      } else {
        Serial.println("Invalid Command dude, try again.");
        break;
      }
    }

    delay(10);
  }

  delay(500);
}

