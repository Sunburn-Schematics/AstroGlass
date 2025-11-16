/*
=============================================================== //
   PROJECT:   AstroGlass, The  Middle Man, Control System
   PLATFORM:  Arduino MEGA 2560
   DRIVER:    x2 DualVNH5019 Motor Shield
   MOTOR:     Maverick 12V DC Gear Motor w/Encoder (61:1)
   AUTHOR:    Pedro Ortiz
   VERSION:   v1.0
=============================================================== //
*/

#include <EEPROM.h>
#include <Arduino.h>
#include <DualVNH5019MotorShield.h>

// ==================== SHIELD 2 CONTROL PINS ================= //
// M1 Plunger Motor
const int M1_INA = 30;
const int M1_INB = 31;
const int M1_PWM = 12;

// M2 Platform Motor
const int M2_INA = 7;
const int M2_INB = 8;
const int M2_PWM = 11;

// ================= EMERGENCY STOP BUTTON ==================== //
const int EMERGENCY_STOP_PIN = 21;

// ======================= ENCODER PINS ======================= //
// M1 Encoder
const int m1PinA     = 18;
const int m1PinB     = 19;
volatile int m1PrevA = LOW;

// M2 Encoder (NOT CONFIGURED)
const int m2PinA     = 20;
const int m2PinB     = 22;
volatile int m2PrevA = LOW;

// M3 Encoder
const int m3PinA     = 2;
const int m3PinB     = 3;
volatile int m3PrevA = LOW;

// ============== MOTOR-SPECIFIC PARAMETERS =================== //
// M1 Plunger Parameters
const long M1_EXTEND_COUNTS = 200;
const long M1_HOLD_TIME     = 2000;
const int m1Speed           = 75;

// M2 Platform Parameters
const long M2_LOWER_COUNTS = 200;       // How far platform lowers (adjust after testing)
const int m2Speed          = 75;        // Speed for M2 movement

// M3 Conveyor Parameters
const long SAFE_POS_COUNTS   = 0;
const long CONST_90_DEG      = 214;
const long M3_MAX_VALID_POS  = 500;
const long M3_MIN_VALID_POS  = -500;
const int m3Speed            = 75;
const int SAFE_SPEED         = 150;
const int M3_EEPROM_ADDR_POS = 0;

// M4 Belt Parameters
const int m4Speed = 75;

// Extra Parameters
const long countsPerRev = 854;

// ===================== TIMING PARAMETERS ==================== //
const unsigned long MOTOR_TIMEOUT_MOVE  = 15000;
const unsigned long MOTOR_TIMEOUT_HOME  = 15000;
const unsigned long M1_ACTIVATION_DELAY = 10000;
const unsigned long M4_TOTAL_RUN_TIME   = 5000;
const unsigned long SEQUENCE_PAUSE      = 500;

// ================ EEPROM CONFIGURATION ====================== //
const int EEPROM_SYSTEM_STATE_ADDR = 4;
const byte SYSTEM_RUNNING         = 0xAA;
const byte SYSTEM_SAFE            = 0x55;

// ==================== MOTOR POSITIONS ======================= //
volatile long m1Position = 0;
volatile long m2Position = 0;
volatile long m3Position = 0;

// ============ EMERGENCY STOP FLAG ============ //
volatile bool emergencyStopTriggered = false;

// ===== MOTOR SHIELD ===== //
DualVNH5019MotorShield md;

// =================== FUNCTION DECLARATIONS ================== //
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

void updateM1Encoder();
long getM1Position();
void setM1Direction(int dir);
bool runM1Sequence();
void testM1();

void updateM2Encoder();
long getM2Position();
void setM2Direction(int dir);
bool runM2Sequence();
void testM2();

void updateM3Encoder();
long getM3Position();
void savedM3EncoderPos();
long loadM3EncoderPos();
bool moveM3ToPos(long targetCount, int speed, const char* direction);
void testM3();

void runM4Cont(int speed);
void testM4();

// =================== SYSTEM FUNCTIONS ======================= //
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
  
  // Initialize M1 encoder
  pinMode(m1PinA, INPUT_PULLUP);
  pinMode(m1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m1PinA), updateM1Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m1PinB), updateM1Encoder, CHANGE);

  // Initialize M2 control pins (Shield 2)
  pinMode(M2_INA, OUTPUT);
  pinMode(M2_INB, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  digitalWrite(M2_INA, LOW);
  digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);
  
  // Initialize M2 encoder
  pinMode(m2PinA, INPUT_PULLUP);
  pinMode(m2PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m2PinA), updateM2Encoder, CHANGE);
  // Note: m2PinB (22) is not an interrupt pin, but encoder will still work
  
  // Initialize M3 encoder
  pinMode(m3PinA, INPUT_PULLUP);
  pinMode(m3PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
  
  // Initialize Emergency Stop Button
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), emergencyStopISR, FALLING);
}

void clearMotorFaults(){
  Serial.println("Clearing motor faults...");
  
  // Stop all motors
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

void emergencyStopISR(){
  emergencyStopTriggered = true;
}

bool checkMotorFaults(){
  if (md.getM1Fault()) {
    Serial.println("ERROR: M4 fault detected!");
    return true;
  }

  if (md.getM2Fault()){
    Serial.println("ERROR: M3 fault detected!");
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
  
  if (savedPos < M3_MIN_VALID_POS || savedPos > M3_MAX_VALID_POS){
    Serial.println("WARNING: EEPROM data corrupted - resetting to 0");
    
    long zeroValue = 0;
    EEPROM.put(M3_EEPROM_ADDR_POS, zeroValue);
    
    Serial.println("EEPROM reset complete");
    Serial.println("");
    return true;
  } else {
    Serial.println("EEPROM data valid");
    Serial.println("");
    return true;
  }
}

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

void setSystemState(byte state){
  EEPROM.put(EEPROM_SYSTEM_STATE_ADDR, state);
}

bool waitForRun(){
  clearSerialInput();
  Serial.println("Commands: [SPACE] = Start | 1-4 = Test Motor | [S] = Stop");
  
  unsigned long startWaitTime = millis();
  const unsigned long WAIT_TIMEOUT = 60000;

  while (true) {
    if (millis() - startWaitTime > WAIT_TIMEOUT) {
      Serial.println("Timeout - system idle");
      delay(5000);
      return false;
    }
    
    if (Serial.available() > 0){
      char input = Serial.read();

      if (input == ' ') {
        Serial.println("Starting full sequence...");
        return true;
      } else if (input == 'S' || input == 's'){
        Serial.println("System idle");
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

void clearSerialInput(){
  while (Serial.available() > 0){
    Serial.read();
  }
}

void stopAllMotors(){
  setM1Direction(0);
  md.setM2Speed(0);
  md.setM1Speed(0);
}

bool allMotorsToSafePos(){
  bool allSuccess = true;
  
  Serial.println("Moving motors to safe positions...");
  
  // M1 - Plunger
  Serial.println("M1: Stopping plunger");
  setM1Direction(0);
  
  // M2 - Platform
  Serial.println("M2: Not yet implemented");
  
  // M3 - Conveyor
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
  
  // M4 - Belt
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

void emergencyStop(){
  Serial.println("!! EMERGENCY STOP !!");

  stopAllMotors();
  delay(500);

  if (allMotorsToSafePos()){
    Serial.println("System secured successfully");
  } else {
    Serial.println("WARNING: System may not be fully secured!");
  }

  Serial.println("");
  Serial.println("System halted. Press RESET to restart.");
  while(true);
}

void stopMotor(int motorNum){
  if (motorNum == 1) {
    setM1Direction(0);
    Serial.println("M1 stopped");

  } else if (motorNum == 2){
    Serial.println("M2 not yet implemented");

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

// =================== M1 FUNCTIONS ======================= //
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
    m1PrevA = currentA;
  }
}

long getM1Position(){
  noInterrupts();
  long pos = m1Position;
  interrupts();
  return pos;
}

void setM1Direction(int dir){
  if (dir > 0){
    digitalWrite(M1_INA, HIGH);
    digitalWrite(M1_INB, LOW);
    analogWrite(M1_PWM, m1Speed);
  } else if (dir < 0){
    digitalWrite(M1_INA, LOW);
    digitalWrite(M1_INB, HIGH);
    analogWrite(M1_PWM, m1Speed);
  } else {
    digitalWrite(M1_INA, LOW);
    digitalWrite(M1_INB, LOW);
    analogWrite(M1_PWM, 0);
  }
}

bool runM1Sequence(){
  Serial.println("");
  Serial.println("=== M1 PLUNGER SEQUENCE ===");
  
  m1Position = 0;
  
  Serial.println("Extending plunger...");
  setM1Direction(1);
  
  unsigned long startTime = millis();
  while (abs(getM1Position()) < M1_EXTEND_COUNTS) {
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > 10000) {
      Serial.println("ERROR: M1 extend timeout!");
      setM1Direction(0);
      return false;
    }
    delay(10);
  }
  
  setM1Direction(0);
  Serial.println("Extension complete");
  
  Serial.print("Holding for ");
  Serial.print(M1_HOLD_TIME / 1000);
  Serial.println(" seconds...");
  delay(M1_HOLD_TIME);
  
  Serial.println("Retracting plunger...");
  m1Position = 0;
  setM1Direction(-1);
  
  startTime = millis();
  while (abs(getM1Position()) < M1_EXTEND_COUNTS) {
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > 10000) {
      Serial.println("ERROR: M1 retract timeout!");
      setM1Direction(0);
      return false;
    }
    delay(10);
  }
  
  setM1Direction(0);
  Serial.println("=== M1 SEQUENCE COMPLETE ===");
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

// =================== M2 FUNCTIONS ======================= //
void updateM2Encoder(){
  int currentA = digitalRead(m2PinA);
  int currentB = digitalRead(m2PinB);

  if (currentA != m2PrevA) {
    if (currentA == HIGH) {
      if (currentB == LOW) m2Position++;
      else m2Position--;
    } else {
      if (currentB == HIGH) m2Position++;
      else m2Position--;
    }
    m2PrevA = currentA;
  }
}

long getM2Position(){
  noInterrupts();
  long pos = m2Position;
  interrupts();
  return pos;
}

void setM2Direction(int dir){
  if (dir > 0){         // 1 = Lower (platform goes down)
    digitalWrite(M2_INA, HIGH);
    digitalWrite(M2_INB, LOW);
    analogWrite(M2_PWM, m2Speed);
  } else if (dir < 0){  // -1 = Raise (platform goes up)
    digitalWrite(M2_INA, LOW);
    digitalWrite(M2_INB, HIGH);
    analogWrite(M2_PWM, m2Speed);
  } else {              // 0 = Stop
    digitalWrite(M2_INA, LOW);
    digitalWrite(M2_INB, LOW);
    analogWrite(M2_PWM, 0);
  }
}

bool runM2Sequence(){
  Serial.println("");
  Serial.println("=== M2 PLATFORM SEQUENCE ===");
  
  m2Position = 0;
  
  // STEP 1: Lower platform to halfway point (100 counts)
  Serial.println("M2: Lowering to halfway point...");
  setM2Direction(1);  // Lower
  
  unsigned long startTime = millis();
  while (abs(getM2Position()) < 100) {
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > 10000) {
      Serial.println("ERROR: M2 halfway timeout!");
      setM2Direction(0);
      return false;
    }
    delay(10);
  }
  
  // STEP 2: Start M4 at halfway point
  Serial.println("M2: Reached halfway - Starting M4 belt");
  
  // Clear M4 faults before starting
  if (md.getM1Fault() || md.getM2Fault()){
    Serial.println("Clearing faults before M4...");
    md.init();
    delay(500);
    
    // Re-attach M3 encoder
    pinMode(m3PinA, INPUT_PULLUP);
    pinMode(m3PinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
    delay(500);
  }
  
  runM4Cont(m4Speed);
  unsigned long m4StartTime = millis();
  
  // STEP 3: Continue lowering to full position (200 counts total)
  Serial.println("M2: Continuing to full lower position...");
  
  startTime = millis();
  while (abs(getM2Position()) < M2_LOWER_COUNTS) {
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > 10000) {
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
  
  // STEP 4: Wait 1 second (M4 still running)
  Serial.println("M2: Waiting 1 second...");
  delay(1000);
  
  // STEP 5: Raise platform back up
  Serial.println("M2: Raising back to start position...");
  setM2Direction(-1);  // Raise
  
  startTime = millis();
  while (abs(getM2Position()) > 5) {  // Count back to ~0 (allow small tolerance)
    if (emergencyStopTriggered) emergencyStop();
    
    if (millis() - startTime > 10000) {
      Serial.println("ERROR: M2 raise timeout!");
      setM2Direction(0);
      stopMotor(4);
      return false;
    }
    
    // Add position monitoring every 100ms for better feedback
    unsigned long currentTime = millis();
    static unsigned long lastPrintTime = 0;
    if (currentTime - lastPrintTime > 100) {
      Serial.print("  M2 Position: ");
      Serial.println(getM2Position());
      lastPrintTime = currentTime;
    }
    
    delay(10);
  }
  
  setM2Direction(0);
  Serial.println("M2: Back at start position");
  
  // STEP 6: Keep M4 running until 5 seconds total
  unsigned long m4ElapsedTime = millis() - m4StartTime;
  if (m4ElapsedTime < M4_TOTAL_RUN_TIME) {
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

void testM2(){
  Serial.println("Testing M2 Platform (3 seconds)...");
  Serial.println("Platform lowering...");
  m2Position = 0;
  
  setM2Direction(1);  // Lower
  unsigned long startTime = millis();
  
  while (millis() - startTime < 3000) {
    Serial.print("M2 Count: ");
    Serial.println(getM2Position());
    delay(500);
  }
  
  setM2Direction(0);
  Serial.println("M2 test complete");
}

// =================== M3 FUNCTIONS ======================= //
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
    m3PrevA = currentA;
  }
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
}

long loadM3EncoderPos(){
  long savedPos;
  EEPROM.get(M3_EEPROM_ADDR_POS, savedPos);
  return savedPos;
}

bool moveM3ToPos(long targetCount, int speed, const char* direction){
  // CLEAR ANY FAULTS BEFORE STARTING
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
  
  m3Position = 0;
  
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

    // UPDATE MORE OFTEN - Changed from 500ms to 100ms
    if (millis() - lastPrintTime > 100){
      Serial.print("  Count: ");
      Serial.println(getM3Position());
      lastPrintTime = millis();
    }
    delay(10);
  }

  stopMotor(3);
  Serial.print("M3 ");
  Serial.print(direction);
  Serial.println(" complete");
  return true;
}

void testM3(){
  Serial.println("Testing M3 (3 seconds)...");
  
  // CLEAR FAULTS BEFORE TESTING
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
  Serial.println("Starting motor...");
  
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

// =================== M4 FUNCTIONS ======================= //
void runM4Cont(int speed){
  md.setM1Speed(speed);
  Serial.print("M4 running at speed: ");
  Serial.println(speed);
}

void testM4(){
  Serial.println("Testing M4 (3 seconds)...");
  
  // CLEAR FAULTS BEFORE TESTING
  Serial.println("Clearing faults...");
  md.setM1Speed(0);
  delay(100);
  
  if (md.getM1Fault() || md.getM2Fault()){
    Serial.println("Fault detected - reinitializing...");
    md.init();
    delay(500);
    
    // Re-attach M3 encoder (since md.init() clears it)
    pinMode(m3PinA, INPUT_PULLUP);
    pinMode(m3PinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
    delay(500);
  }
  
  Serial.println("Starting motor...");
  md.setM1Speed(m4Speed);
  delay(3000);
  md.setM1Speed(0);
  
  Serial.println("M4 test complete");
}

// ========================= SETUP ============================ //
void setup(){
  Serial.begin(115200);
  delay(500);
  
  Serial.println("AstroGlass Control System v0.33");

  bool emergencyDetected = checkEmergencyStop();
  
  if (!validateAndFixEEPROM()){
    Serial.println("FATAL: EEPROM error!");
    while(true);
  }
  
  initializeMotors();
  
  // CLEAR ANY LATCHED FAULTS FROM PREVIOUS RUNS
  clearMotorFaults();
  
  if (!allMotorsToSafePos()){
    Serial.println("ERROR: Failed to reach safe positions!");
    Serial.println("Manually position motors and press RESET");
    while(true);
  }

  if (emergencyDetected){
    Serial.println("Emergency recovery complete");
  }
  
  Serial.println("System ready");
  delay(1000);
}

// ======================== MAIN LOOP ========================= //
void loop() {
  if (emergencyStopTriggered) {
    emergencyStop();
  }
  
  if (!waitForRun()) {
    delay(250);
    return;
  }

  Serial.println("");
  Serial.println("=== STARTING SEQUENCE ===");
  setSystemState(SYSTEM_RUNNING);

  // Step One: M3 Lowers
  if (!moveM3ToPos(CONST_90_DEG, -m3Speed, "Lowering")){
    Serial.println("Sequence aborted: M3 lowering failed");
    setSystemState(SYSTEM_SAFE);
    return;
  }

  // Clear M3 fault after lowering
  Serial.println("Clearing M3 fault after lowering...");
  md.setM2Speed(0);
  delay(500);

  delay(3000);
  
  // Step Two: M1 Plunger
  if (!runM1Sequence()){
    Serial.println("Sequence aborted: M1 failed");
    setSystemState(SYSTEM_SAFE);
    return;
  }

  // ADD THIS: Wait 2 seconds before M2 activates
  Serial.println("Waiting 2 seconds before M2 activates...");
  delay(2000);

  // Step Three: M2 Platform + M4 Belt (coordinated)
  if (!runM2Sequence()){
    Serial.println("Sequence aborted: M2 failed");
    setSystemState(SYSTEM_SAFE);
    return;
  }

  delay(1000);

  // Step Four: M3 Rises
  if (!moveM3ToPos(CONST_90_DEG, m3Speed, "Raising")){
    Serial.println("Sequence aborted: M3 raising failed");
    setSystemState(SYSTEM_SAFE);
    return;
  }

  // Reset position
  m3Position = 0;
  EEPROM.put(M3_EEPROM_ADDR_POS, 0);
  setSystemState(SYSTEM_SAFE);

  Serial.println("=== SEQUENCE COMPLETE ===");
  Serial.println("");
  delay(SEQUENCE_PAUSE);
}