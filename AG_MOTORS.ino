// ============================================================= //
// PROJECT:   AstroGlass Control System
// PLATFORM:  Arduino MEGA 2560
// DRIVER:    x2 DualVNH5019 Motor Shield
// MOTOR(S):  x3 Maverick 12V DC Gear Motor w/Encoder (61:1),
//            x1 Maverick Planetary DC Gear Motor w/Encoder (3.7:1)
// AUTHOR:    Pedro Ortiz
// VERSION:   2.0.1
// ============================================================= //

#include "AG_MOTORS.h"

// Store strings in program memory instead of RAM
#define PRINT_PROGMEM(str) Serial.println(F(str))
#define PRINT_PROGMEM_INLINE(str) Serial.print(F(str))

// ========================= SETUP ============================ //
void setup(){
  // Initialize shield IMMEDIATELY (controls M3 & M4)
  md.init();
  md.setM1Speed(0);  // Stop M4
  md.setM2Speed(0);  // Stop M3
  delay(100);
  
  // Stop M1 and M2
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
  
  // NOW start serial and rest of initialization
  Serial.begin(115200);
  delay(500);
  
  printStartUp();
  delay(2000);

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
  printMainMenu();

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

        if (moveM3ToPosition(CONST_90_DEG, MOTOR_TIMEOUT_MOVE)){
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

        if (moveM3ToPosition(-CONST_90_DEG, MOTOR_TIMEOUT_MOVE)){
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
        Serial.println(">>> RUNNING M4 BELT FOR 3 SECONDS <<<");
        setSystemState(SYSTEM_RUNNING);

        printProgressBar(0, 1, "Starting M4");

        // Stop and brake M3
        Serial.println("Locking M3 conveyor...");
        md.setM2Speed(0);
        delay(100);
        md.setM2Brake(400);
        delay(300);

        // ALWAYS reinitialize for clean state
        Serial.println("Reinitializing shield...");
        md.setM1Speed(0);
        delay(100);
        md.init();
        delay(1000);  // Longer delay
        
        // Re-attach M3 encoder
        pinMode(m3PinA, INPUT_PULLUP);
        pinMode(m3PinB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(m3PinA), updateM3Encoder, CHANGE);
        attachInterrupt(digitalPinToInterrupt(m3PinB), updateM3Encoder, CHANGE);
        delay(500);

        // Re-apply M3 brake
        Serial.println("Re-locking M3...");
        md.setM2Brake(400);
        delay(300);

        // Start M4 with REVERSED direction and HIGH speed
        Serial.println("Starting M4...");
        md.setM1Speed(m4Speed);
        delay(200);

        // Diagnostic check
        if (md.getM1Fault()){
          Serial.println("ERROR: M4 fault detected!");
        } else {
          Serial.println("M4 running successfully");
        }

        delay(M4_TOTAL_RUN_TIME);
        
        Serial.println("Stopping M4...");
        md.setM1Speed(0);
        delay(100);
        md.setM2Brake(400);

        printProgressBar(1, 1, "M4 Complete");
        Serial.println("M4 Belt Complete");
        setSystemState(SYSTEM_SAFE);
        break;

      } else if (input == '6'){
        Serial.println("");
        Serial.println(">>> RUNNING M1 AND M2 SEQUENCE <<<");
        setSystemState(SYSTEM_RUNNING);

        // STEP 1: Run M1 Plunger
        printProgressBar(0, 2, "Starting M1");

        if (runM1Sequence()){
          printProgressBar(1, 2, "M1 Complete");
          Serial.println("M1 Sequence Complete");
        } else {
          Serial.println("M1 Sequence failed - Aborting");
          setSystemState(SYSTEM_SAFE);
          break;
        }

        // 1 second delay
        delay(1000);

        // STEP 2: Run M2 Platform
        printProgressBar(1, 2, "Starting M2");

        if (runM2Sequence()){
          printProgressBar(2, 2, "M2 Complete");
          Serial.println("M2 Sequence Complete");
        } else {
          Serial.println("M2 Sequence Failed");
          setSystemState(SYSTEM_SAFE);
          break;
        }

        Serial.println("");
        Serial.println("=== M1 + M2 SEQUENCE COMPLETE===");
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
      } else if (input == 'M' || input == 'm'){
        // Motor Speed Menu
        Serial.println("");
        Serial.println("=== MOTOR SPEED ADJUSTMENT ===");
        Serial.println("Current Speeds:");
        Serial.println("  M1  (Plunger): "); Serial.println(m1Speed);
        Serial.println("  M2 (Platform): "); Serial.println(m2Speed);
        Serial.println("  M3 (Conveyor): "); Serial.println(m3Speed);
        Serial.println("  M4 (The Belt): "); Serial.println(abs(m4Speed));
        Serial.println("");
        Serial.println("NOTE: Speeds are set in AG_MOTORS.cpp");
        Serial.println("Modify and reupload to change speeds.");
        delay(3000);
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
        Serial.println("╔════════════════════════════════════════╗");
        Serial.println("║         CURRENT MOTOR POSITIONS        ║");
        Serial.println("╠════════════════════════════════════════╣");
  
        Serial.print("║ M1 (Plunger):   ");
        long m1Pos = getM1Position();
        if (m1Pos >= 0) Serial.print(" ");
        Serial.print(m1Pos);
        Serial.print(" counts");
  
        // Padding
        int m1Digits = String(abs(m1Pos)).length();
        if (m1Pos < 0) m1Digits++;
        for (int i = 0; i < (15 - m1Digits); i++) Serial.print(" ");
        Serial.println("║");
  
        Serial.print("║ M2 (Platform):  ");
        long m2Pos = getM2Position();
        if (m2Pos >= 0) Serial.print(" ");
        Serial.print(m2Pos);
        Serial.print(" counts");
  
        int m2Digits = String(abs(m2Pos)).length();
        if (m2Pos < 0) m2Digits++;
        for (int i = 0; i < (15 - m2Digits); i++) Serial.print(" ");
        Serial.println("║");
  
        Serial.print("║ M3 (Conveyor):  ");
        long m3Pos = getM3Position();
        if (m3Pos >= 0) Serial.print(" ");
        Serial.print(m3Pos);
        Serial.print(" counts");
  
        int m3Digits = String(abs(m3Pos)).length();
        if (m3Pos < 0) m3Digits++;
        for (int i = 0; i < (15 - m3Digits); i++) Serial.print(" ");
        Serial.println("║");
  
        Serial.println("╚════════════════════════════════════════╝");
        Serial.println("");
        break;

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
      } else if (input == 'V' || input == 'v'){
        PRINT_PROGMEM("");
        PRINT_PROGMEM("╔════════════════════════════════════════╗");
        PRINT_PROGMEM("║           ASTROGLASS PROJECT           ║");
        PRINT_PROGMEM("║      Lunar Regolith Glassblowing       ║");
        PRINT_PROGMEM("╠════════════════════════════════════════╣");
        PRINT_PROGMEM("║            VERSION: 2.0.1              ║");
        PRINT_PROGMEM("╚════════════════════════════════════════╝");
        PRINT_PROGMEM("");
        delay(2000);
        break;
      
      // ========== CREDITS ========== //
      } else if (input == '?'){
        PRINT_PROGMEM("");
        PRINT_PROGMEM("╔════════════════════════════════════════╗");
        PRINT_PROGMEM("║           ASTROGLASS PROJECT           ║");
        PRINT_PROGMEM("║      Lunar Regolith Glassblowing       ║");
        PRINT_PROGMEM("╠════════════════════════════════════════╣");
        PRINT_PROGMEM("║             TEAM MEMBERS               ║");
        PRINT_PROGMEM("╠════════════════════════════════════════╣");
        PRINT_PROGMEM("║  • Braddley Dobbins                    ║");
        PRINT_PROGMEM("║  • Aidan O'Brien-Turner                ║");
        PRINT_PROGMEM("║  • Charles Nelson                      ║");
        PRINT_PROGMEM("║  • Jordan Sziegeti-Larenne             ║");
        PRINT_PROGMEM("║  • Robby Warkentine                    ║");
        PRINT_PROGMEM("╠════════════════════════════════════════╣");
        PRINT_PROGMEM("║  Author: Pedro Ortiz                   ║");
        PRINT_PROGMEM("╚════════════════════════════════════════╝");
        PRINT_PROGMEM("");
        delay(5000);
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


