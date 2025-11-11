#include <EEPROM.h>
#include "Astro_Motors.h"
#include <DualVNH5019MotorShield.h>

void setup() {
  Serial.begin(115200);
  delay(500);
  
  // Initialize motor shield and encoder
  initializeMotors();

  Serial.println("AstroGlass Motor Control System");
  Serial.println("Initializing...");
  Serial.println("");
  
  // Clear EEPROM and reset position on every boot
  Serial.println("Resetting EEPROM to 0...");
  position = 0;
  EEPROM.put(EEPROM_ADDR_POS, position);
  Serial.println("EEPROM cleared.");
  Serial.println("");
  
  Serial.println("System Ready");
  Serial.println("Encoder on Pins 18 & 19");
  Serial.println("Position reset to 0");
  Serial.println("");
  Serial.println("IMPORTANT: Ensure M3 is at safe position (UP) before running.");
  Serial.println("");
  
  delay(2000);
}

void loop() {
  // Wait for user to press SPACE to start
  if (!waitForRun()){
    delay(250);
    return;
  }
  
  position = 0;   // Force encoder position to 0 at start of every sequence
  
  Serial.println("");
  Serial.println("Starting Sequence...");
  Serial.println("");
  Serial.print("Initial position: ");
  Serial.println(getPosition());
  Serial.println("");

  // STEP 1: Lower M3 conveyor 90 degrees
  Serial.println("STEP 1: Lowering M3 (target: 214 counts)...");
  
  md_main.setM2Speed(-m3DownSpeed);
  unsigned long startTime = millis();
  unsigned long lastPrint = 0;
  
  // Wait until encoder reaches target count
  while (abs(getPosition()) < CONST_90_DEG){
    // Check for timeout
    if (millis() - startTime > MOTOR_TIMEOUT_MOVE){
      Serial.println("");
      Serial.println("ERROR: M3 lowering timeout!");
      Serial.print("Reached: ");
      Serial.print(getPosition());
      Serial.print(" / Target: ");
      Serial.println(CONST_90_DEG);
      Serial.println("");
      Serial.println("Problem: Encoder not counting or motor not moving.");
      Serial.println("Check wiring. [Run MOTOR_ENCODER_TEST, if needed].");
      md_main.setM2Speed(0);
      while(true);  // Halt
    }
    
    // Print progress every 200ms
    if (millis() - lastPrint > 200){
      Serial.print("  Count: ");
      Serial.println(getPosition());
      lastPrint = millis();
    }
    
    delay(10);
  }
  
  // Stop motor
  md_main.setM2Speed(0);
  Serial.println("");
  Serial.print("M3 lowering complete. Final count: ");
  Serial.println(getPosition());
  Serial.println("");
  delay(DELAY_AFTER_DOWN);

  // STEP 2: Run M4 belt motor
  Serial.println("STEP 2: Running M4 belt...");
  md_main.setM1Speed(m4Speed);
  delay(M4_RUN_TIME);
  md_main.setM1Speed(0);
  Serial.println("M4 stopped.");
  Serial.println("");
  delay(DELAY_AFTER_M4);

  // STEP 3: Raise M3 conveyor 90 degrees
  Serial.println("STEP 3: Raising M3 (target: 214 counts)...");
  
  // Reset encoder for relative movement
  position = 0;
  
  md_main.setM2Speed(m3UpSpeed);
  startTime = millis();
  lastPrint = 0;
  
  // Wait until encoder reaches target count
  while (abs(getPosition()) < CONST_90_DEG) {
    // Check for timeout
    if (millis() - startTime > MOTOR_TIMEOUT_MOVE) {
      Serial.println("");
      Serial.println("ERROR: M3 raising timeout!");
      Serial.print("Reached: ");
      Serial.print(getPosition());
      Serial.print(" / Target: ");
      Serial.println(CONST_90_DEG);
      md_main.setM2Speed(0);
      while(true);  // Halt
    }
    
    // Print progress every 200ms
    if (millis() - lastPrint > 200) {
      Serial.print("  Count: ");
      Serial.println(getPosition());
      lastPrint = millis();
    }
    
    delay(10);
  }
  
  // Stop motor
  md_main.setM2Speed(0);
  Serial.println("");
  Serial.print("M3 raising complete. Final count: ");
  Serial.println(getPosition());
  Serial.println("");
  delay(DELAY_AFTER_UP);

  // Stop all motors and save position
  md_main.setM2Speed(0);
  md_main.setM1Speed(0);
  
  // Reset encoder and save to EEPROM
  position = 0;
  EEPROM.put(EEPROM_ADDR_POS, position);
  
  Serial.println("Sequence Complete");
  Serial.println("Encoder reset to 0 and saved.");
  Serial.println("Ready for next cycle.");
  Serial.println("");
  
  delay(SEQUENCE_PAUSE);
}



