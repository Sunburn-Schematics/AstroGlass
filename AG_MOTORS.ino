#include <EEPROM.h>
#include "Astro_Motors.h"
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

void loop() {
  if (!waitForRun()) {
    delay(250);
    return;
  }

  position = 0;

  // M3 lowers down
  if (!moveM3ToPos(CONST_90_DEG, -m3DownSpeed, "Lowering")) {
    emergencyStop();
  }

  delay(DELAY_AFTER_DOWN);

  // M4 runs continuously
  runM4Cont(m4Speed);
  delay(M4_RUN_TIME);
  stopMotor(Motor::M4);
  delay(DELAY_AFTER_M4);

  // M3 raises back up
  if (!moveM3ToPos(CONST_90_DEG, m3UpSpeed, "Raising")) {
    emergencyStop();
  }

  delay(DELAY_AFTER_UP);

  // Stop all motors
  stopMotor(Motor::M3);
  stopMotor(Motor::M4);

  savedEncoderPos();

  Serial.println("=== SEQUENCE COMPLETE ===");
  delay(SEQUENCE_PAUSE);
}

// FOR PERSONAL //
/*
Updating GITHUB:
  - git status
  - git add .
  - git commit -m "Enter any comments"
  - git push
*/