#include <DualVNH5019MotorShield.h>

DualVNH5019MotorShield md;

// ===== Encoder Pins ===== //
const int pinA = 2;   // CLK - Interrupt pin (Channel A)
const int pinB = 5;   // DT  - Regular digital pin (Channel B)
volatile long position = 0;
volatile int prevA = LOW;

// ===== Constants ===== //
const long countsPerRev = 50;     // Adjust for the motor rotaion
const int m3DownSpeed   = 100;    // Speed for M3 to lower
const int m3UpSpeed     = 100;    // Speed for M3 to raise
const int m4Speed       = 100;    // Speed for M4 to move the belt

// ===== REFERENCE SPEED TABLE (-400 to +400, with Duty Percentage) ===== //
/*
  -400: Full (MAX) Power Reverse      (100%)
  -300: High Reverse Speed            (75%)
  -200: Medium Reverse Speed          (50%)
  -100: Slow Reverse Speed            (25%)
   -50: Extremely Slow Reverse Speed  (12.5%)
     0: No Speed (IDLE)               (0%)
    50: Extremely Slow Forward Speed  (12.5%)
   100: Slow Forward Speed            (25%)
   200: Medium Forward Speed          (50%)
   300: High Forward Speed            (75%)
   400: Full (MAX) Power Forward      (100%)
*/

// ===== Function Declarations ===== //
void updateEncoder();
void moveM3Down(long targetCount, int speed);
void moveM3Up(long targetCount, int speed);
void runMotor4Cont(int speed);
void stopMotor(int motorNumber);

// ====== Function Definitions ====== //

// Encoder Interrupt Service Routine
void updateEncoder(){
  int currentA = digitalRead(pinA);
  int currentB = digitalRead(pinB);

  if (prevA == LOW && currentA == HIGH) {
    if (currentB == LOW) position++;
    else position--;
  }
  prevA = currentA;
}

// Move Motor Down (M3)
void moveM3Down(long targetCount, int speed) {
  position = 0;
  md.setM2Speed(-speed);
  Serial.println("M3 lowering...");

  while (abs(position) < targetCount) {
    Serial.print("Encoder count: ");
    Serial.println(position);
    delay(10);
  }

  stopMotor(3);
  Serial.println("M3 reached lower position.");
}

// Move Motor Up (M3)
void moveM3Up(long targetCount, int speed) {
  position = 0;
  md.setM2Speed(speed);
  Serial.println("M3 raising...");

  while (abs(position) < targetCount) {
    Serial.print("Encoder count: ");
    Serial.println(position);
    delay(10);
  }

  stopMotor(3);
  Serial.println("M3 returned to start position.");
}

// Stop Motor
void stopMotor(int motorNumber) {
  if (motorNumber == 3) {
    md.setM2Speed(0);
    Serial.println("Motor 3 stopped.");
  } 
  else if (motorNumber == 4) {
    md.setM1Speed(0);
    Serial.println("Motor 4 stopped.");
  } 
  else {
    Serial.println("Invalid motor number! Use 3 or 4.");
  }
}

// Run Motor 4 Continuously
void runM4Cont(int speed) {
  md.setM1Speed(speed);
  Serial.print("Motor 4 running continuously at speed ");
  Serial.println(speed);
}

// ===== Full Sequence in Action ===== //
void setup(){
  Serial.begin(115200);
  md.init();    // Initialize Motor Driver

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), updateEncoder, CHANGE);

  Serial.println("=== Test Sequence: M3 Down, M4 Run,  M3 Up, Stop ===");
  delay(1000);
}

void loop(){
  // Wait for the user to type "START"
  Serial.println("Type 'START' to activate sequence...");
  while (true) {
    if (Serial.available() > 0){  // If something was typed in
      String input = Serial.readStringUntil('\n');  // Reads until 'Enter' key
      input.trim();  // Removes the space and/or newline
      if (input.equalsIgnoreCase("START")){
        Serial.println("STARTING SEQUENCE");
        break;  // Exits the waiting loop and continues
      } else {
        Serial.println("Not a correct input, type 'START' to run the sequence.");
      }
    }
  }

  position = 0;  // Reset encoder position

  // M3 lowers down
  moveM3Down(countsPerRev, m3DownSpeed);
  delay(2000);  // 2 sec

  // M4 runs continuously for 5 seconds
  runM4Cont(m4Speed);
  delay(5000);  // 5 sec
  stopMotor(4);
  delay(2000);  // 2 sec

  // M3 raises back up
  moveM3Up(countsPerRev, m3UpSpeed);
  delay(2000);  // 2 sec

  // Stop all motors
  stopMotor(3);
  stopMotor(4);

  Serial.println("=== SEQUENCE COMPLETE ===");
  while (true);  // Stops program after one full sequence
}

