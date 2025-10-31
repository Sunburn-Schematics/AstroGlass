#include <EEPROM.h>                 // Safe Positons
#include <DualVNH5019MotorShield.h> // Motor Shield Built-In Library

DualVNH5019MotorShield md;

// ===== Encoder Pins ===== //
const int pinA                = 2;      // CLK - Interrupt pin (Channel A)
const int pinB                = 5;      // DT  - Regular digital pin (Channel B)
volatile int prevA            = LOW;

// ===== Constants ===== //
const long SAFE_POS_COUNTS    = 0;      // Encoder count for home/safe zone
const long HOME_OFFSET_COUNTS = 150;    // Small offset to ensure it's settled
const long countsPerRev       = 50;     // Adjust for the motor rotaion
const int m3DownSpeed         = 100;    // Speed for M3 to lower
const int m3UpSpeed           = 100;    // Speed for M3 to raise
const int m4Speed             = 100;    // Speed for M4 to move the belt
const int EEPROM_ADDR_POS     = 0;      // Where encoder count is stored
const int SAFE_SPEED          = 150;    // Speed to reach home
volatile long position        = 0;      // Motor Position


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
void updateEncoder();                           // Encoder Interrupt Service Routine
bool waitForStart();                            // Waits For User To Input START
void moveM3Down(long targetCount, int speed);   // Move Motor Down (M3)
void moveM3Up(long targetCount, int speed);     // Move Motor Up (M3)
void runMotor4Cont(int speed);                  // Run Motor 4 Continuously
void stopMotor(int motorNumber);                // Stop Individual Motor        
void savedEncoderPos();                         // Saves The Encoder Position To EEPROM
long loadEncoderPos();                          // Load The Encoder Position From EEPROM
void goToSafePos();                             // Returns The Motors To The Safe Position

// ====== Function Definitions ====== //

void updateEncoder(){
  int currentA = digitalRead(pinA);
  int currentB = digitalRead(pinB);

  if (prevA == LOW && currentA == HIGH){
    if (currentB == LOW) position++;
    else position--;
  }
  prevA = currentA;
}

bool waitForStart(){
  Serial.println("Type 'START' to activate the sequence or 'STOP' to abort...");

  while (true){
    if (Serial.available() > 0){  // If something was typed in
      String input = Serial.readStringUntil('\n');  // Reads until 'Enter' key
      input.trim();  // Removes the space and/or newline

      if (input.equalsIgnoreCase("START")){
        Serial.println("STARTING SEQUENCE");
        return true;  // Continues to main loop
      } else if (input.equalsIgnoreCase("STOP")){
        Serial.println("ABORTING SEQUENCE");
        return false;
      } else {
        Serial.println("Wrong input dude, type 'START' or 'STOP'.");
      }
    }
  }
}

void moveM3Down(long targetCount, int speed){
  position = 0;
  md.setM2Speed(-speed);
  Serial.println("M3 Lowering...");

  while (abs(position) < targetCount){
    Serial.print("Encoder Count: ");
    Serial.println(position);
    delay(10);
  }

  stopMotor(3);
  Serial.println("M3 Reached Lower Position.");
}

void moveM3Up(long targetCount, int speed){
  position = 0;
  md.setM2Speed(speed);
  Serial.println("M3 Raising...");

  while (abs(position) < targetCount){
    Serial.print("Encoder Count: ");
    Serial.println(position);
    delay(10);
  }

  stopMotor(3);
  Serial.println("M3 Returned To Start Position.");
}

void runM4Cont(int speed){
  md.setM1Speed(speed);
  Serial.print("Motor 4 Running Continuously At Speed ");
  Serial.println(speed);
}

void stopMotor(int motorNumber){
  if (motorNumber == 3){
    md.setM2Speed(0);
    savedEncoderPos();
    Serial.println("Motor 3 Stopped And Position Saved.");
  } else if (motorNumber == 4){
    md.setM1Speed(0);
    Serial.println("Motor 4 Stopped.");
  } else {
    Serial.println("Invalid Motor number (For Now), Use Either 3 or 4.");
  }
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

void goToSafePos(){
  long savedPos = loadEncoderPos();   // Reads the last saved encoder value
  position = savedPos;                     // Initialize the position tracking

  Serial.println("Moving M3 To Safe Position...");

  if (position > SAFE_POS_COUNTS){
    md.setM2Speed(-SAFE_SPEED);
    while (position > SAFE_POS_COUNTS + HOME_OFFSET_COUNTS){
      Serial.print("Safe Homing... Encoder: ");
      Serial.println(position);
      delay(10);
    }
  } else if (position < SAFE_POS_COUNTS){
    md.setM2Speed(SAFE_SPEED);
    while (position < SAFE_POS_COUNTS - HOME_OFFSET_COUNTS){
      Serial.print("Safe Homing... Encoder: ");
      Serial.println(position);
      delay(10);
    }
  }

  md.setM2Speed(0);
  position = SAFE_POS_COUNTS;
  savedEncoderPos();
  Serial.println("Gentlemen, We Have landed In The Safe Position");
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
  // If STOP was entered, it does nothing
  if (!waitForStart()){
    Serial.println("SYSTEM IDLE");
    while (true);  // Freezes "safely"
  }

  goToSafePos();
  delay(1000);

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


// FOR PEROSNAL //
/*
Updating GITHUB:
  - git status
  - git add .
  - git commit -m "Enter any comments"
  - git push
*/



