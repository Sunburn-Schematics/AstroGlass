#include "Astro_Motors.h"
#include <EEPROM.h>

// void updateEncoder(){
//   int currentA = digitalRead(pinA);
//   int currentB = digitalRead(pinB);

//   if (prevA == LOW && currentA == HIGH){
//     if (currentB == LOW) position++;
//     else position--;
//   }
//   prevA = currentA;
// }

// void savedEncoderPos(){
//   noInterrupts();
//   long tempPos = position;
//   interrupts();
//   EEPROM.put(EEPROM_ADDR_POS, tempPos);
//   Serial.print("Saved Encoder Position to EEPROM: ");
//   Serial.println(tempPos);
// }

// long loadEncoderPos(){
//   long savedPos;
//   EEPROM.get(EEPROM_ADDR_POS, savedPos);
//   Serial.print("Loaded Encoder Position from EEPROM: ");
//   Serial.println(savedPos);
//   return savedPos;
// }

// long getPosition(){
//   noInterrupts();   // Stops all interrupts temporarily
//   long pos = position;  // Safely read the value
//   interrupts();   // Re-enable interrupts
//   return pos;
// }

// bool waitForRun(){
//   clearSerialInput();
//   Serial.println("Press [SPACE] to start or 'S' to stop.");

//   while (true){
//     if (Serial.available() > 0){
//       char input = Serial.read();

//       if (input == ' '){
//         Serial.println("Starting the sequence...");
//         return true;
//       } else if (input == 'S' || input == 's'){
//         Serial.println("System idle.");
//         delay(5000);
//         return false;
//       }
//     }
//   }
// }

// void runM4Cont(int speed){
//   md.setM1Speed(speed);
//   Serial.print("Motor 4 Running Continuously At Speed ");
//   Serial.println(speed);
// }

// void stopMotor(int motorNumber){
//   if (motorNumber == 3){
//     md.setM2Speed(0);
//     savedEncoderPos();
//     Serial.println("Motor 3 Stopped And Position Saved.");
//   } else if (motorNumber == 4){
//     md.setM1Speed(0);
//     Serial.println("Motor 4 Stopped.");
//   } else {
//     Serial.println("Invalid Motor number (For Now), Use Either 3 or 4.");
//   }
// }

// void goToSafePos(){
//   long savedPos = loadEncoderPos();   // Reads the last saved encoder value
//   position = savedPos;                     // Initialize the position tracking

//   Serial.println("Moving M3 To Safe Position...");

//   if (position > SAFE_POS_COUNTS){
//     md.setM2Speed(-SAFE_SPEED);
//     while (position > SAFE_POS_COUNTS + HOME_OFFSET_COUNTS){
//       Serial.print("Safe Homing... Encoder: ");
//       Serial.println(position);
//       delay(10);
//     }
//   } else if (position < SAFE_POS_COUNTS){
//     md.setM2Speed(SAFE_SPEED);
//     while (position < SAFE_POS_COUNTS - HOME_OFFSET_COUNTS){
//       Serial.print("Safe Homing... Encoder: ");
//       Serial.println(position);
//       delay(10);
//     }
//   }

//   md.setM2Speed(0);
//   position = SAFE_POS_COUNTS;
//   savedEncoderPos();
//   Serial.println("Gentlemen, We Have landed In The Safe Position");
// }

// void clearSerialInput(){
//   while (Serial.available() > 0){
//     Serial.read();
//   }
// }