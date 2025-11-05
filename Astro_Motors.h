#ifndef ASTRO_MOTORS_H
#define ASTRO_MOTORS_H

#include <EEPROM.h>                 // Safe Positons
#include <DualVNH5019MotorShield.h> // Motor Shield Built-In Library

extern DualVNH5019MotorShield md;

// void updateEncoder();                           // Encoder Interrupt Service Routine
// void savedEncoderPos();                         // Saves The Encoder Position To EEPROM
// long loadEncoderPos();                          // Load The Encoder Position From EEPROM
// long getPosition();                             // 
// bool waitForRun();                              // Waits for input activation, then asks after sequence
// void moveM3Down(long targetCount, int speed);   // Move Motor Down (M3)
// void moveM3Up(long targetCount, int speed);     // Move Motor Up (M3)
// void runMotor4Cont(int speed);                  // Run Motor 4 Continuously
// void stopMotor(int motorNumber);                // Stop Individual Motor        
// void goToSafePos();                             // Returns The Motors To The Safe Position
// void clearSerialInput();                        // Clears the serial input

#endif