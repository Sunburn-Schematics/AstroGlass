/**
Astro_Motors.h - Header File for AstroGlass/Sunburn's Motors

To make it simple, (for me), the header file declares all constants, variables, and functions used in to motor control 
system. It provides the interface between both the main program and the motor control implementation.

Motor Assignments:
  M1 - Plunger
  M2 - Platform
  M3 - Conveyor Lift
  M4 - Belt
[Motors used: 4x Studia Maverick 7 PPR with a 61:1 gear ratio (854 per revolution)]
[Motor Drivers used: 2x DualVNH5019 Motor Shield]
*/

#ifndef ASTRO_MOTORS_H
#define ASTRO_MOTORS_H

#include <Arduino.h>
#include <EEPROM.h>
#include <DualVNH5019MotorShield.h>

// ===== HARDWARE CONFIGURATION ===== //
extern const int pinA;      // Encoder Channel A (Blue)
extern const int pinB;      // Encoder Channel B (Yellow)
extern volatile int prevA;  // Previous state of Channel A

// ===== ENCODER & MOTOR PARAMETERS ===== //
extern const long SAFE_POS_COUNTS;  // Safe position = 0 when conveyor is fully raised
extern const long countsPerRev;     // 854 counts
extern const long CONST_90_DEG;     // 854/4 = 214 counts

// ===== MOTOR SPEEDS ===== //
extern const int m3DownSpeed;   // Speed to lower the conveyor's arm
extern const int m3UpSpeed;     // Speed to raise the conveyor's arm
extern const int m4Speed;       // Speed to run the conveyor belt
extern const int SAFE_SPEED;    // Speed to return to the safe position

// ===== TIMING PARAMETERS ===== //
extern const unsigned long MOTOR_TIMEOUT_MOVE;    // Maximum amount of time allowed for the normal M3 movements
extern const unsigned long MOTOR_TIMEOUT_HOME;    // Maximum amount of time allowed for the homing sequence
extern const unsigned long DELAY_AFTER_DOWN;      // Pauses after M3 has lowered
extern const unsigned long M4_RUN_TIME;           // Duration of the belt to run
extern const unsigned long DELAY_AFTER_M4;        // Delay after M4 stops
extern const unsigned long DELAY_AFTER_UP;        // Pauses after M3 has risen
extern const unsigned long SEQUENCE_PAUSE;        // Pauses between cycles

// ===== EEPROM CONFIGURATION ===== //
extern const int EEPROM_ADDR_POS;                 // Address 0: M3 position storage

// ===== MOTOR POSITION ===== //
extern volatile long position;                    // Current M3 encoder count

// ===== MOTOR ENUM ===== //
enum Motor {
  M1 = 1,   // External shield, plunger motor
  M2 = 2,   // External shield, platform motor
  M3 = 3,   // Main sheild, conveyor lift motor
  M4 = 4    // Main sheild, belt drive motor
};

// ===== MOTOR SHIELD ===== //
extern DualVNH5019MotorShield md_main;      // Controls the main motors (M3 and M4)
extern DualVNH5019MotorShield md_external;  // Controls the other motors (M1 and M2)

// ===== FUNCTION DECLARATIONS ===== //
void initializeMotors();                                                // Initialize motor system
void updateEncoder();                                                   // Encoder ISR
bool checkMotorFaults();                                                // Check for motor faults
void savedEncoderPos();                                                 // Save encoder position to EEPROM
long loadEncoderPos();                                                  // Load encoder position from EEPROM
long getPosition();                                                     // Get current encoder position safely
bool waitForRun();                                                      // Wait for user input to start

bool moveM1ToPos(long targetCount, int speed, const char* direction);   // Move M1 motor
bool moveM2ToPos(long targetCount, int speed, const char* direction);   // Move M2 motor

bool moveM3ToPos(long targetCount, int speed, const char* direction);   // Move M3 motor
void runM4Cont(int speed);                                              // Run M4 continuously
void stopMotor(int motorNumber);                                        // Stop individual motor
bool goToSafePos();                                                     // Return to safe position
void emergencyStop();                                                   // Emergency stop procedure
void clearSerialInput();                                                // Clear serial buffer

#endif