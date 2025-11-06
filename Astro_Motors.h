#ifndef ASTRO_MOTORS_H
#define ASTRO_MOTORS_H

#include <Arduino.h>
#include <EEPROM.h>
#include <DualVNH5019MotorShield.h>

// ===== HARDWARE CONFIGURATION ===== //
extern const int pinA;
extern const int pinB;
extern volatile int prevA;

// ===== ENCODER & MOTOR PARAMETERS ===== //
extern const long SAFE_POS_COUNTS;
extern const long countsPerRev;
extern const long CONST_90_DEG;

// ===== MOTOR SPEEDS ===== //
extern const int m3DownSpeed;
extern const int m3UpSpeed;
extern const int m4Speed;
extern const int SAFE_SPEED;

// ===== TIMING PARAMETERS ===== //
extern const unsigned long MOTOR_TIMEOUT_MOVE;
extern const unsigned long MOTOR_TIMEOUT_HOME;
extern const unsigned long DELAY_AFTER_DOWN;
extern const unsigned long M4_RUN_TIME;
extern const unsigned long DELAY_AFTER_M4;
extern const unsigned long DELAY_AFTER_UP;
extern const unsigned long SEQUENCE_PAUSE;

// ===== EEPROM CONFIGURATION ===== //
extern const int EEPROM_ADDR_POS;

// ===== MOTOR POSITION ===== //
extern volatile long position;

// ===== MOTOR ENUM ===== //
enum Motor {
  M1 = 1,
  M2 = 2,
  M3 = 3,
  M4 = 4
};

// ===== MOTOR SHIELD ===== //
extern DualVNH5019MotorShield md;

// ===== FUNCTION DECLARATIONS ===== //
void initializeMotors();                                        // Initialize motor system
void updateEncoder();                                           // Encoder ISR
bool checkMotorFaults();                                        // Check for motor faults
void savedEncoderPos();                                         // Save encoder position to EEPROM
long loadEncoderPos();                                          // Load encoder position from EEPROM
long getPosition();                                             // Get current encoder position safely
bool waitForRun();                                              // Wait for user input to start
bool moveM3ToPos(long targetCount, int speed, const char* direction);  // Move M3 motor
void runM4Cont(int speed);                                      // Run M4 continuously
void stopMotor(int motorNumber);                                // Stop individual motor
bool goToSafePos();                                             // Return to safe position
void emergencyStop();                                           // Emergency stop procedure
void clearSerialInput();                                        // Clear serial buffer

#endif