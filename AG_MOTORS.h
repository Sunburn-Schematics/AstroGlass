// ============================================================= //
// PROJECT:   AstroGlass Control System
// PLATFORM:  Arduino MEGA 2560
// DRIVER:    x2 DualVNH5019 Motor Shield
// MOTOR(S):  x3 Maverick 12V DC Gear Motor w/Encoder (61:1),
//            x1 Maverick Planetary DC Gear Motor w/Encoder (3.7:1)
// AUTHOR:    Pedro Ortiz
// VERSION:   h1.0.2
// ============================================================= //

#ifndef AG_MOTORS_H
#define AG_MOTORS_H

#include <EEPROM.h>
#include <Arduino.h>
#include <DualVNH5019MotorShield.h>

// ======================== MOTOR SPEEDS ====================== //
extern const int m1Speed;  // PWM speed (0-255)
extern const int m2Speed;  // PWM speed (0-255)
extern const int m3Speed;  // Shield speed (-400-400)
extern const int m4Speed;  // Shield speed (-400-400)

// ==================== SHIELD 2 CONTROL PINS ================= //
// M1 Plunger Motor
extern const int M1_INA;  // M1 Direction A
extern const int M1_INB;  // M1 Direction B
extern const int M1_PWM;  // M1 PWM Speed Control

// M2 Platform Motor
extern const int M2_INA;  // M2 Direction A
extern const int M2_INB;  // M2 Direction B
extern const int M2_PWM;  // M2 PWM Speed Control

// ======================= ENCODER PINS ======================= //
// M1 Plunger Encoder
extern const int m1PinA;  // Encoder Channel A (interrupt)
extern const int m1PinB;  // Encoder Channel B (interrupt)
extern volatile int m1PrevA;
extern volatile int m1PrevB;

// M2 Platform Encoder
extern const int m2PinA;  // Encoder Channel A (interrupt)
extern const int m2PinB;  // Encoder Channel B (interrupt)
extern volatile int m2PrevA;
extern volatile int m2PrevB;

// M3 Conveyor Encoder
extern const int m3PinA;  // Encoder Channel A (interrupt)
extern const int m3PinB;  // Encoder Channel B (interrupt)
extern volatile int m3PrevA;
extern volatile int m3PrevB;

// ============== MOTOR-SPECIFIC PARAMETERS =================== //
// M1 Plunger Parameters
extern const int M1_POSITION_TOLERANCE;  // Acceptable position error (counts)
extern const long M1_EXTEND_COUNTS;      // Extension distance: 50 rotations
extern const long M1_HOLD_TIME;          // Compression hold time (milliseconds)

// M2 Platform Parameters
extern const int M2_POSITION_TOLERANCE;  // Acceptable position error (counts)
extern const long M2_LOWER_COUNTS;         // Lowering distance: 92 rotations

// M3 Conveyor Parameters
extern const int M3_POSITION_TOLERANCE;  // Acceptable position error (counts)
extern const long SAFE_POS_COUNTS;       // Home/safe position
extern const long CONST_90_DEG;          // 90-degree movement (1/4 revolution)
extern const long M3_MAX_VALID_POS;      // Maximum valid EEPROM position
extern const long M3_MIN_VALID_POS;      // Minimum valid EEPROM position
extern const int SAFE_SPEED;             // Speed for returning to safe position

// Extra Parameters
extern const long countsPerRev;    // Maverick: 7 PPR × 2 edges × 61:1 gear
extern const long m1CountsPerRev;  // Planetary: 7 PPR × 2 edges × 3.7:1 gear

// ===================== TIMING PARAMETERS ==================== //
extern const unsigned long MOTOR_TIMEOUT_MOVE;  // M3 movement timeout (15s)
extern const unsigned long MOTOR_TIMEOUT_HOME;  // M3 homing timeout (15s)
extern const unsigned long M1_TIMEOUT;          // M1 movement timeout (75s)
extern const unsigned long M2_TIMEOUT;          // M2 movement timeout (120s)
extern const unsigned long M4_TOTAL_RUN_TIME;   // M4 total run duration (7s)

// ================ EEPROM CONFIGURATION ====================== //
extern const int M1_EEPROM_ADDR_POS;        // EEPROM storage address
extern const int M2_EEPROM_ADDR_POS;        // EEPROM storage address
extern const int M3_EEPROM_ADDR_POS;        // EEPROM storage address
extern const int EEPROM_SYSTEM_STATE_ADDR;  // System state flag address
extern const byte SYSTEM_RUNNING;           // Magic byte: system active
extern const byte SYSTEM_SAFE;              // Magic byte: system safe

// ==================== MOTOR POSITIONS ======================= //
extern volatile long m1Position;  // M1 current encoder position
extern volatile long m2Position;  // M2 current encoder position
extern volatile long m3Position;  // M3 current encoder position

// ================= MOTOR SHIELD OBJECT ====================== //
extern DualVNH5019MotorShield md;

// ===================== PAUSE/RESUME ========================= //
extern volatile bool systemPaused;

// =================== FUNCTION DECLARATIONS ================== //
// System Functions
void initializeMotors();
bool checkMotorFaults();
void clearMotorFaults();
bool checkEmergencyStop();
void setSystemState(byte state);
void clearSerialInput();
bool checkPauseResume();
bool allMotorsToSafePos();
void stopMotor(int motorNum);
void stopAllMotors();
void emergencyStop();
bool validateAndFixEEPROM();

// M1 Functions
void updateM1Encoder();
long getM1Position();
void savedM1EncoderPos();
long loadM1EncoderPos();
void setM1Direction(int dir);
bool runM1Sequence();
bool moveM1ToPosition(long targetPosition, unsigned long timeout);
void testM1();

// M2 Functions
void updateM2Encoder();
long getM2Position();
void savedM2EncoderPos();
long loadM2EncoderPos();
void setM2Direction(int dir);
bool runM2Sequence();
bool moveM2ToPosition(long targetPosition, unsigned long timeout);
void testM2();

// M3 Functions
void updateM3Encoder();
long getM3Position();
void savedM3EncoderPos();
long loadM3EncoderPos();
void brakeM3();
bool moveM3ToPosition(long targetCount, unsigned long timeout);
void testM3();

// M4 Functions
void runM4Cont(int speed);
void testM4();

// ASCII Functions
void printProgressBar(int step, int totalSteps, const char* stepName);
void printStartUp();
void printMainMenu();

#endif




