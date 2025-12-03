# AstroGlass Motor Control System

**Author:** Pedro Ortiz  
**Version:** 2.0.3

**Last Update:** 12/03/25 at 12:02p

---

## OVERVIEW
This repository holds the Arduino code used to control the four-motor subsystem of **FRRED - Freeform Robotic Regolith Engineering Device**. The primary file, **AG_MOTORS.ino**, includes all logic for coordinated movements, encoder tracking, emergency-handling, and safe-position homing.

---

## WHAT THE CODE DOES
- Controls **4 DC motors**: M1 (Plunger), M2 (Platform), M3 (Conveyor), M4 (Belt)  
- Uses encoders for real-time position feedback  
- Handles **fault detection** for Dual VNH5019 motor drivers  
- Implements a full **automated motion sequence**  
- Using EEPROM, stores **state + encoder position**
- Has closed-loop **[P Controller]** integration to arrive "exactly" at the counts

---

## HARDWARE USED
- Arduino **Mega 2560**  
- **(x2)** Dual VNH5019 Motor Shield  
- **(x3)** Maverick **12V DC** Gear Motors w/ Encoders (61:1)
- **(x1)** Maverick Planetary DC Gear Motor w/Encoder (3.7:1)
- Emergency-Stop Button  
- Supporting power + wiring hardware  

---

## FILES
- **AG_MOTORS.ino** - Full control system
- **AG_MOTORS.h** - All declared functions and constants
- **AG_MOTORS.cpp** -  All funnction definitions
- **README.md** - What **YOU** are currently reading

---

## OPERATING THE SYSTEM
1. Open the `.ino`, '.h', and '.cpp' file in **Arduino IDE**  
2. Select **Board:** Arduino Mega 2560  
3. Select the correct **COM** port  
4. Upload code  
5. Open the Serial Monitor (`115200`) to view the prompts/commands  

---

## COMMANDS
- **[1]-[5]** → Run different parts of the sequence
- **[6]** → Run M1 and M2 sequence
- **[T1]-[T4]** → Test motors, 1–4
- **[H]** → Have all motors go to their home position
- **[M]** → Display the speed of motors
- **[P]** → Pause/Resume during sequence
- **[R]** → Reset all motor positions to zero
- **[S]** → Display all current motor positions
- **[V]** → Version of code
- **[?]** → Team Credits 

---

## NOTES
- "Without Music, Life Would Be A Mistake."

