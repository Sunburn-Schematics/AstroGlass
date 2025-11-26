# AstroGlass Motor Control System

**Author:** Pedro Ortiz  
**Version:** v1.6.82

**Last Update:** 11/26/25 at 11:58a

---

## OVERVIEW
This repository holds the Arduino code used to control the four-motor subsystem of **FRRED - Freeform Robotic Regolith Engineering Device**. The primary file, **AG_MOTORS.ino**, includes all logic for coordinated movements, encoder tracking, emergency-handling, and safe-position homing.

If you're wondering which file is the correct one, it's this one. You're welcome.

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
- **(x4)** Maverick **12V DC** Gear Motors w/ Encoders  
- Emergency-Stop Button  
- Supporting power + wiring hardware  

---

## FILES
- **AG_MOTORS.ino** — Full control system
- **AG_MOTORS.h** — Coming Soon  
- **AG_MOTORS.cpp** — Coming Soon  

---

## OPERATING THE SYSTEM
1. Open the `.ino` file in **Arduino IDE**  
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
- **[R]** → Reset all motor positions to zero
- **[S]** → Show all current motor positions
- **[?]** → Team Credits 

---

## NOTES
- "Without Music, Life Would Be A Mistake."
