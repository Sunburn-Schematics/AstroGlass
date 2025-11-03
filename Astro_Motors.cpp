#include "Astro_Motors.h"
#include <EEPROM.h>

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