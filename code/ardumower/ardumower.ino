/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  Copyright (c) 2013-2015 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri    
  Copyright (c) 2014-2015 by Stefan Manteuffel
  Copyright (c) 2015 by Uwe Zimprich
  Copyright (c) 2016-2017 by Reiner Ehlers
    
  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
*/


/* LMU uninitialised data */
StartOfUninitialised_LMURam_Variables
/* Put your LMU RAM fast access variables that have no initial values here e.g. uint32 LMU_var; */
EndOfUninitialised_LMURam_Variables

/* LMU uninitialised data */
StartOfInitialised_LMURam_Variables
/* Put your LMU RAM fast access variables that have an initial value here e.g. uint32 LMU_var_init = 1; */
EndOfInitialised_LMURam_Variables

/* If you do not care where variables end up, declare them here! */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "config.h"
#include "buzzer.h"
 


/***********************************/
/***           Core 0            ***/
/***********************************/
void setup()
{

  // Set Arduino PWM Frequency about 1,5kHz
  useArduinoPwmFreq();
  
  // Initialise EEPROM system
  if(EEPROM.eeprom_initialise() == EEPROM_Not_Initialised)
  {
    // EEPROM is bad 
    // TODO: Timeout einbauen
    while(1) { ; }
  }
 
  robot.setup(); 
}


void loop()
{
  robot.loop();    
}



/***********************************/
/***           Core 1            ***/
/***********************************/

/* CPU1 Uninitialised Data */
StartOfUninitialised_CPU1_Variables
/* Put your CPU1 fast access variables that have no initial values here e.g. uint32 CPU1_var; */
EndOfUninitialised_CPU1_Variables
/* CPU1 Initialised Data */
StartOfInitialised_CPU1_Variables
/* Put your CPU1 fast access variables that have an initial value here e.g. uint32 CPU1_var_init = 1; */
EndOfInitialised_CPU1_Variables

void setup1()
{
  Beep_begin();
}

void loop1() 
{  
  
}



/***********************************/
/***           Core 2            ***/
/***********************************/

/* CPU2 Uninitialised Data */
StartOfUninitialised_CPU2_Variables
/* Put your CPU2 fast access variables that have no initial values here e.g. uint32 CPU2_var; */
EndOfUninitialised_CPU2_Variables
/* CPU2 Initialised Data */
StartOfInitialised_CPU2_Variables
/* Put your CPU2 fast access variables that have an initial value here e.g. uint32 CPU2_var_init = 1; */
EndOfInitialised_CPU2_Variables


void setup2()
{
  // put your setup code for core 2 here, to run once:
}

void loop2()
{
  // put your main code for core 2 here, to run repeatedly:
}



