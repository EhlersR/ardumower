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

#include "buzzer.h"
#include "config.h"
#include <Arduino.h>

#define pinBuzzer 53 //Ehl

volatile boolean tone_pin_state = false;
volatile uint16_t dura = 0;
volatile bool BeepOn;
volatile long millisStart;
uint8_t count;

// ISR for Beep (Toggles pinBuzzer at 1kHz if BeepOn)
void IntBeep(void)
{ 
  if (BeepOn)
  {
    digitalWrite(pinBuzzer, tone_pin_state = !tone_pin_state); 
    if (millis() >= millisStart + dura)
      BeepOn = false;
  }
  else
  {
    BeepOn = false;
    digitalWrite(pinBuzzer, LOW);
  }
}

void Beep_begin()
{ 
  pinMode(pinBuzzer, OUTPUT);                
  digitalWrite(pinBuzzer, LOW);
  return;
} 

void MyBeep(uint16_t duration, float frequency)
{
  float interval;
  
  dura = duration;
  millisStart = millis();

  //Calculate Timer Interval
  interval = 1 / frequency *  50000000.0;
  
  CreateTimerInterrupt0_Core1(ContinuousTimerInterrupt, uint16_t(interval), IntBeep); 
  BeepOn = true;  
  return;
}


