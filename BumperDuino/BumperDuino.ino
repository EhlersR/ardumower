/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  Copyright (c) 2013-2015 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri    
  Copyright (c) 2015 by Uwe Zimprich
  Copyright (c) 2015 by Frederic Goddeeres
  Copyright (c) 2015 by Jürgen Lange
  
  
  Autor: Jürgen Lange
  Stand: 19.08.2015
  Version: 0.04 Testversion
  
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

//#include "Wtv020sd16p.h"
// Created by Diego J. Arevalo, August 6th, 2012.
// Released into the public domain.
int resetPin = 13;  // The pin number of the reset pin.
int clockPin = 11;  // The pin number of the clock pin.
int dataPin = 13;  // The pin number of the data pin.
int busyPin = 12;  // The pin number of the busy pin.

/*
Create an instance of the Wtv020sd16p class.
 1st parameter: Reset pin number.
 2nd parameter: Clock pin number.
 3rd parameter: Data pin number.
 4th parameter: Busy pin number.
 */
//Wtv020sd16p wtv020sd16p(resetPin,clockPin,dataPin,busyPin);

// These constants won't change.  They're used to give names
// to the pins used:
const int sensor1InPin = A0;  // Analog input pin that the MPX5010 is attached to
const int sensor2InPin = A1;  // Analog input pin that the MPX5010 is attached to
//const int analogOut1Pin = 9; // Analog output pin Bumper 1 (PWM)
//const int analogOut2Pin = 10; // Analog output pin Bumper 2 (PWM)
const int Bumper1OutPin = 9;
const int Bumper2OutPin = 10;
const int LEDCollision1 = 7;
const int LEDCollision2 = 8;
const int LEDActive = 13;
const int LEDFault = 16;
const int LEDok = 17;
bool LEDActiveState = LOW;             // ledState used to set the LED

int sensor1Diff = 0;  // = sensor1DiffSecond - sensor1DiffFirst
int sensor1DiffFirst = 0;        // value for zero Point
int sensor1DiffSecond = 0;        // value for touch Point
int sensor1MAP = 0;        // value output to the PWM (analog out)
int sensor1Trigger = 5;  // Sensor-Trigger-Level Sensor 1
int trigger1Counter = 0; // Trigger-Counter Sensor 1
byte sensor1State = 0; // Sensor-State Sensor 1 zero point or touch point
byte count1FirstRead = 0; // Read-Counter for zero point Sensor 1
byte count1SecondRead = 0; // Read-Counter for touch point Sensor 1
byte flagAdc0Read = 0; // Flag its Time to read ADC Sensor 1
int soundBumper1 = 0;

int sensor2Diff = 0;  // = sensor2DiffSecond - sensor2DiffFirst
int sensor2DiffFirst = 0;        // value for zero Point
int sensor2DiffSecond = 0;       // value for touch Point
int sensor2MAP = 0;        // value output to the PWM (analog out)
int sensor2Trigger = 3;  // Sensor-Trigger-Level Sensor 2
int trigger2Counter = 0;  // Trigger-Counter Sensor 2
byte sensor2State = 0;  // Sensor-State Sensor 2 zero point or touch point
byte count2FirstRead = 0;  // Read-Counter for zero point Sensor 2
byte count2SecondRead = 0;  // Read-Counter for touch point Sensor 2
byte flagAdc1Read = 0;  // Flag its Time to read ADC Sensor 2
int soundBumper2 = 2;

unsigned long previousMillis = 0;        // will store last time Timer was updated
const long interval = 15;           // interval at which ADC was read (milliseconds)
const long intervalLedActive = 250; // interval at which LED was blink (milliseconds)
unsigned long previousMillisLedActive = 0;

//Ehl
unsigned long nextTimeCalcZero = 0;

unsigned long playJokeSoundMillis = 0;
unsigned long playNextJokeSound = 60000;
int jokeSound1 = 3;
//====================================================================================
//================================== SETUP ==========================================
void setup() 
{
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  // setup Ports
  pinMode(LEDCollision1, OUTPUT);
  pinMode(LEDCollision2, OUTPUT);
  pinMode(LEDActive, OUTPUT);
  pinMode(LEDFault, OUTPUT);
  pinMode(LEDok, OUTPUT);
  pinMode(Bumper1OutPin, OUTPUT);
  pinMode(Bumper2OutPin, OUTPUT);
  digitalWrite(LEDCollision1,HIGH);
  digitalWrite(LEDCollision2,HIGH);
  digitalWrite(LEDActive,HIGH);
  digitalWrite(LEDFault,HIGH);
  digitalWrite(LEDok,HIGH);
  delay(2000);
  digitalWrite(LEDActive,LOW);
  digitalWrite(LEDFault,LOW);
  digitalWrite(LEDok,LOW);
  digitalWrite(LEDCollision1,LOW);
  digitalWrite(LEDCollision2,LOW);
  digitalWrite(Bumper1OutPin,LOW);
  digitalWrite(Bumper2OutPin,LOW);
}
//================================== END SETUP =======================================
//====================================================================================
//==================================== MAIN ==========================================
void loop() 
{
  // ----------------------------------------- mS Timer for measurement interval -----
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;   
    //flagAdc0Read = 1;
    //flagAdc1Read = 1;
  }
  
  
  // --------------------------------------------------------------------------------
  // ----------------------------------------------- active LED blink ---------------
  if(currentMillis - previousMillisLedActive >= intervalLedActive) 
  {
    previousMillisLedActive = currentMillis;   
    //if (LEDActiveState == LOW)
    //  LEDActiveState = HIGH;
    //else
    //  LEDActiveState = LOW;
    LEDActiveState = !LEDActiveState;
    digitalWrite(LEDActive, LEDActiveState);
  }

  
  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Zero-Point ---------------------------------
  // ------------------------------- SENSOR 1 ---------------------------------------
  if(sensor1State == 0)
  {
    for (count1FirstRead = 0; count1FirstRead <= 2; count1FirstRead++)
    {    
      sensor1DiffFirst = sensor1DiffFirst + analogRead(sensor1InPin);
      Serial.print("Calc Zero-Point - Nr.: ");    
      Serial.print(count1FirstRead);
      Serial.print(" - Sum: ");
      Serial.println(sensor1DiffFirst);        
    }
    
    sensor1DiffFirst = sensor1DiffFirst / 3;
    Serial.print("Calc Zero-Point Erg = ");
    Serial.println(sensor1DiffFirst);
  
    sensor1State = 1;
  }

  
  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Touch-Point --------------------------------
  // ------------------------------- SENSOR 1 ---------------------------------------  
  if(sensor1State == 1)
  {
    if (count1SecondRead == 0)
      sensor1DiffSecond = 0;
      
    if(count1SecondRead < 4)
    { 
      sensor1DiffSecond = sensor1DiffSecond + analogRead(sensor1InPin);
      Serial.print("count1SecondRead = ");
      Serial.print(count1SecondRead);
      Serial.print(" - Sum: ");
      Serial.println(sensor1DiffSecond);        
      count1SecondRead++;
    }
    else
    {
      sensor1DiffSecond = sensor1DiffSecond / 4;
      Serial.print("Calc TP = ");
      Serial.println(sensor1DiffSecond);
      count1SecondRead = 0;
      sensor1State = 2;
    }
  }

  
  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Zero-Touch-Diff ----------------------------
  // ------------------------------- SENSOR 1 ---------------------------------------  
/*
  if(sensor1State == 2)
  {
    sensor1Diff = sensor1DiffSecond - sensor1DiffFirst;
    Serial.println(sensor1Diff);
    
    sensor1DiffSecond = 0;
    count1SecondRead = 0;
    sensor1State = 3;
  }
*/

  // --------------------------------------------------------------------------------
  // -------------------- Check Zero-Touch-Diff for Trigger -------------------------
  // ------------------------------- SENSOR 1 ---------------------------------------
  if (sensor1State == 2)
  {
    if((sensor1DiffSecond - sensor1DiffFirst) >= sensor1Trigger) 
    {
      Serial.print("Calc-Zero-Touch-Diff Trigger = ");
      Serial.println(sensor1DiffSecond - sensor1DiffFirst);
    
      digitalWrite(LEDCollision1,HIGH);
      digitalWrite(Bumper1OutPin,HIGH);
      trigger1Counter++;
      sensor1State = 1;
    }
    else
    {
      sensor1State = 1;
      digitalWrite(LEDCollision1,LOW);
      digitalWrite(Bumper1OutPin,LOW);
      
      if (millis() > nextTimeCalcZero + 60000)
      {
        nextTimeCalcZero = millis();
        sensor1DiffFirst = 0;
        sensor1State = 0;
      }
    }
  }

//  
//  if (sensor1State == 3 && sensor1Diff < sensor1Trigger) 
//  {
//    if (millis() > nextTimeCalcZero + 10000)
//    {
//      nextTimeCalcZero = millis();
//      sensor1State = 0;
//    }
//    sensor1DiffFirst = 0;
//    sensor1DiffSecond = 0;
//    count1SecondRead = 0;
//    count1FirstRead = 0;
//    digitalWrite(LEDCollision1,LOW);
//    digitalWrite(Bumper1OutPin,LOW);
//  }


}// end void loop()
//==================================== END MAIN ======================================
//====================================================================================
