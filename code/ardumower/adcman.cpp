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

#include <chip.h>
#include <Arduino.h>
#include <limits.h>
#include "adcman.h"
#include "drivers.h"
#include "flashmem.h"

#define ADDR 500
#define MAGIC 1
#define CHANNELS 11
#define MAXSAMPLECOUNT 255
#define NO_CHANNEL 255

volatile short position = 0;
volatile int16_t lastvalue = 0;
volatile uint8_t channel = 0;
volatile boolean busy = false;
volatile boolean ADC_Enable = false; 
int16_t capture[CHANNELS][MAXSAMPLECOUNT];        // ADC capture buffer (ADC0-ADC7) - 10 bit signed (signed: zero = ADC/2)     
uint8_t captureSize[CHANNELS];                    // ADC sample buffer size (ADC0-ADC7)
int16_t ofs[CHANNELS];                            // ADC zero offset (ADC0-ADC7)
int16_t ADCMin[CHANNELS];                         // ADC min sample value (ADC-ADC7)
int16_t ADCMax[CHANNELS];                         // ADC max sample value (ADC-ADC7)
boolean captureComplete[CHANNELS];                // ADC buffer filled?
boolean autoCalibrate[CHANNELS];                  // do auto-calibrate? (ADC0-ADC7)
int16_t sample[CHANNELS][MAXSAMPLECOUNT];         // ADC one sample (ADC0-ADC7) - 10 bit unsigned
ADCManager ADCMan;


ADCManager::ADCManager()
{
  calibrationAvail = false;
  for (int i=0; i < CHANNELS; i++)
  {
    captureSize[i]=0;
    ofs[i]=0;
    captureComplete[i]=false;
    capture[i][0] = 0;
    autoCalibrate[i] = false;
    ADCMax[i] = -9999;
    ADCMin[i] = 9999;
  }
  capturedChannels = 0;
  
  //sampleRate = SRATE_19231;
  sampleRate = SRATE_38462;
  //sampleRate = SRATE_9615;

}

void ADCManager::init()
{    
  analogReadResolution(12);

  switch (sampleRate)
  {
    case SRATE_38462:
      CreateTimerInterrupt0_Core1(ContinuousTimerInterrupt, 2599, ReadADC);
      break;
    case SRATE_19231:
      CreateTimerInterrupt0_Core1(ContinuousTimerInterrupt, 5198, ReadADC);   //alle 50us
      break;  
    case SRATE_9615:
      CreateTimerInterrupt0_Core1(ContinuousTimerInterrupt, 10396, ReadADC);  //alle 100us  
      break;
  }
  
  //Console.print("ADCMan:init() -> SampleRate=");
  //Console.println(sampleRate);
  if (loadCalib())
    printCalib();
}



// ADC ISR
void ReadADC(void)
{
  if (ADC_Enable == false)
    return;
      
  volatile int16_t value;
  switch (channel)
  {
    case 0:
      value = ReadAD0(); break;
    case 1:
      value = ReadAD1(); break;
    case 2:
      value = ReadAD2(); break;
    case 3:
      value = ReadAD3(); break;
    case 4:
      value = ReadAD4(); break;
    case 5:
      value = ReadAD5(); break;
    case 6:
      value = ReadAD6(); break;
    case 7:
      value = ReadAD7(); break;
    case 8:
      value = ReadAD8(); break;
    case 9:
      value = ReadAD9(); break;
    case 10:
      value = ReadAD10(); break;
    case 11:
      value = ReadAD11(); break;
  }
  
  if (!busy)
    return;
    
  if (position >= captureSize[channel])
  {
    // stop capture
    captureComplete[channel]=true;    
    busy=false;
    return;
  } 
  
  value -= ofs[channel];                   
  capture[channel][position] =  min(2048,  max(-2047, value / 4));              // convert to signed (zero = ADC/2)                                    
  sample[channel][position] = value;           
  // determine min/max 
  if (value < ADCMin[channel]) ADCMin[channel]  = value;
  if (value > ADCMax[channel]) ADCMax[channel]  = value;        
  position++;   
    
}


void ADCManager::setCapture(byte pin, byte samplecount, boolean autoCalibrateOfs)
{  
  int ch = pin-A0;
  captureSize[ch] = samplecount;
  Console.print("ADC Set Capture - Channel ");
  Console.print(ch);
  Console.print(" - Samplecount = ");
  Console.println(samplecount);
  autoCalibrate[ch] = autoCalibrateOfs;
}


void ADCManager::calibrate()
{
  Console.println("ADC calibration...");
  for (int ch=0; ch < CHANNELS; ch++)
  {    
    ADCMax[ch] = -9999;
    ADCMin[ch] = 9999;    
    ofs[ch] = 0;    
    if (autoCalibrate[ch])
	  {
      calibrateOfs(A0 + ch);
    }
  }
  printCalib();  
  saveCalib();
  calibrationAvail = true;
}


boolean ADCManager::calibrationDataAvail()
{
  return calibrationAvail;
}


void ADCManager::calibrateOfs(byte pin)
{  
  int ch = pin-A0;
  ADCMax[ch] = -9999;
  ADCMin[ch] = 9999;
  ofs[ch]=0;    
  for (int i=0; i < 10; i++)
  {
    captureComplete[ch]=false;      
    while (!isCaptureComplete(pin))
	  {
      delay(20);
      run();    
    } 
  }
  int16_t center = ADCMin[ch] + (ADCMax[ch] - ADCMin[ch]) / 2.0;
  ofs[ch] = center;   
}

void ADCManager::printCalib()
{
  Console.println(F("---ADC calib---"));  
  Console.print(F("ADC sampleRate="));
  switch (sampleRate)
  {
    case SRATE_38462: Console.println(F("38462")); break;
    case SRATE_19231: Console.println(F("19231")); break;
    case SRATE_9615 : Console.println(F("9615")); break;
  }    

  for (int ch=0; ch < CHANNELS; ch++)
  {
    Console.print(F("AD"));
    Console.print(ch);
    Console.print(F("\t"));    
    Console.print(F("min="));    
    Console.print(ADCMin[ch]);
    Console.print(F("\t"));    
    Console.print(F("max="));    
    Console.print(ADCMax[ch]);    
    Console.print(F("\t"));    
    Console.print(F("diff="));        
    Console.print(ADCMax[ch]-ADCMin[ch]);    
    Console.print(F("\t"));    
    Console.print(F("ofs="));    
    Console.println(ofs[ch]);
  }
}

void ADCManager::startADC(int sampleCount)
{
  ADC_Enable = true;
  return;      
}


void ADCManager::startCapture(int sampleCount)
{
  position = 0;
  busy=true;
  startADC(sampleCount);  
}
  

void ADCManager::stopCapture()
{  
  position = 0;
  ADC_Enable = false;
}

void ADCManager::run()
{
  if (busy)
  {
    return;
  }

  if (position != 0)
  {
    stopCapture();
    capturedChannels++;
  }

  // find next channel for capturing
  for (int i=0; i < CHANNELS; i++)
  {    
    channel++;
    if (channel == CHANNELS) channel = 0;
    if ((captureSize[channel] != 0) && (!captureComplete[channel]))
  {        
      // found channel for sampling      
      startCapture( captureSize[channel] );                   
      break;
    }      
  }
}

int16_t* ADCManager::getCapture(byte pin)
{  
  return (int16_t*)capture[pin-A0];
}

boolean ADCManager::isCaptureComplete(byte pin)
{
  int ch = pin-A0;
  return captureComplete[ch];      
}


int ADCManager::read(byte pin)
{    
  int ch = pin-A0;
  captureComplete[ch]=false;    
  if (captureSize[ch] == 0)
    return 0;  
  else
    return sample[ch][(captureSize[ch]-1)];        
}


//Not used
int ADCManager::readMedian(byte pin)
{
  int ch = pin-A0;
  captureComplete[ch]=false;    
  if (captureSize[ch] == 0)
    return 0;
  else if (captureSize[ch] == 1)
    return sample[ch][0];
  else
  { 
    for (int i = 1; i < captureSize[ch]; ++i)
    {
      int j = sample[ch][i];
      int k;
      for (k = i - 1; (k >= 0) && (j > sample[ch][k]); k--)
      {
        sample[ch][k + 1] = sample[ch][k];
      }
      sample[ch][k + 1] = j;
    }
    return sample[ch][(int)(captureSize[ch]/2 )];
  }
}  

void ADCManager::restart(byte pin)
{
  captureComplete[pin-A0]=false;
}

      
int ADCManager::getCapturedChannels()
{
  int res = capturedChannels;
  capturedChannels=0;
  return res;
}


int ADCManager::getCaptureSize(byte pin)
{
  int ch = pin-A0;  
  return captureSize[ch];
}

int16_t ADCManager::getADCMin(byte pin)
{
  int ch = pin-A0;  
  if (ch >= CHANNELS)
    return 0;
  return ADCMin[ch];
}

int16_t ADCManager::getADCMax(byte pin)
{
  int ch = pin-A0;  
  if (ch >= CHANNELS)
    return 0;
  return ADCMax[ch];
}

int16_t ADCManager::getADCOfs(byte pin)
{
  int ch = pin-A0;  
  if (ch >= CHANNELS)
    return 0;
  return ofs[ch];
}  

void ADCManager::loadSaveCalib(boolean readflag)
{
  int addr = ADDR;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  for (int ch=0; ch < CHANNELS; ch++)
  {
    eereadwrite(readflag, addr, ofs[ch]);
  }  

  // Daten vom RAM-Buffer ins FLASH schreiben
  if (!readflag)
    EEPROM.eeprom_update();
}

boolean ADCManager::loadCalib()
{
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC)
  {
    Console.println(F("ADCMan error: no calib data"));
    return false;   
  }
  calibrationAvail = true;
  Console.println(F("ADCMan: found calib data"));
  loadSaveCalib(true);
  return true;
}

void ADCManager::saveCalib()
{
  loadSaveCalib(false);
}




