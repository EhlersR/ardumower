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

#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include "drivers.h"
#include "flashmem.h"
#include "buzzer.h"
#include "globals.h"
#include "tone.h"

// -------------I2C addresses ------------------------
#define ADXL345B 83          // ADXL345B acceleration sensor (GY-80 PCB)
#define HMC5883L 30          // HMC5883L compass sensor (GY-80 PCB)
#define L3G4200D (0xD2 >> 1) // L3G4200D gyro sensor (GY-80 PCB)

#define pinLED 13
#define ADDR 600
#define MAGIC 1

struct 
{
  uint8_t xl;
  uint8_t xh;
  uint8_t yl;
  uint8_t yh;
  uint8_t zl;
  uint8_t zh;
} gyroFifo[32];

// rescale to -PI..+PI  **OK
float IMU_scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) 
    return (-2*PI+d); 
  else if (d < -PI) 
    return (2*PI+d);
  else 
    return d;  
}

// rescale to -180..+180 **OK
float IMU_scale180(float v)
{
  float d = v;
  while (d < 0) d+=2*180;
  while (d >= 2*180) d-=2*180;
  if (d >= 180)
    return (-2*180+d); 
  else if (d < -180)
    return (2*180+d);
  else
    return d;  
}

// computes minimum distance between x radiant (current-value) and w radiant (set-value) **OK
float IMU_distancePI(float x, float w)
{
  // cases:   
  // w=330 degree, x=350 degree => -20 degree
  // w=350 degree, x=10  degree => -20 degree
  // w=10  degree, x=350 degree =>  20 degree
  // w=0   degree, x=190 degree => 170 degree
  // w=190 degree, x=0   degree => -170 degree 
  float d = IMU_scalePI(w - x);
  if (d < -PI)
    d = d + 2*PI;
  else if (d > PI) 
    d = d - 2*PI;  
  
  return d;
}

//not used
//float IMU_distance180(float x, float w)
//{
//  float d = IMU_scale180(w - x);
//  if (d < -180)
//    d = d + 2*180;
//  else if (d > 180)
//    d = d - 2*180;  
//  
//  return d;
//}

//not used
// weight fusion (w=0..1) of two radiant values (a,b)
//float IMU_fusionPI(float w, float a, float b)
//{ 
//  float c;
//  if ((b >= PI/2) && (a <= -PI/2))
//  {
//    c = w * a + (1.0-w) * (b-2*PI);
//  }
//  else if ((b <= -PI/2) && (a >= PI/2))
//  {
//    c = w * (a-2*PI) + (1.0-w) * b;
//  }
//  else
//    c = w * a + (1.0-w) * b;
//  
//  return IMU_scalePI(c);
//}

void IMU_loadSaveCalib(boolean readflag)
{
  int addr = ADDR;
  short magic = MAGIC;
  eereadwrite(readflag, addr, magic); // magic
  eereadwrite(readflag, addr, IMU_accOfs);
  eereadwrite(readflag, addr, IMU_accScale);    
  eereadwrite(readflag, addr, IMU_comOfs);
  eereadwrite(readflag, addr, IMU_comScale);    
  
  // Daten vom RAM-Buffer ins FLASH schreiben
  if (!readflag)
    EEPROM.eeprom_update();  
}

void IMU_loadCalib()
{
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC)
  {
    Console.println(F("IMU error: no calib data"));
    return;  
  }
  IMU_calibrationAvail = true;
  Console.println(F("IMU: found calib data"));
  IMU_loadSaveCalib(true);
}

void IMU_saveCalib()
{
  IMU_loadSaveCalib(false);
}

void IMU_deleteCalib()
{
  int addr = ADDR;
  eewrite(addr, (short)0); // magic  
  IMU_accOfs.x=IMU_accOfs.y=IMU_accOfs.z=0;
  IMU_accScale.x=IMU_accScale.y=IMU_accScale.z=2;  
  IMU_comOfs.x=IMU_comOfs.y=IMU_comOfs.z=0;
  IMU_comScale.x=IMU_comScale.y=IMU_comScale.z=2;  
  Console.println("IMU calibration deleted");  
}

void IMU_printPt(point_float_t p)
{
  Console.print(p.x);
  Console.print(",");    
  Console.print(p.y);  
  Console.print(",");
  Console.println(p.z);  
}

void IMU_printCalib()
{
  Console.println(F("--------"));
  Console.print(F("accOfs="));
  IMU_printPt(IMU_accOfs);
  Console.print(F("accScale="));
  IMU_printPt(IMU_accScale);
  Console.print(F("comOfs="));
  IMU_printPt(IMU_comOfs);
  Console.print(F("comScale="));
  IMU_printPt(IMU_comScale);  
  Console.println(F("--------"));
}


// calculate gyro offsets 
void IMU_calibGyro()
{
  //Console.println(F("---calibGyro---"));  
  IMU_useGyroCalibration = false;
  IMU_gyroOfs.x = IMU_gyroOfs.y = IMU_gyroOfs.z = 0;
  point_float_t ofs;
  while(true)
  {    
    float zmin =  99999;
    float zmax = -99999;  
    IMU_gyroNoise = 0;
    ofs.x = ofs.y = ofs.z = 0;      
    for (int i=0; i < 50; i++)
    {
      delay(10);
      IMU_readL3G4200D(true);      
      zmin = min(zmin, IMU_gyro.z);
      zmax = max(zmax, IMU_gyro.z);
      ofs.x += ((float)IMU_gyro.x)/ 50.0;
      ofs.y += ((float)IMU_gyro.y)/ 50.0;
      ofs.z += ((float)IMU_gyro.z)/ 50.0;          
      IMU_gyroNoise += sq(IMU_gyro.z-IMU_gyroOfs.z) /50.0;   // noise is computed with last offset calculation
    }
//    Console.print(F("gyro calib min="));
//    Console.print(zmin);
//    Console.print(F("\tmax="));    
//    Console.print(zmax);
//    Console.print(F("\tofs="));        
//    Console.print(ofs.z);    
//    Console.print(F("\tnoise="));                
//    Console.println(IMU_gyroNoise);  
    if (IMU_gyroNoise < 20)
      break; // optimum found    
    
    IMU_gyroOfs = ofs; // new offset found
  }  
  IMU_useGyroCalibration = true;
//  Console.print(F("counter="));
//  Console.println(IMU_gyroCounter);  
//  Console.print(F("ofs="));
  IMU_printPt(IMU_gyroOfs);  
//  Console.println(F("------------"));  
}      


// ADXL345B acceleration sensor driver
// Beschleunigungssensor
void  IMU_initADXL345B()
{
  I2CwriteTo(ADXL345B, 0x2D, 0);
  I2CwriteTo(ADXL345B, 0x2D, 16); // AUTO-Sleep
  I2CwriteTo(ADXL345B, 0x2D, 8);  // Measure       
}

//Beschleunigungssensor lesen
void IMU_readADXL345B()
{  
  uint8_t buf[6];
  if (I2CreadFrom(ADXL345B, 0x32, 6, (uint8_t*)buf) != 6)
  {
//    Console.println("Error ADXL345B");
    IMU_errorCounter++;
    return;
  }
  // Convert the accelerometer value to G's. 
  // With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  // Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  // ( *0.0078 )
  float x=(int16_t) (((uint16_t)buf[1]) << 8 | buf[0]); 
  float y=(int16_t) (((uint16_t)buf[3]) << 8 | buf[2]); 
  float z=(int16_t) (((uint16_t)buf[5]) << 8 | buf[4]); 
/*
  Console.print("buf[0]=");
  Console.print(buf[0]);
  Console.print(" / buf[1]=");
  Console.print(buf[1]);
  Console.print(" / buf[2]=");
  Console.print(buf[2]);
  Console.print(" / buf[3]=");
  Console.print(buf[3]);
  Console.print(" / buf[4]=");
  Console.print(buf[4]);
  Console.print(" / buf[5]=");
  Console.println(buf[5]);
*/

  if (IMU_useAccCalibration)
  {
    x -= IMU_accOfs.x;
    y -= IMU_accOfs.y;
    z -= IMU_accOfs.z;
    x /= IMU_accScale.x*0.5;    
    y /= IMU_accScale.y*0.5;    
    z /= IMU_accScale.z*0.5;
    IMU_acc.x = x;
    //Console.println(z);
    IMU_acc.y = y;
    IMU_acc.z = z;
  }
  else
  {
    IMU_acc.x = x;
    IMU_acc.y = y;
    IMU_acc.z = z;
  }
  /*float accXreal = x + sin(gyroYpr.pitch);
  accXmax = max(accXmax, accXreal );    
  accXmin = min(accXmin, accXreal );        */
  //float amag = sqrt(Xa*Xa + Ya*Ya + Za*Za);
  //Xa /= amag;
  //Ya /= amag;
  //Za /= amag;  
  IMU_accelCounter++;

/*
  Console.print("AccX=");
  Console.print(x);
  Console.print(" / AccY=");
  Console.print(y);
  Console.print(" / AccZ=");
  Console.print(z);
*/

}

// L3G4200D gyro sensor driver                 
// Misst Rotationsgeschwindigkeit in °/sek
// INIT
boolean IMU_initL3G4200D()
{
//  Console.println(F("initL3G4200D"));
  uint8_t buf[6];    
  int retry = 0;
  while (true)
  {
    I2CreadFrom(L3G4200D, 0x0F, 1, (uint8_t*)buf);
    //Console.println(buf[0]);
    if (buf[0] != 0xD3)
    {        
//      Console.println(F("gyro read error"));
      retry++;
      if (retry > 2)
      {
        IMU_errorCounter++;
        return false;
      }
      delay(1000);            
    }
    else
      break;
  }
  
  I2CwriteTo(L3G4200D, 0x20, 0b00001100);    // Normal power mode, all axes enabled, 100 Hz
  I2CwriteTo(L3G4200D, 0x23, 0b00100000);    // 2000 dps (degree per second)
  I2CreadFrom(L3G4200D, 0x23, 1, (uint8_t*)buf);
  if (buf[0] != 0b00100000)
  {
//      Console.println(F("gyro write error")); 
      while(true);
  }  
  // fifo mode 
  // I2CwriteTo(L3G4200D, 0x24, 0b01000000);        
  // I2CwriteTo(L3G4200D, 0x2e, 0b01000000);          
  delay(250);
  IMU_calibGyro();    
  return true;
}

// Rotationsgeschwindigkeit lesen    //**OK
void IMU_readL3G4200D(boolean useTa)
{  
  IMU_now = micros();
  float Ta = 1;
  Ta = ((IMU_now - IMU_lastGyroTime) / 1000000.0);    
  IMU_lastGyroTime = IMU_now;
  if (Ta > 0.5)
    Ta = 0;   // should only happen for the very first call
      
  uint8_t fifoSrcReg = 0;  
  I2CreadFrom(L3G4200D, 0x2F, sizeof(fifoSrcReg), &fifoSrcReg);         // read the FIFO_SRC_REG
   // FIFO_SRC_REG
   // 7: Watermark status. (0: FIFO filling is lower than WTM level; 1: FIFO filling is equal or higher than WTM level)
   // 6: Overrun bit status. (0: FIFO is not completely filled; 1:FIFO is completely filled)
   // 5: FIFO empty bit. (0: FIFO not empty; 1: FIFO empty)
   // 4..0: FIFO stored data level
   //Console.print("FIFO_SRC_REG: "); Console.println(fifoSrcReg, HEX);
  uint8_t countOfData = (fifoSrcReg & 0x1F) + 1;   
  //  if (bitRead(fifoSrcReg, 6)==1) Console.println(F("IMU error: FIFO overrun"));

  memset(gyroFifo, 0, sizeof(gyroFifo[0])*32);
  I2CreadFrom(L3G4200D, 0xA8, sizeof(gyroFifo[0])*countOfData, (uint8_t *)gyroFifo);         // the first bit of the register address specifies we want automatic address increment

  IMU_gyro.x = IMU_gyro.y = IMU_gyro.z = 0;
  //Console.print("fifo:");
  //Console.println(countOfData);
  if (!IMU_useGyroCalibration)
    countOfData = 1;
  
  for (uint8_t i=0; i<countOfData; i++)
  {
      IMU_gyro.x += (int16_t) (((uint16_t)gyroFifo[i].xh) << 8 | gyroFifo[i].xl);
      IMU_gyro.y += (int16_t) (((uint16_t)gyroFifo[i].yh) << 8 | gyroFifo[i].yl);
      IMU_gyro.z += (int16_t) (((uint16_t)gyroFifo[i].zh) << 8 | gyroFifo[i].zl);
      if (IMU_useGyroCalibration)
      {
        IMU_gyro.x -= IMU_gyroOfs.x;
        IMU_gyro.y -= IMU_gyroOfs.y;
        IMU_gyro.z -= IMU_gyroOfs.z;               
      }
  }
  if (IMU_useGyroCalibration)
  {
    IMU_gyro.x *= 0.07 * PI/180.0;  // convert to radiant per second
    IMU_gyro.y *= 0.07 * PI/180.0; 
    IMU_gyro.z *= 0.07 * PI/180.0;      
  }
  IMU_gyroCounter++;

/*
  Console.print(" / GyroX=");
  Console.print(gyro.x);
  Console.print(" / GyroY=");
  Console.print(gyro.y);
  Console.print(" / GyroZ=");
  Console.println(gyro.z);
*/
  
}


// HMC5883L compass sensor driver  //**OK
// Kompass
void  IMU_initHMC5883L()
{
  I2CwriteTo(HMC5883L, 0x00, 0x70);  // 8 samples averaged, 75Hz frequency, no artificial bias.       
  I2CwriteTo(HMC5883L, 0x01, 0x20);  // gain
  I2CwriteTo(HMC5883L, 0x02, 00);    // mode         
}

// Kompass lesen
void IMU_readHMC5883L()
{    
  uint8_t buf[6];  
  if (I2CreadFrom(HMC5883L, 0x03, 6, (uint8_t*)buf) != 6)
  {
    IMU_errorCounter++;
    return;
  }
  // scale +1.3Gauss..-1.3Gauss  (*0.00092)  
  float x = (int16_t) (((uint16_t)buf[0]) << 8 | buf[1]);
  float y = (int16_t) (((uint16_t)buf[4]) << 8 | buf[5]);
  float z = (int16_t) (((uint16_t)buf[2]) << 8 | buf[3]);  
  //Console.print("Kompass x = ");
  //Console.println(x);
  
  
  if (IMU_useComCalibration)
  {
    x -= IMU_comOfs.x;
    y -= IMU_comOfs.y;
    z -= IMU_comOfs.z;
    x /= IMU_comScale.x*0.5;    
    y /= IMU_comScale.y*0.5;    
    z /= IMU_comScale.z*0.5;

/*    
    com.x = x;
    com.y = y;
    com.z = z;
*/

    unsigned long TaC = millis() - IMU_lastCompassReadTime;    // sampling time in millis
    IMU_lastCompassReadTime = millis();  

    if (TaC > 1000)
      TaC = 1;  

    // http://phrogz.net/js/framerate-independent-low-pass-filter.html
    // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;          
    IMU_com.x += TaC * (x - IMU_com.x) / 100;        
    IMU_com.y += TaC * (y - IMU_com.y) / 100;
    IMU_com.z += TaC * (z - IMU_com.z) / 100;  

/*
    Console.print("ComX = ");
    Console.print(IMU_com.x);
    Console.print(" / ComY = ");
    Console.print(IMU_com.y);
    Console.print(" / ComZ = ");
    Console.println(IMU_com.z);   
*/
    
  }
  else
  {
    IMU_com.x = x;
    IMU_com.y = y;
    IMU_com.z = z;
  }  
}

//not used
//float IMU_sermin(float oldvalue, float newvalue)
//{
//  if (newvalue < oldvalue) 
//  {
//    Console.print(".");
//    digitalWrite(pinLED, true);
//  }
//  return min(oldvalue, newvalue);
//}

//not used
//float IMU_sermax(float oldvalue, float newvalue)
//{
//  if (newvalue > oldvalue) 
//  {
//    Console.print(".");
//    digitalWrite(pinLED, true);
//  }
//  return max(oldvalue, newvalue);
//}

void IMU_calibComStartStop()
{  
  while (Console.available())
    Console.read();  
  
  if (IMU_state == IMU_CAL_COM)
  {
    // stop 
    Console.println(F("com calib completed"));    
    IMU_calibrationAvail = true;
    float xrange = IMU_comMax.x - IMU_comMin.x;
    float yrange = IMU_comMax.y - IMU_comMin.y;
    float zrange = IMU_comMax.z - IMU_comMin.z;
    IMU_comOfs.x = xrange/2 + IMU_comMin.x;
    IMU_comOfs.y = yrange/2 + IMU_comMin.y;
    IMU_comOfs.z = zrange/2 + IMU_comMin.z;
    IMU_comScale.x = xrange;
    IMU_comScale.y = yrange;  
    IMU_comScale.z = zrange;
    IMU_saveCalib();  
    IMU_printCalib();
    IMU_useComCalibration = true; 
    IMU_state = IMU_RUN;    

    // completed sound
    tone(pinBuzzer, 600);
    delay(200); 
    tone(pinBuzzer, 880);
    delay(200); 
    tone(pinBuzzer, 1320);              
    delay(200); 
    noTone(pinBuzzer);    
    delay(500);
/*  
    MyBeep(200, 600);
    delay(200); 
    MyBeep(200, 880);
    delay(200); 
    MyBeep(200, 1320);              
    delay(200); 
*/
  }
  else
  {
    // start
    Console.println(F("com calib..."));
    Console.println(F("rotate sensor 360 degree around all three axis"));
    IMU_foundNewMinMax = false;  
    IMU_useComCalibration = false;
    IMU_state = IMU_CAL_COM;  
    IMU_comMin.x = IMU_comMin.y = IMU_comMin.z = 9999;
    IMU_comMax.x = IMU_comMax.y = IMU_comMax.z = -9999;
  }
}


//not used
//boolean IMU_newMinMaxFound()
//{
//  boolean res = IMU_foundNewMinMax;
//  IMU_foundNewMinMax = false;
//  return res;
//}  

void IMU_calibComUpdate()
{
  IMU_comLast = IMU_com;
  delay(20);
  IMU_readHMC5883L();  
  boolean newfound = false;
  if ( (abs(IMU_com.x - IMU_comLast.x) < 10) &&  (abs(IMU_com.y - IMU_comLast.y) < 10) &&  (abs(IMU_com.z - IMU_comLast.z) < 10))
  {
    if (IMU_com.x < IMU_comMin.x)
    { 
      IMU_comMin.x = IMU_com.x;
      newfound = true;      
    }
    if (IMU_com.y < IMU_comMin.y)
    { 
      IMU_comMin.y = IMU_com.y;
      newfound = true;      
    }
    if (IMU_com.z < IMU_comMin.z)
    { 
      IMU_comMin.z = IMU_com.z;
      newfound = true;      
    }
    if (IMU_com.x > IMU_comMax.x)
    { 
      IMU_comMax.x = IMU_com.x;
      newfound = true;      
    }
    if (IMU_com.y > IMU_comMax.y)
    { 
      IMU_comMax.y = IMU_com.y;
      newfound = true;      
    }
    if (IMU_com.z > IMU_comMax.z)
    { 
      IMU_comMax.z = IMU_com.z;
      newfound = true;      
    }    
    if (newfound)
    {      
      IMU_foundNewMinMax = true;   
      //MyBeep(100, 440);
      tone(pinBuzzer, 440);
//      Console.print("x:");
//      Console.print(IMU_comMin.x);
//      Console.print(",");
//      Console.print(IMU_comMax.x);
//      Console.print("\t  y:");
//      Console.print(IMU_comMin.y);
//      Console.print(",");
//      Console.print(IMU_comMax.y);
//      Console.print("\t  z:");
//      Console.print(IMU_comMin.z);
//      Console.print(",");
//      Console.print(IMU_comMax.z);    
//      Console.println("\t");
    }
    else
    {
      noTone(pinBuzzer);
    }   
  }    
}


// calculate acceleration sensor offsets  //**OK
boolean IMU_calibAccNextAxis()
{  
  boolean complete = false; 
  tone(pinBuzzer, 440);
  
  while (Console.available())
    Console.read();  
  
  IMU_useAccCalibration = false;  
  if (IMU_calibAccAxisCounter >= 6) IMU_calibAccAxisCounter = 0;
  if (IMU_calibAccAxisCounter == 0)
  {
    // restart
    Console.println(F("acc calib restart..."));
    IMU_accMin.x = IMU_accMin.y = IMU_accMin.z = 99999;
    IMU_accMax.x = IMU_accMax.y = IMU_accMax.z = -99999;    
  }
  point_float_t pt = {0,0,0};
  for (int i=0; i < 100; i++)
  {        
    IMU_readADXL345B();            
    pt.x += IMU_acc.x / 100.0;
    pt.y += IMU_acc.y / 100.0;
    pt.z += IMU_acc.z / 100.0;                  
    Console.print(IMU_acc.x);
    Console.print(",");
    Console.print(IMU_acc.y);
    Console.print(",");
    Console.println(IMU_acc.z);
    delay(1);
  }
  IMU_accMin.x = min(IMU_accMin.x, pt.x);
  IMU_accMax.x = max(IMU_accMax.x, pt.x);         
  IMU_accMin.y = min(IMU_accMin.y, pt.y);
  IMU_accMax.y = max(IMU_accMax.y, pt.y);         
  IMU_accMin.z = min(IMU_accMin.z, pt.z);
  IMU_accMax.z = max(IMU_accMax.z, pt.z);           
  IMU_calibAccAxisCounter++;        
  IMU_useAccCalibration = true;  
  Console.print("side ");
  Console.print(IMU_calibAccAxisCounter);
  Console.println(" of 6 completed");    
  if (IMU_calibAccAxisCounter == 6)
  {    
    // all axis complete 
    float xrange = IMU_accMax.x - IMU_accMin.x;
    float yrange = IMU_accMax.y - IMU_accMin.y;
    float zrange = IMU_accMax.z - IMU_accMin.z;
    IMU_accOfs.x = xrange/2 + IMU_accMin.x;
    IMU_accOfs.y = yrange/2 + IMU_accMin.y;
    IMU_accOfs.z = zrange/2 + IMU_accMin.z;
    IMU_accScale.x = xrange;
    IMU_accScale.y = yrange;  
    IMU_accScale.z = zrange;    
    IMU_printCalib();
    IMU_saveCalib();    
    Console.println("acc calibration completed");    
    complete = true;

    // completed sound
/*
    MyBeep(200, 600);
    delay(200);
    MyBeep(200, 880);
    delay(200);
    MyBeep(200, 1320);
    delay(200);
*/
    tone(pinBuzzer, 600);
    delay(200); 
    tone(pinBuzzer, 880);
    delay(200); 
    tone(pinBuzzer, 1320);              
    delay(200); 
  };
  noTone(pinBuzzer);
  delay(500);
  return complete;
}      

// first-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary2(float newAngle, float newRate,int looptime, float angle)
{
  float k=10;
  float dtc2=float(looptime)/1000.0;
  float x1 = (newAngle -   angle)*k*k;
  float y1 = dtc2*x1 + y1;
  float x2 = y1 + (newAngle -   angle)*2*k + newRate;
  angle = dtc2*x2 + angle;
  return angle;
}

// second-order complementary filter
// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary(float newAngle, float newRate,int looptime, float angle)
{
  float tau=0.075;
  float a=0.0;
  float dtC = float(looptime)/1000.0;
  a=tau/(tau+dtC);
  angle= a* (angle + newRate * dtC) + (1-a) * (newAngle);
  return angle;
}

// Kalman filter                                      
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Kalman(float newAngle, float newRate,int looptime, float x_angle)
{
  float Q_angle  =  0.02;    //0.001 Vertrauenswürdigkeit des Signals (weniger vertrauen = Zahl größer / mehr vertauen = Zahl kleiner)
  float Q_gyro   =  0.0003;  //0.003
  
  float R_angle  =  0.01;    //0.03  Sensorrauschen

  float x_bias = 0;
  float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  float y, S;
  float K_0, K_1;

  float dt = float(looptime) / 1000;
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
}

// scale setangle, so that both PI angles have the same sign    
float scalePIangles(float setAngle, float currAngle)
{
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}

void IMU_update()     //**OK
{
  IMU_read();
  IMU_now = millis();
  int lt = (IMU_now - IMU_lastAHRSTime);
  IMU_lastAHRSTime = IMU_now;

  IMU_Ta_sum = 0; 
  //Ta mitteln (Versuch)
  IMU_Ta_Array[0] = lt;
  for (int i=99; i>0; i--)
  {
    IMU_Ta_Array[i] = IMU_Ta_Array[i-1];
  }
  for (int i=0; i < 100; i++)
  {
    IMU_Ta_sum += IMU_Ta_Array[i];
  }
  int IMU_looptime = IMU_Ta_sum / 100;


  //Console.print("Looptime IMU-Update = ");
  //Console.println(looptime);
    
  
  if (IMU_state == IMU_RUN)
  {     
    
    // ------ roll, pitch --------------  
    float forceMagnitudeApprox = abs(IMU_acc.x) + abs(IMU_acc.y) + abs(IMU_acc.z);    
    //if (forceMagnitudeApprox < 1.2)
    //{
      //Console.println(forceMagnitudeApprox);      
      IMU_accPitch   = atan2(-IMU_acc.x , sqrt(sq(IMU_acc.y) + sq(IMU_acc.z)));         
/*
      Console.print("acc.x=");
      Console.print(IMU_acc.x);        
      Console.print(" / acc.y=");
      Console.print(IMU_acc.y);        
      Console.print(" / acc.z=");
      Console.println(IMU_acc.z);        
*/
      IMU_accRoll  = atan2(IMU_acc.y , IMU_acc.z);             
      IMU_accPitch = scalePIangles(IMU_accPitch, IMU_ypr.pitch);
      IMU_accRoll  = scalePIangles(IMU_accRoll, IMU_ypr.roll);
      
      // complementary filter            
      IMU_ypr.pitch = Kalman(IMU_accPitch, IMU_gyro.x, IMU_looptime, IMU_ypr.pitch);  
      IMU_ypr.roll  = Kalman(IMU_accRoll,  IMU_gyro.y, IMU_looptime, IMU_ypr.roll);            
    
    /*} else {
      //Console.print("too much acceleration ");
      //Console.println(forceMagnitudeApprox);
      ypr.pitch = ypr.pitch + gyro.y * ((float)(looptime))/1000.0;
      ypr.roll  = ypr.roll  + gyro.x * ((float)(looptime))/1000.0;
    }*/
    IMU_ypr.pitch=IMU_scalePI(IMU_ypr.pitch);
    IMU_ypr.roll=IMU_scalePI(IMU_ypr.roll);
    // ------ yaw --------------
    // tilt-compensated yaw
    IMU_comTilt.x =  IMU_com.x * cos(IMU_ypr.pitch) + IMU_com.z * sin(IMU_ypr.pitch);
    IMU_comTilt.y =  IMU_com.x * sin(IMU_ypr.roll)  * sin(IMU_ypr.pitch) + IMU_com.y * cos(IMU_ypr.roll) - IMU_com.z * sin(IMU_ypr.roll) * cos(IMU_ypr.pitch);
    IMU_comTilt.z = -IMU_com.x * cos(IMU_ypr.roll)  * sin(IMU_ypr.pitch) + IMU_com.y * sin(IMU_ypr.roll) + IMU_com.z * cos(IMU_ypr.roll) * cos(IMU_ypr.pitch);     
    IMU_comYaw = IMU_scalePI( atan2(IMU_comTilt.y, IMU_comTilt.x));  
    IMU_comYaw = scalePIangles(IMU_comYaw, IMU_yaw);
    //IMU_comYaw = atan2(IMU_com.y, IMU_com.x);  // assume pitch, roll are 0
    //complementary filter
    IMU_yaw = Complementary2(IMU_comYaw, -IMU_gyro.z, IMU_looptime, IMU_yaw);
    IMU_yaw = IMU_scalePI(IMU_yaw);

/*
    Console.print("yaw_roh = ");
    Console.print(yaw/PI*180);
*/
    // Tiefpass
    unsigned long TaC = millis() - IMU_lastIMUupdate;    
    IMU_lastIMUupdate = millis();  
    if (TaC > 1000)
      TaC = 1;

    IMU_yaw1 += TaC * (IMU_yaw - IMU_yaw1) / 1000;
    IMU_ypr.yaw = IMU_yaw1;
/*
    Console.print(" / yaw = ");
    Console.println(ypr.yaw/PI*180);  
 */
   
  } 
  else if (IMU_state == IMU_CAL_COM)
  {
    IMU_calibComUpdate();
  }
}  

boolean IMU_init()    //**OK
{    
  IMU_loadCalib();
  IMU_printCalib();    
  if (!IMU_initL3G4200D()) return false;
  IMU_initADXL345B();
  IMU_initHMC5883L();    
  IMU_now = 0;  
  IMU_hardwareInitialized = true;
  return true;
}

int IMU_getCallCounter()      //**OK
{
  int res = IMU_callCounter;
  IMU_callCounter = 0;
  return res;
}

int IMU_getErrorCounter()     //**OK
{
  int res = IMU_errorCounter;
  IMU_errorCounter = 0;
  return res;
}

void IMU_read()               //**OK
{  
  if (!IMU_hardwareInitialized)
  {
    IMU_errorCounter++;
    return;
  }
  IMU_callCounter++;    
  IMU_readL3G4200D(true);   //Rotationsgeschwindigkeit
  IMU_readADXL345B();
  IMU_readHMC5883L();  
}


