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

/* pitch/roll and heading estimation (IMU sensor fusion)  
   requires: GY-80 module (L3G4200D, ADXL345B, HMC5883L) 
   
How to use it (example):     
  1. initialize IMU:                 IMU imu;  imu.init(); 
  2. read IMU (yaw/pitch/roll:       Serial.println( imu.ypr.yaw );
*/


#ifndef IMU_H
#define IMU_H

  #include <Arduino.h>
  #include "globals.h"
 
  boolean IMU_init(); //**OK
  void IMU_update();  //**OK
  int IMU_getCallCounter();  //**OK
  int IMU_getErrorCounter();  //**OK
  void IMU_deleteCalib();  //**OK
  boolean IMU_calibAccNextAxis();  //**OK
  void IMU_calibComStartStop();                             //**OK
  void IMU_calibComUpdate();                                //**OK
  boolean IMU_newMinMaxFound();                             //**OK
  float IMU_scalePI(float v);                               //**OK
  float IMU_scale180(float v);                              //**OK
  float IMU_distancePI(float x, float w);                   //**OK
  void IMU_read();                                          //**OK
  void IMU_loadSaveCalib(boolean IMU_readflag);  
  void IMU_calibGyro();
  void IMU_loadCalib();  
  void IMU_printPt(point_float_t p);
  void IMU_printCalib();
  void IMU_saveCalib();
  float IMU_sermin(float oldvalue, float newvalue);
  float IMU_sermax(float oldvalue, float newvalue);
  void IMU_initADXL345B();
  boolean IMU_initL3G4200D();
  void IMU_initHMC5883L();
  void IMU_readL3G4200D(boolean useTa);
  void IMU_readADXL345B();
  void IMU_readHMC5883L();

#endif

