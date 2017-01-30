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

// Android remote control (pfod App)
// For a detailed specification of the pfodApp protocol, please visit:  http://www.forward.com.au/pfod/

#include "pfod.h"
#include "robot.h"
#include "adcman.h"
#include "imu.h"
#include "perimeter.h"


RemoteControl::RemoteControl()
{
  pfodCmdComplete = false;
  pfodCmd = "";
  pfodState = PFOD_OFF;
  testmode = 0;
  nextPlotTime = 0;  
  perimeterCaptureIdx = 0;  
}

void RemoteControl::setRobot(Robot *aRobot)
{
  this->robot = aRobot;
}

void RemoteControl::initSerial(uint32_t baudrate)
{
  Serial1.begin(baudrate);
  Console.println("initSerial");
}

float RemoteControl::stringToFloat(String &s)
{
  float v;
  char tmp[20];  
  s.toCharArray(tmp, sizeof(tmp));
  v = s.toFloat();
  return v;
}

void RemoteControl::sendYesNo(int value)
{
  if (value == 1)
    Serial1.print("YES");
  else
    Serial1.print("NO");
}

void RemoteControl::sendOnOff(int value)
{
   if (value == 1)
     Serial1.print("ON");
   else
     Serial1.print("OFF");
}

void RemoteControl::sendTimer(ttimer_t timer)
{
  if (timer.active) 
    Serial1.print(F("(X)  "));
  else
    Serial1.print(F("(   )  "));
  
  Serial1.print(time2str(timer.startTime));
  Serial1.print("-");
  Serial1.print(time2str(timer.stopTime));
  Serial1.println();
  if (timer.daysOfWeek == 255)
  {
    Serial1.print(F("every day"));
  }
  else
  {
    int counter= 0;  
    for (int j=0; j < 7; j++)
    {
      if ((timer.daysOfWeek >> j) & 1)
      {
        if (counter != 0) Serial1.print(",");
        Serial1.print(dayOfWeek[j]);
        counter++;      
      }
    }
  }
}


// NOTE: pfodApp rev57 changed slider protocol:  displayValue = (sliderValue + offset) * scale
void RemoteControl::sendSlider(String cmd, String title, float value, String unit, double scale, float maxvalue, float minvalue)
{
  Serial1.print("|");
  Serial1.print(cmd);
  Serial1.print("~");
  Serial1.print(title);
  Serial1.print(" `");
  Serial1.print(((int)(value/scale)));
  Serial1.print("`");
  Serial1.print(((int)(maxvalue/scale)));
  Serial1.print("`");
  Serial1.print(((int)(minvalue/scale)));
  Serial1.print("~ ~");
  if (scale == 10)
    Serial1.print("10");
  else if (scale == 1)
    Serial1.print("1");
  else if (scale == 0.1)
    Serial1.print("0.1");
  else if (scale == 0.01)
    Serial1.print("0.01");
  else if (scale == 0.001)
    Serial1.print("0.001");
  else if (scale == 0.0001)
    Serial1.print("0.0001");
}

void RemoteControl::sendPIDSlider(String cmd, String title, PID &pid, double scale, float maxvalue)
{
  sendSlider(cmd + "p", title + "_P", pid.Kp, "", scale, maxvalue);
  sendSlider(cmd + "i", title + "_I", pid.Ki, "", scale, maxvalue);
  sendSlider(cmd + "d", title + "_D", pid.Kd, "", scale, maxvalue);  
}

void RemoteControl::processSlider_float(String result, float &value, double scale)
{
  int idx = result.indexOf('`');
  String s = result.substring(idx + 1);      
  float v = stringToFloat(s);
  value = v * scale;  
}


void RemoteControl::processPIDSlider(String result, String cmd, PID &pid, double scale, float maxvalue)
{
  int idx = result.indexOf('`');
  String s = result.substring(idx + 1);      
  float v = stringToFloat(s);
  if (pfodCmd.startsWith(cmd + "p"))
  {
    pid.Kp = v * scale;
    if (pid.Kp < scale) pid.Kp = 0.0;
  }
  else if (pfodCmd.startsWith(cmd + "i"))
  {
    pid.Ki = v * scale;
    if (pid.Ki < scale) pid.Ki = 0.0;
  }
  else if (pfodCmd.startsWith(cmd + "d"))
  { 
    pid.Kd = v * scale;      
    if (pid.Kd < scale) pid.Kd = 0.0;
  }
}

void RemoteControl::processSlider_long(String result, long &value, double scale)
{
  float v;
  processSlider_float(result, v, scale);
  value = v;
}

void RemoteControl::processSlider_int(String result, int &value, double scale)
{
  float v;
  processSlider_float(result, v, scale);
  value = v;
}

void RemoteControl::processSlider_int16(String result, int16_t &value, double scale)
{
  float v;
  processSlider_float(result, v, scale);
  value = v;
}

void RemoteControl::processSlider_byte(String result, byte &value, double scale)
{
  float v;
  processSlider_float(result, v, scale);
  value = v;
}

void RemoteControl::processSlider_short(String result, short &value, double scale)
{
  float v;
  processSlider_float(result, v, scale);
  value = v;
}
    
void RemoteControl::sendMainMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
  {
    Serial1.print(F("{.Ardumower"));
    Serial1.print(" (");
    Serial1.print(robot->name);
    Serial1.print(")");
  }
  
  Serial1.print(F("|r~Commands|n~Manual|s~Settings|in~Info|c~Test compass|m1~Log sensors|yp~Plot"));
  Serial1.println(F("|y4~Error counters|y9~ADC calibration}"));
}

void RemoteControl::sendADCMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.ADC calibration`1000"));
  
  Serial1.print(F("|c1~Calibrate (perimeter sender, charger must be off) "));
  for (int ch=0; ch < 16; ch++)
  {   
    int16_t adcMin = ADCMan.getADCMin(A0+ch);
    int16_t adcMax = ADCMan.getADCMax(A0+ch);
    int16_t adcOfs = ADCMan.getADCOfs(A0+ch);    
    Serial1.print(F("|zz~AD"));
    Serial1.print(ch);
    Serial1.print(" min=");
    Serial1.print(adcMin);
    Serial1.print(" max=");
    Serial1.print(adcMax);
    Serial1.print(" diff=");
    Serial1.print(adcMax-adcMin);
    Serial1.print(" ofs=");
    Serial1.print(adcOfs);
  }
  Serial1.println("}");
}  


void RemoteControl::sendPlotMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Plot"));
  
  Serial1.print(F("|y7~Sensors|y5~Sensor counters|y3~IMU|y6~Perimeter|y8~GPS"));
  Serial1.println(F("|y1~Battery|y2~Odometry2D|y11~Motor control|y10~GPS2D}"));
}  


void RemoteControl::sendSettingsMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Settings"));
  
  Serial1.print(F("|sz~Save settings|s1~Motor|s2~Mow|s3~Bumper|s4~Sonar|s5~Perimeter|s6~Lawn sensor|s7~IMU|s8~R/C"));
  Serial1.println(F("|s9~Battery|s10~Station|s11~Odometry|s13~Rain|s15~Drop sensor|s14~GPS|i~Timer|s12~Date/time|sx~Factory settings}"));
}  

void RemoteControl::sendErrorMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Error counters`1000"));
  
  Serial1.print(F("|z00~Reset counters"));
  Serial1.print(F("|zz~ADC calibration "));
  Serial1.print(robot->errorCounterMax[ERR_ADC_CALIB]);
  Serial1.print(F("|zz~Charger "));
  Serial1.print(robot->errorCounterMax[ERR_CHARGER]);
  Serial1.print(F("|zz~Battery "));
  Serial1.print(robot->errorCounterMax[ERR_BATTERY]);
  Serial1.print(F("|zz~Motor left "));
  Serial1.print(robot->errorCounterMax[ERR_MOTOR_LEFT]);
  Serial1.print(F("|zz~Motor right "));
  Serial1.print(robot->errorCounterMax[ERR_MOTOR_RIGHT]);
  Serial1.print(F("|zz~Motor mow "));
  Serial1.print(robot->errorCounterMax[ERR_MOTOR_MOW]);
  Serial1.print(F("|zz~Mow sense "));
  Serial1.print(robot->errorCounterMax[ERR_MOW_SENSE]);
  Serial1.print(F("|zz~Odometry left "));
  Serial1.print(robot->errorCounterMax[ERR_ODOMETRY_LEFT]);
  Serial1.print(F("|zz~Odometry right "));
  Serial1.print(robot->errorCounterMax[ERR_ODOMETRY_RIGHT]);
  Serial1.print(F("|zz~Perimeter timeout "));
  Serial1.print(robot->errorCounterMax[ERR_PERIMETER_TIMEOUT]);
  Serial1.print(F("|zz~Perimeter tracking "));
  Serial1.print(robot->errorCounterMax[ERR_TRACKING]);
  Serial1.print(F("|zz~IMU comm "));
  Serial1.print(robot->errorCounterMax[ERR_IMU_COMM]);
  Serial1.print(F("|zz~IMU calibration "));
  Serial1.print(robot->errorCounterMax[ERR_IMU_CALIB]);
  Serial1.print(F("|zz~IMU tilt "));
  Serial1.print(robot->errorCounterMax[ERR_IMU_TILT]);
  Serial1.print(F("|zz~RTC comm "));
  Serial1.print(robot->errorCounterMax[ERR_RTC_COMM]);
  Serial1.print(F("|zz~RTC data "));
  Serial1.print(robot->errorCounterMax[ERR_RTC_DATA]);
  Serial1.print(F("|zz~GPS comm "));
  Serial1.print(robot->errorCounterMax[ERR_GPS_COMM]);
  Serial1.print(F("|zz~GPS data "));
  Serial1.print(robot->errorCounterMax[ERR_GPS_DATA]);
  Serial1.print(F("|zz~Robot stuck "));
  Serial1.print(robot->errorCounterMax[ERR_STUCK]);
  Serial1.print(F("|zz~EEPROM data "));
  Serial1.print(robot->errorCounterMax[ERR_EEPROM_DATA]);
  Serial1.println("}");
}  

void RemoteControl::processErrorMenu(String pfodCmd)
{      
  if (pfodCmd == "z00")
  {
    robot->resetErrorCounters();
    robot->setNextState(STATE_OFF, 0);
  }
  sendErrorMenu(true);
}


void RemoteControl::sendMotorMenu(boolean update)
{  
  if (update)
    Serial1.print("{:");
  else 
     Serial1.print(F("{.Motor`1000"));
  
  Serial1.println(F("|a00~Overload Counter l, r "));
  Serial1.print(robot->motorLeftSenseCounter);
  Serial1.print(", ");
  Serial1.print(robot->motorRightSenseCounter);
  Serial1.println(F("|a01~Power in Watt l, r "));
  Serial1.print(robot->motorLeftSense);
  Serial1.print(", ");
  Serial1.print(robot->motorRightSense);
  Serial1.println(F("|a05~motor current in mA l, r "));
  Serial1.print(robot->motorLeftSenseCurrent);
  Serial1.print(", ");
  Serial1.print(robot->motorRightSenseCurrent);
  //Console.print("motorpowermax=");
  //Console.println(robot->motorPowerMax);
  sendSlider("a02", F("Power max"), robot->motorPowerMax, "", 0.1, 100);  
  sendSlider("a03", F("calibrate left motor "), robot->motorLeftSenseCurrent, "", 1, 1000, 0);       
  sendSlider("a04", F("calibrate right motor"), robot->motorRightSenseCurrent, "", 1, 1000, 0);      
  Serial1.print(F("|a05~Speed l, r"));
  Serial1.print(robot->motorLeftPWMCurr);
  Serial1.print(", ");
  Serial1.print(robot->motorRightPWMCurr);
  sendSlider("a06", F("Speed max in rpm"), robot->motorSpeedMaxRpm, "", 1, 100);    
  sendSlider("a15", F("Speed max in pwm"), robot->motorSpeedMaxPwm, "", 1, 255);      
  sendSlider("a11", F("Accel"), robot->motorAccel, "", 1, 2000, 500);  
  sendSlider("a18", F("Power ignore time"), robot->motorPowerIgnoreTime, "", 1, 8000);     
  sendSlider("a07", F("Roll time max"), robot->motorRollTimeMax, "", 1, 8000); 
  sendSlider("a19", F("Roll time min"), robot->motorRollTimeMin, "", 1, (robot->motorRollTimeMax - 500)); 
  sendSlider("a08", F("Reverse time"), robot->motorReverseTime, "", 1, 8000);     
  sendSlider("a09", F("Forw time max"), robot->motorForwTimeMax, "", 10, 80000);       
  sendSlider("a12", F("Bidir speed ratio 1"), robot->motorBiDirSpeedRatio1, "", 0.01, 1.0);       
  sendSlider("a13", F("Bidir speed ratio 2"), robot->motorBiDirSpeedRatio2, "", 0.01, 1.0);       
  sendPIDSlider("a14", "RPM", robot->motorLeftPID, 0.01, 3.0);        
  Serial1.println(F("|a10~Testing is"));
  switch (testmode)
  {
    case 0: Serial1.print(F("OFF")); break;
    case 1: Serial1.print(F("Left motor forw")); break;
    case 2: Serial1.print(F("Right motor forw")); break;
  }
  Serial1.print(F("|a14~for config file:"));
  Serial1.print(F("motorSenseScale l, r"));
  Serial1.print(robot->motorSenseLeftScale);
  Serial1.print(", ");
  Serial1.print(robot->motorSenseRightScale);
  Serial1.print(F("|a16~Swap left direction "));
  sendYesNo(robot->motorLeftSwapDir);
  Serial1.print(F("|a17~Swap right direction "));
  sendYesNo(robot->motorRightSwapDir);
  Serial1.println("}");
}

void RemoteControl::processMotorMenu(String pfodCmd)
{      
  if (pfodCmd.startsWith("a02"))
  { 
    processSlider_float(pfodCmd, robot->motorPowerMax, 0.1);
    //Console.print("motorpowermax=");
    //Console.println(robot->motorPowerMax);
  }
  else if (pfodCmd.startsWith("a03"))
  {
      processSlider_float(pfodCmd, robot->motorLeftSenseCurrent, 1);
      robot->motorSenseLeftScale = robot->motorLeftSenseCurrent / max(0,(float)robot->motorLeftSenseADC);                  
  }
  else if (pfodCmd.startsWith("a04"))
  {
      processSlider_float(pfodCmd, robot->motorRightSenseCurrent, 1);
      robot->motorSenseRightScale = robot->motorRightSenseCurrent / max(0,(float)robot->motorRightSenseADC); 
  }      
  else if (pfodCmd.startsWith("a06")) processSlider_int(pfodCmd, robot->motorSpeedMaxRpm, 1);
  else if (pfodCmd.startsWith("a15")) processSlider_int(pfodCmd, robot->motorSpeedMaxPwm, 1);
  else if (pfodCmd.startsWith("a07")) processSlider_int(pfodCmd, robot->motorRollTimeMax, 1); 
  else if (pfodCmd.startsWith("a19")) processSlider_int(pfodCmd, robot->motorRollTimeMin, 1); 
  else if (pfodCmd.startsWith("a08")) processSlider_int(pfodCmd, robot->motorReverseTime, 1);
  else if (pfodCmd.startsWith("a09")) processSlider_long(pfodCmd, robot->motorForwTimeMax, 10);
  else if (pfodCmd.startsWith("a11")) processSlider_float(pfodCmd, robot->motorAccel, 1);    
  else if (pfodCmd.startsWith("a12")) processSlider_float(pfodCmd, robot->motorBiDirSpeedRatio1, 0.01);    
  else if (pfodCmd.startsWith("a13")) processSlider_float(pfodCmd, robot->motorBiDirSpeedRatio2, 0.01);    
  else if (pfodCmd.startsWith("a14")) processPIDSlider(pfodCmd, "a14", robot->motorLeftPID, 0.01, 3.0);
  else if (pfodCmd.startsWith("a16")) robot->motorLeftSwapDir = !robot->motorLeftSwapDir;
  else if (pfodCmd.startsWith("a17")) robot->motorRightSwapDir = !robot->motorRightSwapDir;  
  else if (pfodCmd.startsWith("a18")) processSlider_int(pfodCmd, robot->motorPowerIgnoreTime, 1);        
  else if (pfodCmd == "a10")
  { 
    testmode = (testmode + 1) % 3;
    switch (testmode)
    {
      case 0: robot->setNextState(STATE_OFF,0); break;
      case 1: robot->setNextState(STATE_MANUAL,0); robot->motorRightSpeedRpmSet = 0; robot->motorLeftSpeedRpmSet = robot->motorSpeedMaxRpm; break;
      case 2: robot->setNextState(STATE_MANUAL,0); robot->motorLeftSpeedRpmSet  = 0; robot->motorRightSpeedRpmSet = robot->motorSpeedMaxRpm; break;      
    }
  }
  sendMotorMenu(true);
}
  
void RemoteControl::sendMowMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Mow`1000"));
  
  Serial1.print(F("|o00~Overload Counter "));
  Serial1.print(robot->motorMowSenseCounter);
  Serial1.print(F("|o01~Power in Watt "));
  Serial1.print(robot->motorMowSense);
  Serial1.print(F("|o11~current in mA "));
  Serial1.print(robot->motorMowSenseCurrent);
  sendSlider("o02", F("Power max"), robot->motorMowPowerMax, "", 0.1, 100);         
  sendSlider("o03", F("calibrate mow motor "), robot->motorMowSenseCurrent, "", 1, 3000, 0);          
  Serial1.print(F("|o04~Speed "));
  Serial1.print(robot->motorMowPWMCurr);
  sendSlider("o05", F("Speed max"), robot->motorMowSpeedMaxPwm, "", 1, 255);       
  if (robot->developerActive) 
  {
    Serial1.print(F("|o06~Modulate "));
    sendYesNo(robot->motorMowModulate);
  }      
  Serial1.print(F("|o07~RPM "));
  Serial1.print(robot->motorMowRpmCurr);
  sendSlider("o08", F("RPM set"), robot->motorMowRPMSet, "", 1, 4500);     
  sendPIDSlider("o09", "RPM", robot->motorMowPID, 0.01, 1.0);      
  Serial1.println(F("|o10~Testing is"));
  switch (testmode)
  {
    case 0: Serial1.print(F("OFF")); break;
    case 1: Serial1.print(F("Motor ON")); break;
  }
  Serial1.println(F("|o04~for config file:"));
  Serial1.println(F("motorMowSenseScale:"));
  Serial1.print(robot->motorMowSenseScale);
  Serial1.println("}");
}


void RemoteControl::processMowMenu(String pfodCmd)
{      
  if (pfodCmd.startsWith("o02"))
    processSlider_float(pfodCmd, robot->motorMowPowerMax, 0.1);
  else if (pfodCmd.startsWith("o03"))
  {
    processSlider_float(pfodCmd, robot->motorMowSenseCurrent, 1);
    robot->motorMowSenseScale = robot->motorMowSenseCurrent / max(0,(float)robot->motorMowSenseADC);
  } 
  else if (pfodCmd.startsWith("o05"))
    processSlider_int(pfodCmd, robot->motorMowSpeedMaxPwm, 1);
  else if (pfodCmd == "o06")
    robot->motorMowModulate = !robot->motorMowModulate;    
  else if (pfodCmd.startsWith("o08"))
    processSlider_int(pfodCmd, robot->motorMowRPMSet, 1);    
  else if (pfodCmd.startsWith("o09"))
    processPIDSlider(pfodCmd, "o09", robot->motorMowPID, 0.01, 1.0);
  else if (pfodCmd == "o10")
  { 
    testmode = (testmode + 1) % 2;
    switch (testmode)
    {
      case 0: robot->setNextState(STATE_OFF,0);robot->motorMowRpmCurr = 0; robot->motorMowEnable = false; break;
      case 1: robot->setNextState(STATE_MANUAL,0); robot->motorMowEnable = true; break;
    }
  }    
  sendMowMenu(true);
}


void RemoteControl::sendBumperMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else 
    Serial1.print(F("{.Bumper`1000"));
  
  Serial1.print(F("|b00~Use "));
  sendYesNo(robot->bumperUse);    
  Serial1.println(F("|b01~Counter l, r "));
  Serial1.print(robot->bumperLeftCounter);
  Serial1.print(", ");
  Serial1.print(robot->bumperRightCounter);
  Serial1.println(F("|b02~Value l, r "));
  Serial1.print(robot->bumperLeft);
  Serial1.print(", ");
  Serial1.print(robot->bumperRight);
  Serial1.println("}");
}

void RemoteControl::sendDropMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Drop`1000"));
  
  Serial1.print(F("|u00~Use "));
  sendYesNo(robot->dropUse);    
  Serial1.println(F("|u01~Counter l, r "));
  Serial1.print(robot->dropLeftCounter);
  Serial1.print(", ");
  Serial1.print(robot->dropRightCounter);
  Serial1.println(F("|u02~Value l, r "));
  Serial1.print(robot->dropLeft);
  Serial1.print(", ");
  Serial1.print(robot->dropRight);
  Serial1.println("}");
}


void RemoteControl::processBumperMenu(String pfodCmd)
{      
  if (pfodCmd == "b00")
    robot->bumperUse = !robot->bumperUse;    
  
  sendBumperMenu(true);
}

void RemoteControl::processDropMenu(String pfodCmd)
{      
  if (pfodCmd == "u00")
    robot->dropUse = !robot->dropUse;    
  
  sendDropMenu(true);
}


void RemoteControl::sendSonarMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Sonar`1000"));         
  
  Serial1.print(F("|d00~Use "));
  sendYesNo(robot->sonarUse);
  Serial1.print(F("|d04~Use left "));
  sendYesNo(robot->sonarLeftUse);
  Serial1.print(F("|d05~Use center "));
  sendYesNo(robot->sonarCenterUse);
  Serial1.print(F("|d06~Use right "));
  sendYesNo(robot->sonarRightUse);
  Serial1.print(F("|d01~Counter "));
  Serial1.print(robot->sonarDistCounter);    
  Serial1.println(F("|d02~Value l, c, r"));
  Serial1.print(robot->sonarDistLeft);
  Serial1.print(", ");
  Serial1.print(robot->sonarDistCenter);
  Serial1.print(", ");
  Serial1.print(robot->sonarDistRight);  
  sendSlider("d03", F("Trigger below"), robot->sonarTriggerBelow, "", 1, 80);       
  Serial1.println("}"); 
}

void RemoteControl::processSonarMenu(String pfodCmd)
{      
  if (pfodCmd == "d00")
    robot->sonarUse = !robot->sonarUse;
  else if (pfodCmd.startsWith("d03"))
    processSlider_int(pfodCmd, robot->sonarTriggerBelow, 1);
  else if (pfodCmd == "d04")
    robot->sonarLeftUse = !robot->sonarLeftUse;
  else if (pfodCmd == "d05")
    robot->sonarCenterUse = !robot->sonarCenterUse;
  else if (pfodCmd == "d06")
    robot->sonarRightUse = !robot->sonarRightUse;
  
  sendSonarMenu(true);
}

void RemoteControl::sendPerimeterMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Perimeter`1000"));
  
  Serial1.print(F("|e00~Use "));
  sendYesNo(robot->perimeterUse);  
  Serial1.println(F("|e02~Value"));
  Serial1.print(robot->perimeterMag);
  if (robot->perimeterMag < 0)
    Serial1.print(" (inside)");
  else
    Serial1.print(" (outside)");
  
  sendSlider("e08", F("Timed-out if below smag"), robot->perimeter.timedOutIfBelowSmag, "", 1, 2000);  
  sendSlider("e14", F("Timeout (s) if not inside"), robot->perimeter.timeOutSecIfNotInside, "", 1, 20, 1);  
  sendSlider("e04", F("Trigger timeout"), robot->perimeterTriggerTimeout, "", 1, 2000);
  sendSlider("e05", F("Perimeter out roll time max"), robot->perimeterOutRollTimeMax, "", 1, 8000);       
  sendSlider("e06", F("Perimeter out roll time min"), robot->perimeterOutRollTimeMin, "", 1, 8000); 
  sendSlider("e15", F("Perimeter out reverse time"), robot->perimeterOutRevTime, "", 1, 8000); 
  sendSlider("e16", F("Perimeter tracking roll time"), robot->perimeterTrackRollTime, "", 1, 8000); 
  sendSlider("e17", F("Perimeter tracking reverse time"), robot->perimeterTrackRevTime, "", 1, 8000); 
  sendSlider("e11", F("Transition timeout"), robot->trackingPerimeterTransitionTimeOut, "", 1, 5000);
  sendSlider("e12", F("Track error timeout"), robot->trackingErrorTimeOut, "", 1, 10000);             
  sendPIDSlider("e07", F("Track"), robot->perimeterPID, 0.01, 35);  //ehl
  Serial1.print(F("|e09~Use differential signal "));
  sendYesNo(robot->perimeter.useDifferentialPerimeterSignal);    
  Serial1.print(F("|e10~Swap coil polarity "));
  sendYesNo(robot->perimeter.swapCoilPolarity);
  Serial1.print(F("|e13~Block inner wheel  "));
  sendYesNo(robot->trackingBlockInnerWheelWhilePerimeterStruggling);
  Serial1.println("}");
}


void RemoteControl::processPerimeterMenu(String pfodCmd)
{      
  if (pfodCmd == "e00")
    robot->perimeterUse = !robot->perimeterUse;
  else if (pfodCmd.startsWith("e04"))
    processSlider_int(pfodCmd, robot->perimeterTriggerTimeout, 1);  
  else if (pfodCmd.startsWith("e05"))
    processSlider_int(pfodCmd, robot->perimeterOutRollTimeMax, 1);
  else if (pfodCmd.startsWith("e06"))
    processSlider_int(pfodCmd, robot->perimeterOutRollTimeMin, 1);
  else if (pfodCmd.startsWith("e15"))
    processSlider_int(pfodCmd, robot->perimeterOutRevTime, 1);
  else if (pfodCmd.startsWith("e16"))
    processSlider_int(pfodCmd, robot->perimeterTrackRollTime, 1);
  else if (pfodCmd.startsWith("e17"))
    processSlider_int(pfodCmd, robot->perimeterTrackRevTime, 1);
  else if (pfodCmd.startsWith("e07"))
    processPIDSlider(pfodCmd, "e07", robot->perimeterPID, 0.01, 35); //Ehl   
  else if (pfodCmd.startsWith("e08"))
    processSlider_int16(pfodCmd, robot->perimeter.timedOutIfBelowSmag, 1);    
  else if (pfodCmd.startsWith("e09"))
    robot->perimeter.useDifferentialPerimeterSignal = !robot->perimeter.useDifferentialPerimeterSignal;
  else if (pfodCmd.startsWith("e10"))
    robot->perimeter.swapCoilPolarity = !robot->perimeter.swapCoilPolarity;
  else if (pfodCmd.startsWith("e11"))
    processSlider_int(pfodCmd, robot->trackingPerimeterTransitionTimeOut, 1);
  else if (pfodCmd.startsWith("e12"))
    processSlider_int(pfodCmd, robot->trackingErrorTimeOut, 1);
  else if (pfodCmd.startsWith("e13"))
    robot->trackingBlockInnerWheelWhilePerimeterStruggling = !robot->trackingBlockInnerWheelWhilePerimeterStruggling;          
  else if (pfodCmd.startsWith("e14"))
    processSlider_int16(pfodCmd, robot->perimeter.timeOutSecIfNotInside, 1);     
  
  sendPerimeterMenu(true);
}

void RemoteControl::sendLawnSensorMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else 
    Serial1.print(F("{.Lawn sensor`1000"));
  
  Serial1.print(F("|f00~Use "));
  sendYesNo(robot->lawnSensorUse);  
  Serial1.print(F("|f01~Counter "));
  Serial1.print(robot->lawnSensorCounter);
  Serial1.println(F("|f02~Value f, b"));
  Serial1.print(robot->lawnSensorFront);
  Serial1.print(", ");
  Serial1.print(robot->lawnSensorBack);
  Serial1.println("}");
}


void RemoteControl::processLawnSensorMenu(String pfodCmd)
{      
  if (pfodCmd == "f00")
    robot->lawnSensorUse = !robot->lawnSensorUse;
  
  sendLawnSensorMenu(true);
}


void RemoteControl::sendRainMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Rain`1000"));
  
  Serial1.print(F("|m00~Use "));
  sendYesNo(robot->rainUse);  
  Serial1.print(F("|m01~Counter "));
  Serial1.print(robot->rainCounter);
  Serial1.println(F("|m02~Value"));
  Serial1.print(robot->rain);
  Serial1.println("}");
}

void RemoteControl::processRainMenu(String pfodCmd)
{      
  if (pfodCmd == "m00")
    robot->rainUse = !robot->rainUse;
  
  sendRainMenu(true);
}

void RemoteControl::sendGPSMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.GPS`1000"));
    
  Serial1.print(F("|q00~Use "));
  sendYesNo(robot->gpsUse);
  sendSlider("q01", F("Stuck if GPS speed is below"), robot->stuckIfGpsSpeedBelow, "", 0.1, 3); 
  sendSlider("q02", F("GPS speed ignore time"), robot->gpsSpeedIgnoreTime, "", 1, 10000, robot->motorReverseTime);       
  Serial1.println("}");
}

void RemoteControl::processGPSMenu(String pfodCmd)
{      
  if (pfodCmd == "q00")
    robot->gpsUse = !robot->gpsUse;
  else if (pfodCmd.startsWith("q01"))
    processSlider_float(pfodCmd, robot->stuckIfGpsSpeedBelow, 0.1);  
  else if (pfodCmd.startsWith("q02"))
    processSlider_int(pfodCmd, robot->gpsSpeedIgnoreTime, 1);  
  
  sendGPSMenu(true);
}


void RemoteControl::sendImuMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.IMU`1000"));
  
  Serial1.print(F("|g00~Use "));
  sendYesNo(robot->imuUse);
  Serial1.print(F("|g01~Yaw "));
  Serial1.print(robot->imu.ypr.yaw/PI*180);
  Serial1.print(F(" deg"));
  Serial1.print(F("|g09~DriveHeading "));
  Serial1.print(robot->imuDriveHeading/PI*180);
  Serial1.print(F(" deg"));
  Serial1.print(F("|g02~Pitch "));
  Serial1.print(robot->imu.ypr.pitch/PI*180);
  Serial1.print(F(" deg"));
  Serial1.print(F("|g03~Roll "));
  Serial1.print(robot->imu.ypr.roll/PI*180);
  Serial1.print(F(" deg"));
  Serial1.print(F("|g04~Correct dir "));
  sendYesNo(robot->imuCorrectDir);  
  sendPIDSlider("g05", F("Dir"), robot->imuDirPID, 0.1, 20);
  sendPIDSlider("g06", F("Roll"), robot->imuRollPID, 0.1, 30);    
  Serial1.print(F("|g07~Acc cal next side"));
  Serial1.print(F("|g08~Com cal start/stop"));
  Serial1.println("}");
}

void RemoteControl::processImuMenu(String pfodCmd)
{      
  if (pfodCmd == "g00")
    robot->imuUse = !robot->imuUse;
  else if (pfodCmd == "g04")
    robot->imuCorrectDir = !robot->imuCorrectDir;
  else if (pfodCmd.startsWith("g05"))
    processPIDSlider(pfodCmd, "g05", robot->imuDirPID, 0.1, 20);    
  else if (pfodCmd.startsWith("g06"))
    processPIDSlider(pfodCmd, "g06", robot->imuRollPID, 0.1, 30);    
  else if (pfodCmd == "g07")
    robot->imu.calibAccNextAxis();
  else if (pfodCmd == "g08")
    robot->imu.calibComStartStop();
  
  sendImuMenu(true);
}

void RemoteControl::sendRemoteMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Remote R/C`1000"));
  
  Serial1.print(F("|h00~Use "));
  sendYesNo(robot->remoteUse);  
  Serial1.println("}");
}

void RemoteControl::processRemoteMenu(String pfodCmd)
{      
  if (pfodCmd == "h00")
    robot->remoteUse = !robot->remoteUse;    
  
  sendRemoteMenu(true);
}

void RemoteControl::sendBatteryMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Battery`1000"));
  
  Serial1.print(F("|j00~Battery "));
  Serial1.print(robot->batVoltage);
  Serial1.print(" V");
  Serial1.print(F("|j01~Monitor "));
  sendYesNo(robot->batMonitor);
  if (robot->developerActive)
    sendSlider("j05", F("Calibrate batFactor "), robot->batFactor, "", 0.01, 1.0);   
  
  //Console.print("batFactor=");
  //Console.println(robot->batFactor);  
  sendSlider("j02", F("Go home if below Volt"), robot->batGoHomeIfBelow, "", 0.1, robot->batFull, (robot->batFull*0.72));  // for Sony Konion cells 4.2V * 0,72= 3.024V which is pretty safe to use 
  sendSlider("j12", F("Switch off if idle minutes"), robot->batSwitchOffIfIdle, "", 1, 300, 1);  
  sendSlider("j03", F("Switch off if below Volt"), robot->batSwitchOffIfBelow, "", 0.1, robot->batFull, (robot->batFull*0.72));  
  Serial1.print(F("|j04~Charge "));
  Serial1.print(robot->chgVoltage);
  Serial1.print("V ");
  Serial1.print(robot->chgCurrent);
  Serial1.print("A");
  sendSlider("j09", F("Calibrate batChgFactor"), robot->batChgFactor, "", 0.01, 1.0);       
  sendSlider("j06", F("Charge sense zero"), robot->chgSenseZero, "", 1, 600, 400);       
  sendSlider("j08", F("Charge factor"), robot->chgFactor, "", 0.01, 80);       
  sendSlider("j10", F("charging starts if Voltage is below"), robot->startChargingIfBelow, "", 0.1, robot->batFull);       
  sendSlider("j11", F("Battery is fully charged if current is below"), robot->batFullCurrent, "", 0.1, robot->batChargingCurrentMax);       
  Serial1.println("}");
}

void RemoteControl::processBatteryMenu(String pfodCmd)
{      
  if (pfodCmd == "j01")
    robot->batMonitor = !robot->batMonitor;
  else if (pfodCmd.startsWith("j02"))
  {
    processSlider_float(pfodCmd, robot->batGoHomeIfBelow, 0.1);
    //Console.print("gohomeifbelow=");
    //Console.println(robot->batGoHomeIfBelow);
  }
  else if (pfodCmd.startsWith("j03"))
    processSlider_float(pfodCmd, robot->batSwitchOffIfBelow, 0.1); 
  else if (pfodCmd.startsWith("j05"))
    processSlider_float(pfodCmd, robot->batFactor, 0.01);
  else if (pfodCmd.startsWith("j06"))
    processSlider_float(pfodCmd, robot->chgSenseZero, 1);   
  else if (pfodCmd.startsWith("j08"))
    processSlider_float(pfodCmd, robot->chgFactor, 0.01);    
  else if (pfodCmd.startsWith("j09"))
    processSlider_float(pfodCmd, robot->batChgFactor, 0.01);
  else if (pfodCmd.startsWith("j10"))
    processSlider_float(pfodCmd, robot->startChargingIfBelow, 0.1);
  else if (pfodCmd.startsWith("j11"))
    processSlider_float(pfodCmd, robot->batFullCurrent, 0.1);
  else if (pfodCmd.startsWith("j12"))
    processSlider_int(pfodCmd, robot->batSwitchOffIfIdle, 1);
  
  sendBatteryMenu(true);
}

void RemoteControl::sendStationMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Station`1000"));
  
  sendSlider("k00", F("Reverse time"), robot->stationRevTime, "", 1, 8000);     
  sendSlider("k01", F("Roll time"), robot->stationRollTime, "", 1, 8000);       
  sendSlider("k02", F("Forw time"), robot->stationForwTime, "", 1, 8000);         
  sendSlider("k03", F("Station reverse check time"), robot->stationCheckTime, "", 1, 8000);         
  Serial1.println("}");
}

void RemoteControl::processStationMenu(String pfodCmd)
{      
  if (pfodCmd.startsWith("k00"))
    processSlider_int(pfodCmd, robot->stationRevTime, 1);
  else if (pfodCmd.startsWith("k01"))
    processSlider_int(pfodCmd, robot->stationRollTime, 1);
  else if (pfodCmd.startsWith("k02"))
    processSlider_int(pfodCmd, robot->stationForwTime, 1);
  else if (pfodCmd.startsWith("k03")) 
    processSlider_int(pfodCmd, robot->stationCheckTime, 1);
  
  sendStationMenu(true);
}

void RemoteControl::sendOdometryMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Odometry2D`1000"));
  
  Serial1.print(F("|l00~Use "));
  sendYesNo(robot->odometryUse);  
  Serial1.print(F("|l01~Value l, r "));
  Serial1.print(robot->odometryLeft);
  Serial1.print(", ");
  Serial1.println(robot->odometryRight);
  Serial1.println(F("|l03~RPM Motor l, r "));
  Serial1.print(robot->motorLeftRpmCurr);
  Serial1.print(", ");
  Serial1.println(robot->motorRightRpmCurr);
  sendSlider("l04", F("Ticks per one full revolution"), robot->odometryTicksPerRevolution, "", 1, 2120);       
  sendSlider("l01", F("Ticks per cm"), robot->odometryTicksPerCm, "", 0.1, 35);       
  sendSlider("l02", F("Wheel base cm"), robot->odometryWheelBaseCm, "", 0.1, 50);  
  Serial1.print(F("|l08~Use two-way encoder "));
  sendYesNo(robot->twoWayOdometrySensorUse);
  Serial1.print(F("|l05~Swap left direction "));
  sendYesNo(robot->odometryLeftSwapDir);
  Serial1.print(F("|l06~Swap right direction "));
  sendYesNo(robot->odometryRightSwapDir);
  Serial1.println("}");
}


void RemoteControl::processOdometryMenu(String pfodCmd)
{      
  if (pfodCmd == "l00")
    robot->odometryUse = !robot->odometryUse;
  else if (pfodCmd.startsWith("l01"))
    processSlider_float(pfodCmd, robot->odometryTicksPerCm, 0.1);
  else if (pfodCmd.startsWith("l02"))
    processSlider_float(pfodCmd, robot->odometryWheelBaseCm, 0.1); 
  else if (pfodCmd.startsWith("l04"))
    processSlider_int(pfodCmd, robot->odometryTicksPerRevolution, 1);
  else if (pfodCmd.startsWith("l05"))
    robot->odometryLeftSwapDir = !robot->odometryLeftSwapDir;     
  else if (pfodCmd.startsWith("l06"))
    robot->odometryRightSwapDir = !robot->odometryRightSwapDir;
  else if (pfodCmd.startsWith("l08"))
    robot->twoWayOdometrySensorUse = !robot->twoWayOdometrySensorUse;    
  
  sendOdometryMenu(true);
}

void RemoteControl::sendDateTimeMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Date/time"));
  
  Serial1.print("|t00~");
  Serial1.print(date2str(robot->datetime.date));
  Serial1.print(", ");
  Serial1.print(time2str(robot->datetime.time));
  sendSlider("t01", dayOfWeek[robot->datetime.date.dayOfWeek], robot->datetime.date.dayOfWeek, "", 1, 6, 0);       
  sendSlider("t02", "Day ", robot->datetime.date.day, "", 1, 31, 1);       
  sendSlider("t03", "Month ", robot->datetime.date.month, "", 1, 12, 1);       
  sendSlider("t04", "Year ", robot->datetime.date.year, "", 1, 2020, 2013);       
  sendSlider("t05", "Hour ", robot->datetime.time.hour, "", 1, 23, 0);       
  sendSlider("t06", "Minute ", robot->datetime.time.minute, "", 1, 59, 0);           
  Serial1.println("}");
}

void RemoteControl::processDateTimeMenu(String pfodCmd)
{      
  if (pfodCmd.startsWith("t01"))
    processSlider_byte(pfodCmd, robot->datetime.date.dayOfWeek, 1);    
  else if (pfodCmd.startsWith("t02"))
    processSlider_byte(pfodCmd, robot->datetime.date.day, 1);
  else if (pfodCmd.startsWith("t03"))
    processSlider_byte(pfodCmd, robot->datetime.date.month, 1);
  else if (pfodCmd.startsWith("t04"))
    processSlider_short(pfodCmd, robot->datetime.date.year, 1);
  else if (pfodCmd.startsWith("t05"))
    processSlider_byte(pfodCmd, robot->datetime.time.hour, 1);
  else if (pfodCmd.startsWith("t06"))
    processSlider_byte(pfodCmd, robot->datetime.time.minute, 1);    
  
  sendDateTimeMenu(true);
  Console.print(F("setting RTC datetime: "));
  Console.println(date2str(robot->datetime.date));  
  robot->setActuator(ACT_RTC, 0);            
}

void RemoteControl::sendTimerDetailMenu(int timerIdx, boolean update)
{
  if (update)
    Serial1.print("{:"); 
  else
    Serial1.print(F("{.Details"));
  
  Serial1.print("|p0");
  Serial1.print(timerIdx);
  Serial1.print("~Use ");
  sendYesNo(robot->timer[timerIdx].active);        
  int startm = time2minutes(robot->timer[timerIdx].startTime);
  int stopm = time2minutes(robot->timer[timerIdx].stopTime);
  String sidx = String(timerIdx);
  sendSlider("p1"+sidx, F("Start hour "), robot->timer[timerIdx].startTime.hour, "", 1, 23, 0);       
  sendSlider("p2"+sidx, F("Start minute "), robot->timer[timerIdx].startTime.minute, "", 1, 59, 0);         
  sendSlider("p3"+sidx, F("Stop hour "), robot->timer[timerIdx].stopTime.hour, "", 1, 23, 0);       
  sendSlider("p4"+sidx, F("Stop minute "), robot->timer[timerIdx].stopTime.minute, "", 1, 59, 0);             
  for (int i=0; i < 7; i++)
  {
    Serial1.print("|p5");
    Serial1.print(timerIdx);
    Serial1.print(i);
    Serial1.print("~");
    if ((robot->timer[timerIdx].daysOfWeek >> i) & 1)
      Serial1.print("(X)  ");
    else
      Serial1.print("(  )  ");
    
    Serial1.print(dayOfWeek[i]);
  }
  Serial1.print("|p9");
  Serial1.print(timerIdx);
  Serial1.print(F("~Set to current time"));
  Serial1.println("}");
}

void RemoteControl::processTimerDetailMenu(String pfodCmd)
{      
  timehm_t time;
  boolean checkStop = false;
  boolean checkStart = false;
  int startmin, stopmin;
  int timerIdx = pfodCmd[2]-'0';
  if (pfodCmd.startsWith("p0"))
    robot->timer[timerIdx].active = !robot->timer[timerIdx].active;
  else if (pfodCmd.startsWith("p1"))
  {
    processSlider_byte(pfodCmd, robot->timer[timerIdx].startTime.hour, 1);
    checkStop = true;
  }
  else if (pfodCmd.startsWith("p2"))
  {
    processSlider_byte(pfodCmd, robot->timer[timerIdx].startTime.minute, 1);
    checkStop = true;
  }
  else if (pfodCmd.startsWith("p3"))
  {
    processSlider_byte(pfodCmd, robot->timer[timerIdx].stopTime.hour, 1);
    checkStart = true;
  }      
  else if (pfodCmd.startsWith("p4"))
  {
    processSlider_byte(pfodCmd, robot->timer[timerIdx].stopTime.minute, 1);
    checkStart = true;
  }        
  else if (pfodCmd.startsWith("p9"))
  {       
    robot->timer[timerIdx].startTime = robot->datetime.time;
    checkStop = true;      
    robot->timer[timerIdx].daysOfWeek = (1 << robot->datetime.date.dayOfWeek);      
  }
  else if (pfodCmd.startsWith("p5"))
  {
    int day = pfodCmd[3]-'0';
    robot->timer[timerIdx].daysOfWeek = robot->timer[timerIdx].daysOfWeek ^ (1 << day);
  }
  if (checkStop)
  {
    // adjust start time
    startmin = min(1434, time2minutes(robot->timer[timerIdx].startTime));
    minutes2time(startmin, time);
    robot->timer[timerIdx].startTime = time;
    // check stop time
    stopmin  = time2minutes(robot->timer[timerIdx].stopTime);
    stopmin = max(stopmin, startmin + 5);       
    minutes2time(stopmin, time);
    robot->timer[timerIdx].stopTime = time;
  }
  else if (checkStart)
  {
    // adjust stop time
    stopmin = max(5, time2minutes(robot->timer[timerIdx].stopTime));
    minutes2time(stopmin, time);
    robot->timer[timerIdx].stopTime = time;      
    // check start time
    startmin = time2minutes(robot->timer[timerIdx].startTime);      
    startmin = min(startmin, stopmin - 5);       
    minutes2time(startmin, time);
    robot->timer[timerIdx].startTime = time;
  }
  sendTimerDetailMenu(timerIdx, true);  
}

void RemoteControl::sendTimerMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Timer"));
  
  Serial1.print(F("|i99~Use "));
  sendYesNo(robot->timerUse);      
  for (int i=0; i < MAX_TIMERS; i++)
  {
    Serial1.print("|i0");
    Serial1.print(i);
    Serial1.print("~");
    sendTimer(robot->timer[i]);    
  }
  Serial1.println("}");
}

void RemoteControl::processTimerMenu(String pfodCmd)
{      
  if (pfodCmd.startsWith("i0"))
  {
    int timerIdx = pfodCmd[2]-'0';
    sendTimerDetailMenu(timerIdx, false);  
  }
  else
  {
    if (pfodCmd.startsWith("i99"))
      robot->timerUse = !robot->timerUse;
    
    sendTimerMenu(true);
  }  
}

void RemoteControl::sendFactorySettingsMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.println(F("{.Factory settings"));
  
  Serial1.print(F("|x0~Set factory settings (requires reboot)"));
  Serial1.println("}");
}

void RemoteControl::processFactorySettingsMenu(String pfodCmd)
{      
  if (pfodCmd == "x0")
    robot->deleteUserSettings();
  
  sendFactorySettingsMenu(true);
}

void RemoteControl::sendInfoMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Info"));
  
  Serial1.print(F("|v00~Ardumower "));
  Serial1.print(VER);
  Serial1.print(F("|v01~Developer "));
  sendYesNo(robot->developerActive); 
  Serial1.print(F("|v04~Stats override "));  
  sendYesNo(robot->statsOverride);     
  Serial1.print(F("|v02~Mowing time trip (min) "));
  Serial1.print(robot->statsMowTimeMinutesTrip);    
  Serial1.print(F("|v03~Mowing time total (hrs) "));
  Serial1.print(robot->statsMowTimeHoursTotal);
  Serial1.print(F("|v05~Battery charging cycles "));
  Serial1.print(robot->statsBatteryChargingCounterTotal);    
  Serial1.print(F("|v06~Battery recharged capacity trip (mAh)"));
  Serial1.print(robot->statsBatteryChargingCapacityTrip);    
  Serial1.print(F("|v07~Battery recharged capacity total (Ah)"));
  Serial1.print(robot->statsBatteryChargingCapacityTotal / 1000);    
  Serial1.print(F("|v08~Battery recharged capacity average (mAh)"));
  Serial1.print(robot->statsBatteryChargingCapacityAverage);        
  //Serial1.print("|d01~Perimeter v");
  //Serial1.print(verToString(readPerimeterVer()));
  //Serial1.print("|d02~IMU v");
  //Serial1.print(verToString(readIMUver()));
  //Serial1.print("|d02~Stepper v");
  //Serial1.print(verToString(readStepperVer()));
  Serial1.println("}");
}

void RemoteControl::processInfoMenu(String pfodCmd)
{      
  if (pfodCmd == "v01")
    robot->developerActive = !robot->developerActive;
  if (pfodCmd == "v04")
    robot->statsOverride = !robot->statsOverride; robot->saveUserSettings();

  sendInfoMenu(true);
}

void RemoteControl::sendCommandMenu(boolean update)
{  
  if (update)
    Serial1.print("{:");
  else
    Serial1.print(F("{.Commands`5000"));
    
  Serial1.print(F("|ro~OFF|ra~Auto mode|rc~RC mode|"));
  Serial1.print(F("rm~Mowing is "));
  sendOnOff(robot->motorMowEnable);  
  Serial1.print(F("|rp~Pattern is "));
  Serial1.print(robot->mowPatternName());
  Serial1.print(F("|rh~Home|rk~Track|rs~State is "));
  Serial1.print(robot->stateName());
  Serial1.print(F("|rr~Auto rotate is "));
  Serial1.print(robot->motorLeftPWMCurr);
  Serial1.print(F("|r1~User switch 1 is "));
  sendOnOff(robot->userSwitch1);  
  Serial1.print(F("|r2~User switch 2 is "));
  sendOnOff(robot->userSwitch2);  
  Serial1.print(F("|r3~User switch 3 is "));
  sendOnOff(robot->userSwitch3);  
  Serial1.print("}");
  Serial1.println();
}

void RemoteControl::processCommandMenu(String pfodCmd)
{
  if (pfodCmd == "ro")
  {
    // cmd: off      
    robot->setNextState(STATE_OFF, 0);          
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rh")
  {
    // cmd: home      
    robot->setNextState(STATE_PERI_FIND, 0);                      
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rr")
  {
    robot->setNextState(STATE_MANUAL, 0);
    robot->motorLeftSpeedRpmSet += 10; robot->motorRightSpeedRpmSet = -robot->motorLeftSpeedRpmSet;      
    sendCommandMenu(true);  
  }
  else if (pfodCmd == "rk")
  {
    // cmd: track perimeter      
    robot->setNextState(STATE_PERI_TRACK, 0);                      
    sendCommandMenu(true);
  }
  else if (pfodCmd == "ra")
  {
    // cmd: start auto mowing      
    robot->motorMowEnable = true;      
    robot->setNextState(STATE_FORWARD, 0);                
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rc")
  {      
    // cmd: start remote control (RC)      
    robot->motorMowEnable = true;
    robot->motorMowModulate = false;                
    robot->setNextState(STATE_REMOTE, 0);    
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rm")
  {
    // cmd: mower motor on/off
    if (robot->stateCurr == STATE_OFF || robot->stateCurr == STATE_MANUAL) robot->motorMowEnableOverride = false;
    else robot->motorMowEnableOverride = !robot->motorMowEnableOverride;     
    robot->motorMowEnable = !robot->motorMowEnable;      
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rs")
  {
    // cmd: state
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rp")
  {
    // cmd: pattern
    robot->mowPatternCurr = (robot->mowPatternCurr + 1 ) % 3;      
    robot->setNextState(STATE_OFF, 0);            
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r1")
  {
    robot->userSwitch1 = !robot->userSwitch1;
    robot->setUserSwitches();
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r2")
  {
    robot->userSwitch2 = !robot->userSwitch2;
    robot->setUserSwitches();
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r3")
  {
    robot->userSwitch3 = !robot->userSwitch3;
    robot->setUserSwitches();
    sendCommandMenu(true);
  } 
}


void RemoteControl::sendManualMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.println(F("{^Manual navigation`1000"));
  
  Serial1.print(F("|nl~Left|nr~Right|nf~Forward"));
  if (((robot->motorLeftSpeedRpmSet < 5)  && (robot->motorLeftSpeedRpmSet  > -5)) && ((robot->motorRightSpeedRpmSet < 5) && (robot->motorRightSpeedRpmSet > -5)))
  {
    Serial1.print(F("|nb~Reverse"));
  }
  else
    Serial1.print(F("|ns~Stop"));
  
  Serial1.print(F("|nm~Mow is "));
  sendOnOff(robot->motorMowEnable);  
  Serial1.println("}");
}


void RemoteControl::sendCompassMenu(boolean update)
{
  if (update)
    Serial1.print("{:");
  else
    Serial1.println(F("{^Compass`1000"));
  
  Serial1.print(F("|cw~West|ce~East|cn~North "));
  Serial1.print(robot->imu.ypr.yaw/PI*180);
  Serial1.println(F("|cs~South|cm~Mow}"));
}

void RemoteControl::processCompassMenu(String pfodCmd)
{
  if (pfodCmd == "cm")
  {
    robot->motorMowEnable = !robot->motorMowEnable;            
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cn")
  {      
    robot->imuRollHeading = 0;
    robot->setNextState(STATE_ROLL_WAIT, 0);            
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cs")
  {
    robot->imuRollHeading = PI;
    robot->setNextState(STATE_ROLL_WAIT, 0);            
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cw")
  {
    robot->imuRollHeading = -PI/2;
    robot->setNextState(STATE_ROLL_WAIT, 0);            
    sendCompassMenu(true);
  }
  else if (pfodCmd == "ce")
  {
    robot->imuRollHeading = PI/2;
    robot->setNextState(STATE_ROLL_WAIT, 0);            
    sendCompassMenu(true);
  }
}

void RemoteControl::processManualMenu(String pfodCmd)
{
  if (pfodCmd == "nl")
  {
    // manual: left
    robot->setNextState(STATE_MANUAL, 0);          
    float sign = 1.0;
    if (robot->motorLeftSpeedRpmSet < 0)
      sign = -1.0;      
    if (sign*robot->motorLeftSpeedRpmSet >= sign*robot->motorRightSpeedRpmSet)
      robot->motorLeftSpeedRpmSet = sign * robot->motorSpeedMaxRpm/2;      
    else
      robot->motorLeftSpeedRpmSet /= 2; 
    
    robot->motorRightSpeedRpmSet = sign * robot->motorSpeedMaxRpm;
    sendManualMenu(true);
  }
  else if (pfodCmd == "nr")
  {      
    // manual: right
    robot->setNextState(STATE_MANUAL, 0);          
    float sign = 1.0;
    if (robot->motorRightSpeedRpmSet < 0)
      sign = -1.0;
    if (sign*robot->motorRightSpeedRpmSet >= sign*robot->motorLeftSpeedRpmSet)
      robot->motorRightSpeedRpmSet = sign* robot->motorSpeedMaxRpm/2;
    else 
      robot->motorRightSpeedRpmSet /= 2;            
    
    robot->motorLeftSpeedRpmSet = sign * robot->motorSpeedMaxRpm;
    sendManualMenu(true);
  }
  else if (pfodCmd == "nf")
  {
    // manual: forward
    robot->setNextState(STATE_MANUAL, 0);          
    robot->motorLeftSpeedRpmSet  = robot->motorSpeedMaxRpm;
    robot->motorRightSpeedRpmSet = robot->motorSpeedMaxRpm;
    sendManualMenu(true);
  }
  else if (pfodCmd == "nb")
  {
    // manual: reverse
    robot->setNextState(STATE_MANUAL, 0);          
    robot->motorLeftSpeedRpmSet  = -robot->motorSpeedMaxRpm;
    robot->motorRightSpeedRpmSet = -robot->motorSpeedMaxRpm;
    sendManualMenu(true);
  }
  else if (pfodCmd == "nm")
  {
    // manual: mower ON/OFF
    robot->motorMowEnable = !robot->motorMowEnable;            
    sendManualMenu(true);
  }
  else if (pfodCmd == "ns")
  {
    // manual: stop
    //setNextState(STATE_OFF, 0);          
    robot->motorLeftSpeedRpmSet  =  robot->motorRightSpeedRpmSet = 0;      
    sendManualMenu(true);
  }  
}

void RemoteControl::processSettingsMenu(String pfodCmd)
{   
  if (pfodCmd == "s1")
    sendMotorMenu(false);
  else if (pfodCmd == "s2")
    sendMowMenu(false);
  else if (pfodCmd == "s3") 
    sendBumperMenu(false);
  else if (pfodCmd == "s4")
    sendSonarMenu(false);
  else if (pfodCmd == "s5")
    sendPerimeterMenu(false);
  else if (pfodCmd == "s6")
    sendLawnSensorMenu(false);
  else if (pfodCmd == "s7")
    sendImuMenu(false);
  else if (pfodCmd == "s8")
    sendRemoteMenu(false);
  else if (pfodCmd == "s9")
    sendBatteryMenu(false);
  else if (pfodCmd == "s10")
    sendStationMenu(false);
  else if (pfodCmd == "s11")
    sendOdometryMenu(false);
  else if (pfodCmd == "s12")
    sendDateTimeMenu(false);      
  else if (pfodCmd == "s13")
    sendRainMenu(false);            
  else if (pfodCmd == "s15")
    sendDropMenu(false);
  else if (pfodCmd == "s14")
    sendGPSMenu(false);
  else if (pfodCmd == "sx")
    sendFactorySettingsMenu(false);
  else if (pfodCmd == "sz")
  {
    robot->saveUserSettings();
    sendSettingsMenu(true);
  }
  else sendSettingsMenu(true);  
}      

// process pfodState
void RemoteControl::run()
{  
  if (pfodState == PFOD_LOG_SENSORS)
  {
    Serial1.print((float(millis())/1000.0f));
    Serial1.print(",");
    Serial1.print(robot->motorLeftSense);
    Serial1.print(",");
    Serial1.print(robot->motorRightSense);
    Serial1.print(",");
    Serial1.print(robot->motorMowSense);
    Serial1.print(",");
    Serial1.print(robot->sonarDistLeft);
    Serial1.print(",");
    Serial1.print(robot->sonarDistCenter);
    Serial1.print(",");
    Serial1.print(robot->sonarDistRight);
    Serial1.print(",");
    Serial1.print(robot->perimeter.isInside(0));
    Serial1.print(",");
    Serial1.print(robot->perimeterMag);
    Serial1.print(",");
    Serial1.print(robot->odometryLeft);
    Serial1.print(",");
    Serial1.print(robot->odometryRight);
    Serial1.print(",");
    Serial1.print(robot->imu.ypr.yaw/PI*180);
    Serial1.print(",");
    Serial1.print(robot->imu.ypr.pitch/PI*180);
    Serial1.print(",");
    Serial1.print(robot->imu.ypr.roll/PI*180);
    Serial1.print(",");
    Serial1.print(robot->imu.gyro.x/PI*180);
    Serial1.print(",");
    Serial1.print(robot->imu.gyro.y/PI*180);
    Serial1.print(",");
    Serial1.print(robot->imu.gyro.z/PI*180);
    Serial1.print(",");
    Serial1.print(robot->imu.acc.x);
    Serial1.print(",");
    Serial1.print(robot->imu.acc.y);
    Serial1.print(",");
    Serial1.print(robot->imu.acc.z);
    Serial1.print(",");
    Serial1.print(robot->imu.com.x);
    Serial1.print(",");
    Serial1.print(robot->imu.com.y);
    Serial1.print(",");
    Serial1.print(robot->imu.com.z);
    Serial1.print(",");
    float lat, lon;
    unsigned long age;
    robot->gps.f_get_position(&lat, &lon, &age);
    Serial1.print(robot->gps.hdop());
    Serial1.print(",");
    Serial1.print(robot->gps.satellites());
    Serial1.print(",");
    Serial1.print(robot->gps.f_speed_kmph());
    Serial1.print(",");
    Serial1.print(robot->gps.f_course());
    Serial1.print(",");
    Serial1.print(robot->gps.f_altitude());
    Serial1.print(",");
    Serial1.print(lat);
    Serial1.print(",");
    Serial1.print(lon);
    Serial1.println();
  }
  else if (pfodState == PFOD_PLOT_BAT)
  {
    if (millis() >= nextPlotTime)
    {
      nextPlotTime = millis() + 60000;
      Serial1.print(((unsigned long)millis()/60000));
      Serial1.print(",");
      Serial1.print(robot->batVoltage);
      Serial1.print(",");
      Serial1.print(robot->chgVoltage);
      Serial1.print(",");
      Serial1.print(robot->chgCurrent);
      Serial1.print(",");
      Serial1.println(robot->batCapacity);
    }
  }
  else if (pfodState == PFOD_PLOT_ODO2D)
  {
    if (millis() >= nextPlotTime)
    {
      nextPlotTime = millis() + 500;
      Serial1.print(robot->odometryX);
      Serial1.print(",");
      Serial1.println(robot->odometryY);
    }
  }
  else if (pfodState == PFOD_PLOT_IMU)
  {
    if (millis() >= nextPlotTime)
    {
      nextPlotTime = millis() + 200;
      Serial1.print((float(millis())/1000.0f));
      Serial1.print(",");
      Serial1.print(robot->imu.ypr.yaw/PI*180);
      Serial1.print(",");
      Serial1.print(robot->imu.ypr.pitch/PI*180);
      Serial1.print(",");
      Serial1.print(robot->imu.ypr.roll/PI*180);
      Serial1.print(",");
      Serial1.print(robot->imu.gyro.x/PI*180);
      Serial1.print(",");
      Serial1.print(robot->imu.gyro.y/PI*180);
      Serial1.print(",");
      Serial1.print(robot->imu.gyro.z/PI*180);
      Serial1.print(",");
      Serial1.print(robot->imu.acc.x);
      Serial1.print(",");
      Serial1.print(robot->imu.acc.y);
      Serial1.print(",");
      Serial1.print(robot->imu.acc.z);
      Serial1.print(",");
      Serial1.print(robot->imu.com.x);
      Serial1.print(",");
      Serial1.print(robot->imu.com.y);
      Serial1.print(",");
      Serial1.println(robot->imu.com.z);
    }
  }
  else if (pfodState == PFOD_PLOT_SENSOR_COUNTERS)
  {
    if (millis() >= nextPlotTime)
    {
      nextPlotTime = millis() + 200;
      Serial1.print((float(millis())/1000.0f));
      Serial1.print(",");
      Serial1.print(robot->stateCurr);
      Serial1.print(",");
      Serial1.print(robot->motorLeftSenseCounter);
      Serial1.print(",");
      Serial1.print(robot->motorRightSenseCounter);
      Serial1.print(",");
      Serial1.print(robot->motorMowSenseCounter);
      Serial1.print(",");
      Serial1.print(robot->bumperLeftCounter);
      Serial1.print(",");
      Serial1.print(robot->bumperRightCounter);
      Serial1.print(",");
      Serial1.print(robot->sonarDistCounter);
      Serial1.print(",");
      Serial1.print(robot->perimeterCounter);
      Serial1.print(",");
      Serial1.print(robot->lawnSensorCounter);
      Serial1.print(",");
      Serial1.print(robot->rainCounter);
      Serial1.print(",");
      Serial1.print(robot->dropLeftCounter);
      Serial1.print(",");
      Serial1.println(robot->dropRightCounter);
    }
  }
  else if (pfodState == PFOD_PLOT_SENSORS)
  {
    if (millis() >= nextPlotTime)
    {
      nextPlotTime = millis() + 200;
      Serial1.print((float(millis())/1000.0f));
      Serial1.print(",");
      Serial1.print(robot->stateCurr);
      Serial1.print(",");
      Serial1.print(robot->motorLeftSense);
      Serial1.print(",");
      Serial1.print(robot->motorRightSense);
      Serial1.print(",");
      Serial1.print(robot->motorMowSense);
      Serial1.print(",");
      Serial1.print(robot->sonarDistLeft);
      Serial1.print(",");
      Serial1.print(robot->sonarDistCenter);
      Serial1.print(",");
      Serial1.print(robot->sonarDistRight);
      Serial1.print(",");
      Serial1.print(robot->perimeter.isInside(0));
      Serial1.print(",");
      Serial1.print(robot->lawnSensor);
      Serial1.print(",");
      Serial1.print(robot->rain);
      Serial1.print(",");
      Serial1.print(robot->dropLeft);
      Serial1.print(",");
      Serial1.println(robot->dropRight);
    }
  }
  else if (pfodState == PFOD_PLOT_PERIMETER)
  {    
    if (millis() >= nextPlotTime)
    {
      if (perimeterCaptureIdx>=RAW_SIGNAL_SAMPLE_SIZE*3)
        perimeterCaptureIdx = 0;

      if (perimeterCaptureIdx == 0)
      {
        // Get new Perimeter sample to plot
        memcpy(perimeterCapture, robot->perimeter.getRawSignalSample(0), RAW_SIGNAL_SAMPLE_SIZE);
      }

      nextPlotTime = millis() + 200;
      Serial1.print(perimeterCapture[perimeterCaptureIdx / 3]);
      Serial1.print(",");
      Serial1.print(robot->perimeterMag);
      Serial1.print(",");
      Serial1.print(robot->perimeter.getSmoothMagnitude(0));
      Serial1.print(",");
      Serial1.print(robot->perimeter.isInside(0));
      Serial1.print(",");
      Serial1.print(robot->perimeterCounter);
      Serial1.print(",");
      Serial1.print(!robot->perimeter.signalTimedOut(0));
      Serial1.print(",");
      Serial1.println(robot->perimeter.getFilterQuality(0));
      perimeterCaptureIdx++;
    }
  }
  else if (pfodState == PFOD_PLOT_GPS)
  {
    if (millis() >= nextPlotTime)
    {
      nextPlotTime = millis() + 200;
      float lat, lon;
      unsigned long age;
      robot->gps.f_get_position(&lat, &lon, &age);
      Serial1.print((float(millis())/1000.0f));
      Serial1.print(",");
      Serial1.print(robot->gps.hdop());
      Serial1.print(",");
      Serial1.print(robot->gps.satellites());
      Serial1.print(",");
      Serial1.print(robot->gps.f_speed_kmph());
      Serial1.print(",");
      Serial1.print(robot->gps.f_course());
      Serial1.print(",");
      Serial1.print(robot->gps.f_altitude());
      Serial1.print(",");
      Serial1.print(lat);
      Serial1.print(",");
      Serial1.println(lon);
    }
  }
  else if (pfodState == PFOD_PLOT_GPS2D)
  {
    if (millis() >= nextPlotTime)
    {
      nextPlotTime = millis() + 500;
      Serial1.print(robot->gpsX);
      Serial1.print(",");
      Serial1.println(robot->gpsY);
    }
  }
  else if (pfodState == PFOD_PLOT_MOTOR)
  {
    if (millis() >= nextPlotTime)
    {
      nextPlotTime = millis() + 50;
      Serial1.print((float(millis())/1000.0f));
      Serial1.print(",");
      Serial1.print(robot->motorLeftRpmCurr);
      Serial1.print(",");
      Serial1.print(robot->motorRightRpmCurr);
      Serial1.print(",");
      Serial1.print(robot->motorLeftSpeedRpmSet);
      //Serial1.print(robot->motorLeftPID.w);
      Serial1.print(",");
      Serial1.print(robot->motorRightSpeedRpmSet);
      //Serial1.print(robot->motorRightPID.w);
      Serial1.print(",");
      Serial1.print(robot->motorLeftPWMCurr);
      Serial1.print(",");
      Serial1.print(robot->motorRightPWMCurr);
      Serial1.print(",");
      Serial1.print(robot->motorLeftPID.eold);
      Serial1.print(",");
      Serial1.println(robot->motorRightPID.eold);
    }
  }
}


// process serial input from pfod App
bool RemoteControl::readSerial()
{
  bool res = false;
  while(Serial1.available() > 0)
  {
    res = true;
    if (Serial1.available() > 0)
    {
      char ch = Serial1.read();
      if (ch == '}')
        pfodCmdComplete = true; 
      else if (ch == '{')
        pfodCmd = "";
      else pfodCmd += ch;                
    }
    
    if (pfodCmdComplete)
    {
      pfodState = PFOD_MENU;    
      if (pfodCmd == ".")
      {
        sendMainMenu(false);       
      }
      else if (pfodCmd == "m1")
      {
          // log raw sensors
          Serial1.println(F("{=Log sensors}"));
          Serial1.print(F("time,leftsen,rightsen,mowsen,sonleft,soncenter,sonright,"));
          Serial1.print(F("perinside,permag,odoleft,odoright,yaw,pitch,roll,gyrox,gyroy,"));
          Serial1.print(F("gyroz,accx,accy,accz,comx,comy,comz,hdop,sats,gspeed,gcourse,"));
          Serial1.println(F("galt,lat,lon"));
          pfodState = PFOD_LOG_SENSORS;
      }  
      else if (pfodCmd == "y1")
      {
        // plot battery
        Serial1.println(F("{=battery|time min`0|battery V`1|charge V`1|charge A`2|capacity Ah`3}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_BAT;
      }
      else if (pfodCmd == "y2")
      {
        // plot odometry 2d
        Serial1.println(F("{=odometry2d|position`0~~~x|`~~~y}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_ODO2D;
      }
      else if (pfodCmd == "y3")
      {        
        // plot IMU
        Serial1.print(F("{=IMU`60|time s`0|yaw`1~180~-180|pitch`1|roll`1|gyroX`2~90~-90|gyroY`2|gyroZ`2|accX`3~2~-2|accY`3|accZ`3"));
        Serial1.println(F("|comX`4~2~-2|comY`4|comZ`4}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_IMU;
      }
      else if (pfodCmd == "y5")
      {        
        // plot sensor counters
        Serial1.print(F("{=Sensor counters`300|time s`0|state`1|motL`2|motR`3|motM`4|bumL`5|bumR`6"));
        Serial1.println(F("|son`7|peri`8|lawn`9|rain`10|dropL`11|dropR`12}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_SENSOR_COUNTERS;
      }
      else if (pfodCmd == "y6")
      {
        // plot perimeter spectrum
        /*Serial1.print(F("{=Perimeter spectrum`"));
        Serial1.print(Perimeter.getFilterBinCount());
        Serial1.print(F("|freq (Hz)`0|magnitude`0~60~-1|selected band`0~60~-1}"));*/
        Serial1.println(F("{=Perimeter`128|sig`1|mag`2|smag`3|in`4|cnt`5|on`6|qty`7}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_PERIMETER;          
      }
      else if (pfodCmd == "y7")
      {
        // plot sensor values
        Serial1.print(F("{=Sensors`300|time s`0|state`1|motL`2|motR`3|motM`4|sonL`5|sonC`6"));
        Serial1.println(F("|sonR`7|peri`8|lawn`9|rain`10|dropL`11|dropR`12}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_SENSORS;          
      }
      else if (pfodCmd == "y8")
      {
        // plot GPS 
        Serial1.println(F("{=GPS`300|time s`0|hdop`1|sat`2|spd`3|course`4|alt`5|lat`6|lon`7}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_GPS;          
      }
      else if (pfodCmd == "y10")
      {
        // plot GPS 2d
        Serial1.println(F("{=gps2d|position`0~~~x|`~~~y}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_GPS2D;
      }        
      else if (pfodCmd == "c1")
      {
        // ADC calibration          
        ADCMan.calibrate();
        //beep(2, false);      
      }
      else if (pfodCmd == "y11")
      {
        // motor control
        Serial1.println(F("{=Motor control`300|time s`0|lrpm_curr`1|rrpm_curr`2|lrpm_set`3|rrpm_set`4|lpwm`5|rpwm`6|lerr`7|rerr`8}"));
        nextPlotTime = 0;
        pfodState = PFOD_PLOT_MOTOR;
      }              
      else if (pfodCmd == "yp") sendPlotMenu(false);
      else if (pfodCmd == "y4")sendErrorMenu(false);
      else if (pfodCmd == "y9")sendADCMenu(false);
      else if (pfodCmd == "n") sendManualMenu(false);
      else if (pfodCmd == "s") sendSettingsMenu(false);      
      else if (pfodCmd == "r") sendCommandMenu(false);
      else if (pfodCmd == "c") sendCompassMenu(false);
      else if (pfodCmd == "t") sendDateTimeMenu(false);
      else if (pfodCmd == "i") sendTimerMenu(false);   
      else if (pfodCmd == "in") sendInfoMenu(false);        
      else if (pfodCmd.startsWith("s")) processSettingsMenu(pfodCmd);
      else if (pfodCmd.startsWith("r")) processCommandMenu(pfodCmd);
      else if (pfodCmd.startsWith("c")) processCompassMenu(pfodCmd);
      else if (pfodCmd.startsWith("n")) processManualMenu(pfodCmd);    
      else if (pfodCmd.startsWith("a")) processMotorMenu(pfodCmd);       
      else if (pfodCmd.startsWith("o")) processMowMenu(pfodCmd);       
      else if (pfodCmd.startsWith("b")) processBumperMenu(pfodCmd);       
      else if (pfodCmd.startsWith("d")) processSonarMenu(pfodCmd);       
      else if (pfodCmd.startsWith("e")) processPerimeterMenu(pfodCmd);       
      else if (pfodCmd.startsWith("f")) processLawnSensorMenu(pfodCmd);       
      else if (pfodCmd.startsWith("g")) processImuMenu(pfodCmd);       
      else if (pfodCmd.startsWith("h")) processRemoteMenu(pfodCmd);             
      else if (pfodCmd.startsWith("j")) processBatteryMenu(pfodCmd);       
      else if (pfodCmd.startsWith("k")) processStationMenu(pfodCmd);       
      else if (pfodCmd.startsWith("l")) processOdometryMenu(pfodCmd);  
      else if (pfodCmd.startsWith("m")) processRainMenu(pfodCmd);               
      else if (pfodCmd.startsWith("q")) processGPSMenu(pfodCmd);                       
      else if (pfodCmd.startsWith("t")) processDateTimeMenu(pfodCmd);  
      else if (pfodCmd.startsWith("i")) processTimerMenu(pfodCmd);      
      else if (pfodCmd.startsWith("p")) processTimerDetailMenu(pfodCmd);      
      else if (pfodCmd.startsWith("x")) processFactorySettingsMenu(pfodCmd);
      else if (pfodCmd.startsWith("u")) processDropMenu(pfodCmd);            
      else if (pfodCmd.startsWith("v")) processInfoMenu(pfodCmd);                    
      else if (pfodCmd.startsWith("z")) processErrorMenu(pfodCmd);                    
      else
      {
        // no match
        Serial1.println("{ }");
      }
    
      while (Serial1.available() > 0)
      {
        Console.print(Serial1.read());
      }
      pfodCmd = "";
      pfodCmdComplete = false;

    }
  }  
  return res;
}




