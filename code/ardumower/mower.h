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

#ifndef MOWER_H
#define MOWER_H

#include <Arduino.h>
#include "robot.h"
#include "drivers.h"
#include "bt.h"

/*
  Ardumower robot chassis
*/

class Mower : public Robot
{
  public:
    Mower();  
    virtual void setup(void);
    virtual void resetMotorFault();
    virtual int readSensor(char type);
    virtual void setActuator(char type, int value);
    virtual void configureBluetooth(boolean quick);
};


extern Mower robot;

#endif

