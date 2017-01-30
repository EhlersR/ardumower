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

#include "flashmem.h"
#include "config.h"

#include <EEPROM.h>

FlashClass Flash;

int eereadwriteString(boolean readflag, int &ee, String& value)
{
  unsigned int i;
  if (readflag)
  {
    value = "";
    char ch = Flash.read(ee++);
    while (ch)
    {
      value += ch;
      ch = Flash.read(ee++);
    }
  }
  else
  {
    for(i=0; i<value.length(); i++)
    {
      Flash.write(ee++, value.charAt(i));
    }
    Flash.write(ee++, 0);
  }
}

FlashClass::FlashClass()
{
  verboseOutput = true;
}

byte FlashClass::read(uint32_t address)
{
  return EEPROM.read(address);
}

byte* FlashClass::readAddress(uint32_t address)
{
  byte d = EEPROM.read(address);
  return &d;
}

void FlashClass::dump()
{
  Console.println(F("EEPROM dump"));
  for (int i=0; i < 1024; i++)
  {
    byte v = read(i);
    if (v != 0)
    {
//      Console.print(i);
//      Console.print(F("="));
//      Console.print(v);
//      Console.print(F(", "));
    }
  }
//  Console.println();
}

boolean FlashClass::write(uint32_t address, byte value)
{
  if (verboseOutput)
  {
//    Console.print(F("!76,"));
//    Console.print(address);
//    Console.print(F(","));
//    Console.print(value);  
//    Console.println();
  }
  EEPROM.write(address, value);
  return true;
}


boolean FlashClass::write(uint32_t address, byte *data, uint32_t dataLength)
{
  for (int i=0; i < dataLength; i++)
  {
    EEPROM.write(address+i, data[i]);    
  }
  return true;
}




