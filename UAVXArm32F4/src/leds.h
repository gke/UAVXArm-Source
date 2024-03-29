// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU
//    General Public License as published by the Free Software Foundation, either version 3 of the
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.
//    If not, see http://www.gnu.org/licenses/


#ifndef _leds_h
#define _leds_h

void LEDsOn(void);
void LEDsOff(void);
void LEDToggle(uint8 l);
void LEDOn(uint8 l);
void LEDOff(uint8 l);
void LEDsOffExcept(uint8 l);
void LEDRandom(void);
void LEDChaser(void);

void LEDsAndBuzzer(uint8 s);
void InitLEDs(void);

void BeeperOff(void);
void BeeperOn(void);
void BeeperToggle(void);

extern boolean BeeperIsOn;

#endif

