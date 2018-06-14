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


#ifndef _clocks_h
#define _clocks_h

extern volatile timeuS TicksuS;

void cycleCounterInit(void);
timeuS uSClock(void);
void Delay1uS(uint16 DelayuS);
timemS mSClock(void);
void Delay1mS(uint16 DelaymS);
real32 dTUpdate(timeuS NowuS, timeuS * LastUpdateuS);
void mSTimer(timemS NowmS, uint8 t, timemS TimePeriodmS);
void uSTimer(timeuS NowuS, uint8 t, timeuS TimePerioduS);

#endif
