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


#ifndef _spiFLASH_h
#define _spiFLASH_h

#define FLAG_READY			7
#define FLAG_STALE			6
#define FLAG_PROTECTED		1
#define FLAG_256			0

#define FLAG_EPE			5

boolean FLASHInit(void);
boolean FLASHConfig256(uint8 devSel);
void FLASHReset(uint8 devSel);
uint8 FLASHReadStatus(uint8 devSel);
boolean FLASHFlagSet(uint8 devSel, uint8 fb);
boolean FLASHDeviceInfoValid(uint8 devSel);
void FLASHShowStatus(uint8 s);

boolean FLASHErasePage(uint8 devSel, uint32 a);
boolean FLASHEraseBlock(uint8 devSel, uint32 a);
boolean FLASHErase(uint8 devSel);

boolean FLASHReadPage(uint8 devSel, uint32 FLASHAddr, uint32 len, int8 *data);
boolean FLASHReadModifyWrite(uint8 devSel, uint32 a, uint32 len, int8 *data);

extern uint8 FLASHInfo[];
extern uint16 FLASHLastStatus;

#endif
