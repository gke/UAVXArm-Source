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


#include "UAVX.h"

boolean WriteI2CEEPROMBlock(uint32 a, uint16 l, int8 *v) {
	boolean r = true;
	uint16 i;
	uint8 b[66]; // I2C implementation restriction
	uint8 bank;
	i8u8u u;

	if (F.HaveNVMem) {

		while (!uSTimeout(MemReadyuS)) {
			// BLOCKING
		};

		bank = 0;// only one chip (a & 0x00070000) >> 15;
		b[0] = (a >> 8) & 0xff;
		b[1] = a & 0xff; // messy - i2c routines use 8 bit register#
		for (i = 0; i < l; i++) {
			u.i8 = v[i];
			b[i + 2] = u.u8;
		}

		r &= I2CWriteBlock(memSel, busDev[memSel].i2cId | bank, 0xff, 2 + l, b);

	} else
		r = false;
	uSTimer(MemReadyuS, 5000);

	return (r);

} // WriteI2CEEPROMBlock


void ReadI2CEEPROMBlock(uint32 a, uint16 l, int8 * v) {
	uint16 i;
	uint8 bank;
	uint8 b[2];

	if (F.HaveNVMem) {
		while (uSClock() < uS[MemReadyuS]) {
			// BLOCKING
		};

		bank = 0; // only one chip (a & 0x00070000) >> 15;
		b[0] = (a >> 8) & 0xff;
		b[1] = a & 0xff; // messy - sio routines use 8 bit register#

		I2CWriteBlock(memSel, busDev[memSel].i2cId | bank, 0xff, 2, b);
		I2CReadBlock(memSel, busDev[memSel].i2cId | bank, 0xff, l, (uint8*) v);

	} else
		for (i = 0; i < l; i++)
			v[i] = 0xff;

} // ReadI2CEEPROMBlock


boolean EraseI2CEEPROM(void) {
	timemS TimeoutmS;
	uint32 a;
	boolean r = true;

	//memset(&NVMemBuffer, -1, sizeof(NVMemBuffer));
	TimeoutmS = mSClock();
	for (a = 0; a < NVMemSize; a += NVMemBlockSize) {
		r &= WriteNVMemBlock(a, NVMemBlockSize, NVMemBuffer);

		if (mSClock() > TimeoutmS) {
			TimeoutmS += 100;
			LEDToggle(ledBlueSel);
			LEDToggle(ledYellowSel);
		}
	}

	return r;

} // EraseI2CEEPROM


