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

#if !defined(UAVXF4V3)

boolean WriteBlockExtMem(uint32 a, uint16 l, int8 *v) {

	if (F.HaveExtMem)
		FLASHReadModifyWrite(memSel, a, l, v);

	return (F.HaveExtMem);
} // WriteBlockMem

boolean EraseExtMem(void) {
	uint16 i;
	boolean r = true;
#if defined(USE_EXT_MEM_ERASE)
	uint32 p;
	uint32 a;
	uint8 d;
	int8 v[256];
#endif

	SaveLEDs();
	LEDsOff();
	LEDOn(ledBlueSel);

#if defined(USE_EXT_MEM_ERASE)

	if (F.HaveExtMem) {

		for (p = 0; p < (MEM_SIZE) / MEM_PAGE_SIZE; p++) {
			a = p * MEM_PAGE_SIZE;
			// THIS CAN TAKE A VERY LONG TIME ~200Sec.
			FLASHReadPage(memSel, a, MEM_PAGE_SIZE, v);
			d = v[0];
			for (i = 1; i < MEM_PAGE_SIZE; i++)
			d &= v[i];

			if (d != 0xff) {
				TxChar(0,'*');
				r &= FLASHErasePage(memSel, a);

				LEDToggle(ledBlueSel);
				LEDToggle(ledYellowSel);
			} else {
				uSTimer(uSClock(), MemReady, 0);
			}
		}
	}

#else

	for (i = 0; i < 10; i++) {
		Delay1mS(100);
		LEDToggle(ledBlueSel);
		LEDToggle(ledYellowSel);
	}

#endif

	RestoreLEDs();

	return (r);

} // EraseMem


void ReadBlockExtMem(uint32 a, uint16 l, int8 * v) {
	uint16 i;

	if (F.HaveExtMem)
		FLASHReadPage(memSel, a, l, v);
	else
		for (i = 0; i < l; i++)
			v[i] = 0xff;

} // ReadBlockMem


#else

// EEPROM

boolean WriteBlockExtMem(uint32 a, uint16 l, int8 *v) {
	boolean r = true;
	uint16 i;
	uint8 b[66]; // I2C implementation restriction
	uint8 bank;
	i8u8u u;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};
	if (F.HaveExtMem) {
		bank = 0;// only one chip (a & 0x00070000) >> 15;
		b[0] = (a >> 8) & 0xff;
		b[1] = a & 0xff; // messy - i2c routines use 8 bit register#
		for (i = 0; i < l; i++) {
			u.i8 = v[i];
			b[i + 2] = u.u8;
		}

		r &= I2CWriteBlock(busDev[memSel].busNo, EEPROM_ID | bank, 0xff, 2 + l, b);
	} else
	r = false;
	uSTimer(uSClock(), MemReady, 5000);

	return (r);

} // WriteBlockMem

boolean EraseExtMem(void) {
	timemS TimeoutmS;
	uint32 a;
	int8 B[MEM_BUFFER_SIZE];
	boolean r = true;

	SaveLEDs();
	LEDsOff();
	LEDOn(ledBlueSel);

	if (F.HaveExtMem) {
		memset(&B, -1, sizeof(B));
		TimeoutmS = mSClock();
		for (a = 0; a < MEM_SIZE; a += MEM_BUFFER_SIZE) {
			r &= WriteBlockExtMem(a, MEM_BUFFER_SIZE, B);

			if (mSClock() > TimeoutmS) {
				TimeoutmS += 100;
				LEDToggle(ledBlueSel);
				LEDToggle(ledYellowSel);
			}
		}
	} else
	r = false;

	RestoreLEDs();

	return (r);

} // EraseExtMem


void ReadBlockExtMem(uint32 a, uint16 l, int8 * v) {
	uint16 i;
	uint8 bank;
	uint8 b[2];

	if (F.HaveExtMem) {
		while (uSClock() < uS[MemReady]) {
			// BLOCKING
		};
		bank = 0; // only one chip (a & 0x00070000) >> 15;
		b[0] = (a >> 8) & 0xff;
		b[1] = a & 0xff; // messy - sio routines use 8 bit register#
		I2CWriteBlock(busDev[memSel].busNo, EEPROM_ID | bank, 0xff, 2, b);
		I2CReadBlock(busDev[memSel].busNo, EEPROM_ID | bank, 0xff, l, (uint8*) v);
	} else
	for (i = 0; i < l; i++)
	v[i] = 0xff;

} // ReadBlockEE

#endif

int8 ReadExtMem(uint32 a) {
	int8 v = 0xff;

	ReadBlockExtMem(a, 1, &v);
	return (v);
} // ReadEE

int16 Read16ExtMem(uint32 a) {
	i16i8u u;

	u.i8[0] = ReadExtMem(a);
	u.i8[1] = ReadExtMem(a + 1);

	return (u.i16);
} // Read16EE

int32 Read32ExtMem(uint32 a) {
	i32i8u u;

	u.i8[0] = ReadExtMem(a);
	u.i8[1] = ReadExtMem(a + 1);
	u.i8[2] = ReadExtMem(a + 2);
	u.i8[3] = ReadExtMem(a + 3);

	return (u.i32);

} // Read32EE

boolean WriteExtMem(uint32 a, int8 d) {
	boolean r = true;

#ifndef COMMISSIONING_TEST
	if (ReadExtMem(a) != d) // avoid redundant writes
#endif
		r = WriteBlockExtMem(a, 1, &d);
	return (r);
} // WriteEE


boolean Write16ExtMem(uint32 a, int16 d) {
	boolean r = true;
	i16i8u u;

	if (Read16ExtMem(a) != d) {
		u.i16 = d;
		r = WriteBlockExtMem(a, 2, u.i8);
	}

	return (r);
} // Write16EE

boolean Write32ExtMem(uint32 a, int32 d) {
	boolean r = true;
	i32i8u u;

	if (Read32ExtMem(a) != d) {
		u.i32 = d;
		r = WriteBlockExtMem(a, 4, u.i8);
	}
	return (r);
} // Write16EE


void InitExtMem(void) {

	if (busDev[memSel].Used) {

		uS[MemReady] = uSClock();

		int8 v, SaveVal;

		if (busDev[memSel].type == i2cEEPROMMem) {
			F.HaveExtMem = true;
			SaveVal = ReadExtMem(MEM_SIZE - 1);
			v = 0b01010101;
			WriteBlockExtMem(MEM_SIZE - 1, 1, &v);
			v = 0;
			v = ReadExtMem(MEM_SIZE - 1);
			F.HaveExtMem = v == 0b01010101;
			WriteBlockExtMem(MEM_SIZE - 1, 1, &SaveVal);
		} else if (busDev[memSel].type == spiFlashMem)
			F.HaveExtMem = FLASHInit();
		else
			F.HaveExtMem = false;

	} else
		F.HaveExtMem = false;

} // InitMem

void ShowStatusExtMem(uint8 s) {
	if (busDev[memSel].Used && (busDev[memSel].type == spiFlashMem))
		FLASHShowStatus(s);

} // ShowStatusMem


