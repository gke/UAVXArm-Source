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

uint32 NVMemSize;
uint32 NVMemBlockSize;
uint8 NVMemBuffer[512]; // zzz ???

boolean WriteNVMemBlock(uint32 a, uint16 l, int8 *v) {
	boolean r;

	if (F.HaveNVMem)
		switch (busDev[memSel].type) {
		case i2cEEPROMMem:
			r = WriteI2CEEPROMBlock(a, l, v);
			break;
		case spiFlashMem:
			r = ReadModifyWriteSPIFlash(memSel, a, l, v);
			break;
		default:
			r = WriteBlockArmFlash(false, BLACKBOX_FLASH_SECTOR,
					BLACKBOX_FLASH_ADDR + a, l, v);
			break;
		}
	else
		r = false;

	return r;

} // WriteNVMemBlock


void ReadBlockNVMem(uint32 a, uint16 l, int8 * v) {
	uint32 i;

	if (F.HaveNVMem) {
		switch (busDev[memSel].type) {
		case i2cEEPROMMem:
			ReadI2CEEPROMBlock(a, l, v);
			break;
		case spiFlashMem:
			ReadSPIFlashPage(memSel, a, l, v);
			break;
		case ArmFlashMem:
			ReadBlockArmFlash(BLACKBOX_FLASH_ADDR + a, l, v);
			break;
		default:
			break;
		}
	} else
		for (i = 0; i < l; i++)
			v[i] = 0xff;

} // ReadBlockNVMem


int8 ReadNVMem(uint32 a) {
	int8 v = 0xff;

	switch (busDev[memSel].type) {
	case i2cEEPROMMem:
		ReadI2CEEPROMBlock(a, 1, &v);
		break;
	case spiFlashMem:
		ReadSPIFlashPage(memSel, a, 1, &v);
		break;
	default:
		break;
	}

	return v;
} // ReadNVMem


boolean EraseNVMem(void) {
	boolean v;

	LEDsOffExcept(ledBlueSel);

	switch (busDev[memSel].type) {
	case i2cEEPROMMem:
		v = EraseI2CEEPROM();
		break;
	case spiFlashMem:
		v = EraseSPIFlash();
		break;
	default:
		v = false;
		break;
	}

	return v;
} // EraseNVMem


int16 Read16NVMem(uint32 a) {
	i16i8u u;

	u.i8[0] = ReadNVMem(a);
	u.i8[1] = ReadNVMem(a + 1);

	return (u.i16);
} // Read16EE

int32 Read32NVMem(uint32 a) {
	i32i8u u;

	u.i8[0] = ReadNVMem(a);
	u.i8[1] = ReadNVMem(a + 1);
	u.i8[2] = ReadNVMem(a + 2);
	u.i8[3] = ReadNVMem(a + 3);

	return (u.i32);

} // Read32EE

boolean WriteNVMem(uint32 a, int8 d) {
	boolean r = true;

#ifndef COMMISSIONING_TEST
	if (ReadNVMem(a) != d) // avoid redundant writes
#endif
		r = WriteNVMemBlock(a, 1, &d);
	return (r);
} // WriteEE


boolean Write16NVMem(uint32 a, int16 d) {
	boolean r = true;
	i16i8u u;

	if (Read16NVMem(a) != d) {
		u.i16 = d;
		r = WriteNVMemBlock(a, 2, u.i8);
	}

	return (r);
} // Write16EE

boolean Write32NVMem(uint32 a, int32 d) {
	boolean r = true;
	i32i8u u;

	if (Read32NVMem(a) != d) {
		u.i32 = d;
		r = WriteNVMemBlock(a, 4, u.i8);
	}
	return (r);
} // Write16EE


void InitNVMem(void) {
	int8 v, SaveVal;

	if (busDev[memSel].Used) {

		uS[MemReadyuS] = uSClock();

		switch (busDev[memSel].type) {
		case i2cEEPROMMem:
			NVMemSize = 65536;
			NVMemBlockSize = 32;

			F.HaveNVMem = true;

			SaveVal = ReadNVMem(0);

			v = 0b01010101;
			WriteNVMem(0, v);
			v = 0;
			v = ReadNVMem(0);

			WriteNVMem(0, SaveVal);

			F.HaveNVMem = v == 0b01010101;
			break;
		case spiFlashMem:
			NVMemSize = 8388608; //8Mbyte
			NVMemBlockSize = 256;
			F.HaveNVMem = InitSPIFlash();
			break;
		case ArmFlashMem:
			NVMemSize = BLACKBOX_FLASH_SIZE;
			NVMemBlockSize = 256; // zzz ??? TODO:
			EraseArmFlash(BLACKBOX_FLASH_SECTOR);
			F.HaveNVMem = true;
			break;
		default:
			NVMemSize = 0;
			F.HaveNVMem = false;
		}
	} else {
		NVMemSize = 0;
		F.HaveNVMem = false;
	}

} // InitNVMem

void ShowStatusNVMem(uint8 s) {
	if (busDev[memSel].Used && (busDev[memSel].type == spiFlashMem))
		ShowStatusSPIFlash(s);

} // ShowStatusMem


