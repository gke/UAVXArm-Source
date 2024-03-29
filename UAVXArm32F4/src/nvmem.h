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


#ifndef _nonvolatile_h
#define _nonvolatile_h

#define NV_FLASH_SIZE 16384 // TODO: single sector

#define PARAMS_ADDR_NV		0		// code assumes zero!
#define MAX_PARAMETERS		128		// parameters in EEPROM start at zero
#define NO_OF_PARAM_SETS	8
#define MAX_STATS			32 // x 16bit


typedef struct {
	uint8 Calibrated; // exactly = 1 => calibrated because FLASH is initialised to 0xff
	real32 ReferenceTemp;
	real32 Scale[3], Bias[3];
}__attribute__((packed)) AccCalStruct;

typedef struct {
	real32 ReferenceTemp;
	real32 TempGradient[3], Bias[3];
}__attribute__((packed)) GyroCalStruct;

typedef struct {
	uint8 Calibrated; // exactly = 1 => calibrated because FLASH is initialised to 0xff
	real32 Magnitude; // retained for field strength measure - maybe used for gain setting
	real32 Bias[3];
} MagCalStruct;

typedef struct {
	uint16 CurrRevisionNo;
	uint8 CurrPS;
	int8 P[NO_OF_PARAM_SETS][MAX_PARAMETERS]; // TODO: drop to a single parameter set
	real32 History[16];
	MagCalStruct MagCal;
	AccCalStruct AccCal;
	GyroCalStruct GyroCal;

	MissionStruct Mission;
	uint8 CheckSum;
}__attribute__((packed)) ConfigStruct;

void RefreshConfig(void);
int8 ReadConfig(uint32 a);
boolean ConfigUninitialised(void);

boolean EraseNVMem(void);

void ReadBlockNVMem(uint32 a, uint16 l, int8 * v);
boolean WriteNVMemBlock(uint32 a, uint16 l, int8 *v);
void InitNVMem(void);

void ShowStatusNVMem(uint8 s);

int8 ReadNVMem(uint32 a);
int16 Read16NVMem(uint32 a);
int32 Read32NVMem(uint32 a);
boolean WriteNVMem(uint32 a, int8 d);
boolean Write16NVMem(uint32 a, int16 d);
boolean Write32NVMem(uint32 a, int32 d);

extern uint32 NVMemBlockSize;
extern uint8 NVMemBuffer[];

extern uint32 NVMemSize;
extern uint32 CurrNVMemAddr;

#endif

