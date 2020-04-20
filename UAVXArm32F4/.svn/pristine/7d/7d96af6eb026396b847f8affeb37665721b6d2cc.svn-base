// ===============================================================================================
// =                                UAVX Quadrocopter ContRoller                                 =
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

//    You should have received a copy of the GNU General Public License aint32 with this program.
//    If not, see http://www.gnu.org/licenses/

#include "UAVX.h"

// CAUTION: Flash does not follow SPI "convention" where MSB signifies a read operation

// TODO: (IntAdr + len) < buffersize

// Read Ops

//#define RD_CONT			0x0b
#define RD_CONT				0x1b
#define ERASE_BLOCK_FL		0x50
//#define RD_PAGE_FL		0x52
#define RD_PAGE_FL_B1 		0x53
//#define RD_B1_HF			0x54
#define RD_PAGE_FL_B2 		0x55
//#define RD_B2_HF			0x56
//#define STATUS_FL			0x57
#define RD_MOD_WR_B1_FL		0x58
#define RD_MOD_WR_B2_FL		0x59
#define CMP_B1_FL    		0x60
#define CMP_B2_FL	    	0x61

//#define RD_CONT			0x68
#define ERASE_SECTOR_FL		0x7c

// Write Ops

#define ERASE_PAGE_FL       0x81
#define WR_VIA_B1_FL		0x82
#define WR_B1_FL_E   		0x83
#define WR_B1				0x84
#define WR_VIA_B2_FL		0x85
#define WR_B2_FL_E   		0x86
#define WR_B2				0x87
#define WR_B1_FL     		0x88
#define WR_B2_FL		    0x89

#define DEV_FL				0x9f

#define RD_FL				0xd2
#define RD_B1_HF			0xd4
#define RD_B2_HF			0xd6
#define STATUS_FL			0xd7

uint8 oxo;
uint8 FLASHInfo[5] = { 0xff };
uint8 FLASHStatus1, FLASHStatus2;

boolean FLASHInit(void) {
	boolean r;

	FLASHReset(memSel);
	r = FLASHDeviceInfoValid(memSel);
	r &= FLASHConfig256(memSel); // MEM_PAGE_SIZE

	return(r);
} // FLASHInit


void FLASHShowStatus(uint8 s) {
	idx i;

	TxString(s, "0x");
	TxValH(s, FLASHStatus1);
	TxChar(s, ' ');
	for (i = 0; i < 5; i++) {
		TxString(s, "0x");
		TxValH(s, FLASHInfo[i]);
		TxChar(s, ' ');
	}

} // FLASHShowStatus


boolean FLASHFlagSet(uint8 devSel, uint8 fb) {

	return ((FLASHReadStatus(devSel) & (1 << fb)) != 0);

} // FLASHFlagSet


void FLASHSendAddress(SPI_TypeDef * s, uint32 a) {
	oxo = SPISend(s, (uint8) (a >> 16));
	oxo = SPISend(s, (uint8) (a >> 8));
	oxo = SPISend(s, (uint8) a);
} // FLASHPageAddress

//___________________________________________________________________

// READ


uint8 FLASHReadStatus(uint8 devSel) {
	SPI_TypeDef * s;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	s = SPISetBaudRate(devSel, true);

	SPISelect(devSel, true);
	oxo = SPISend(s, STATUS_FL);
	FLASHStatus1 = SPISend(s, 0);
	//FLASHStatus2 = SPISend(s, 0);
	SPISelect(devSel, false);

	return(FLASHStatus1);

} // FLASHReadStatus

boolean FLASHDeviceInfoValid(uint8 devSel) {
	const uint8 expFLASHInfo[5] = { 0x1f, 0x28, 0x00, 0x01, 0x00 };
	idx i;
	SPI_TypeDef * s;
	boolean r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	s = SPISetBaudRate(devSel, true);

	SPISelect(devSel, true);
	oxo = SPISend(s, DEV_FL);
	for (i = 0; i < 5; i++)
		FLASHInfo[i] = SPISend(s, 0);
	SPISelect(devSel, false);

	r = true;
	for (i = 0; i < 5; i++)
		r &= FLASHInfo[i] == expFLASHInfo[i];

	return (r);

} // FLASHDeviceInfoValid


void FLASHReset(uint8 devSel) {
	SPI_TypeDef * s;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPISend(s, 0x80);
	oxo = SPISend(s, 0);
	oxo = SPISend(s, 0);
	oxo = SPISend(s, 0);
	SPISelect(devSel, false);

	uSTimer(uSClock(),MemReady, 35);

} // FLASHReset


boolean FLASHReadPage(uint8 devSel, uint32 a, uint32 len, int8 * data) {
	uint32 i;
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = SPIErrors;

	s = SPISetBaudRate(devSel, true);

	SPISelect(devSel, true);
	oxo = SPISend(s, RD_FL);
	FLASHSendAddress(s, a);
	oxo = SPISend(s, 0); // dummy to allow setup
	oxo = SPISend(s, 0);
	oxo = SPISend(s, 0);
	oxo = SPISend(s, 0);
	for (i = 0; i < len; i++)
		data[i] = SPISend(s, 0);
	SPISelect(devSel, false);

	return (r == SPIErrors);

} // FLASHReadPage

//____________________________________________________________________________

// WRITE


boolean FLASHConfig256(uint8 devSel) {
	SPI_TypeDef * s;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPISend(s, 0x3d);
	oxo = SPISend(s, 0x2a);
	oxo = SPISend(s, 0x80);
	oxo = SPISend(s, 0xa6);
	SPISelect(devSel, false);

	uSTimer(uSClock(),MemReady, 35000);

	return(FLASHFlagSet(devSel, FLAG_256));

} // FLASHConfig256


boolean FLASHReadModifyWrite(uint8 devSel, uint32 a, uint32 len, int8 *data) {
	uint32 i;
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = SPIErrors;

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPISend(s, RD_MOD_WR_B1_FL);
	FLASHSendAddress(s, a);
	for (i = 0; i < len; i++)
		oxo = SPISend(s, data[i]);
	SPISelect(devSel, false);

	uSTimer(uSClock(),MemReady, 35000);

	return (r == SPIErrors);

} // FLASHReadModifyWrite

//____________________________________________________________________________

// ERASE operations

boolean FLASHErasePage(uint8 devSel, uint32 a) {
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = SPIErrors;

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPISend(s, ERASE_PAGE_FL);
	FLASHSendAddress(s, a);
	SPISelect(devSel, false);

	uSTimer(uSClock(),MemReady, 35000);

	return (r == SPIErrors);

} // FLASHErasePage

boolean FLASHEraseSector(uint8 devSel, uint32 a) {
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = SPIErrors;

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);

	oxo = SPISend(s, ERASE_SECTOR_FL);
	FLASHSendAddress(s, a);

	SPISelect(devSel, false);

	uSTimer(uSClock(),MemReady, 6500000);

	return (r == SPIErrors);

} // FLASHEraseSector


boolean FLASHErase(uint8 devSel) {
	SPI_TypeDef * s;
	uint32 r;



	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = SPIErrors;

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPISend(s, 0xc7);
	oxo = SPISend(s, 0x94);
	oxo = SPISend(s, 0x80);
	oxo = SPISend(s, 0x9a);
	SPISelect(devSel, false);

	uSTimer(uSClock(),MemReady, 208000000);

	return (r == SPIErrors);

} // FLASHErase
