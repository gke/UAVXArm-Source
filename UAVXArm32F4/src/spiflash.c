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

#define SPIFLASH_SIZE 16777216
#define SPIFLASH_PAGE_SIZE 256
#define SPIFLASH_BUFFER_SIZE 256

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
uint8 SPIFlashInfo[5] = { 0xff };
uint8 SPIFlashStatus1, SPIFlashStatus2;

boolean InitSPIFlash(void) {
	boolean r;

	ResetSPIFlash(memSel);
	r = DeviceInfoValidSPIFlash(memSel);
	r &= Config256SPIFlash(memSel); // SPIFLASH_PAGE_SIZE

	return(r);
} // InitSPIFlash


void ShowSPIFlashStatus(uint8 s) {
	idx i;

	TxString(s, "0x");
	TxValH(s, SPIFlashStatus1);
	TxChar(s, ' ');
	for (i = 0; i < 5; i++) {
		TxString(s, "0x");
		TxValH(s, SPIFlashInfo[i]);
		TxChar(s, ' ');
	}

} // ShowStatusSPIFlash


boolean FlagSetSPIFlash(uint8 devSel, uint8 fb) {

	return ((ReadStatusSPIFlash(devSel) & (1 << fb)) != 0);

} // FlagSetSPIFlash


void SendSPIFlashAddress(SPI_TypeDef * s, uint32 a) {
	oxo = SPITransfer(s, (uint8) (a >> 16));
	oxo = SPITransfer(s, (uint8) (a >> 8));
	oxo = SPITransfer(s, (uint8) a);
} // SendSPIFlashAddress

//___________________________________________________________________

// READ


uint8 ReadStatusSPIFlash(uint8 devSel) {
	SPI_TypeDef * s;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	s = SPISetBaudRate(devSel, true);

	SPISelect(devSel, true);
	oxo = SPITransfer(s, STATUS_FL);
	SPIFlashStatus1 = SPITransfer(s, 0);
	//FLASHStatus2 = SPITransfer(s, 0);
	SPISelect(devSel, false);

	return(SPIFlashStatus1);

} // ReadStatusSPIFlash

boolean DeviceInfoValidSPIFlash(uint8 devSel) {
	const uint8 expFLASHInfo[5] = { 0x1f, 0x28, 0x00, 0x01, 0x00 };
	idx i;
	SPI_TypeDef * s;
	boolean r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	s = SPISetBaudRate(devSel, true);

	SPISelect(devSel, true);
	oxo = SPITransfer(s, DEV_FL);
	for (i = 0; i < 5; i++)
		SPIFlashInfo[i] = SPITransfer(s, 0);
	SPISelect(devSel, false);

	r = true;
	for (i = 0; i < 5; i++)
		r &= SPIFlashInfo[i] == expFLASHInfo[i];

	return (r);

} // DeviceInfoValidSPIFlash


void ResetSPIFlash(uint8 devSel) {
	SPI_TypeDef * s;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPITransfer(s, 0x80);
	oxo = SPITransfer(s, 0);
	oxo = SPITransfer(s, 0);
	oxo = SPITransfer(s, 0);
	SPISelect(devSel, false);

	uSTimer(MemReady, 35);

} // ResetSPIFlash


boolean ReadSPIFlashPage(uint8 devSel, uint32 a, uint32 len, int8 * data) {
	uint32 i;
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = spiErrors;

	s = SPISetBaudRate(devSel, true);

	SPISelect(devSel, true);
	oxo = SPITransfer(s, RD_FL);
	SendSPIFlashAddress(s, a);
	oxo = SPITransfer(s, 0); // dummy to allow setup
	oxo = SPITransfer(s, 0);
	oxo = SPITransfer(s, 0);
	oxo = SPITransfer(s, 0);
	for (i = 0; i < len; i++)
		data[i] = SPITransfer(s, 0);
	SPISelect(devSel, false);

	return (r == spiErrors);

} // ReadSPIFlashPage

//____________________________________________________________________________

// WRITE


boolean Config256SPIFlash(uint8 devSel) {
	SPI_TypeDef * s;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPITransfer(s, 0x3d);
	oxo = SPITransfer(s, 0x2a);
	oxo = SPITransfer(s, 0x80);
	oxo = SPITransfer(s, 0xa6);
	SPISelect(devSel, false);

	uSTimer(MemReady, 35000);

	return(FlagSetSPIFlash(devSel, FLAG_256));

} // Config256SPIFlash


boolean ReadModifyWriteSPIFlash(uint8 devSel, uint32 a, uint32 len, int8 *data) {
	uint32 i;
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = spiErrors;

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPITransfer(s, RD_MOD_WR_B1_FL);
	SendSPIFlashAddress(s, a);
	for (i = 0; i < len; i++)
		oxo = SPITransfer(s, data[i]);
	SPISelect(devSel, false);

	uSTimer(MemReady, 35000);

	return (r == spiErrors);

} // ReadModifyWriteSPIFlash

//____________________________________________________________________________

// ERASE operations

boolean EraseSPIFlashPage(uint8 devSel, uint32 a) {
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = spiErrors;

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPITransfer(s, ERASE_PAGE_FL);
	SendSPIFlashAddress(s, a);
	SPISelect(devSel, false);

	uSTimer(MemReady, 35000);

	return (r == spiErrors);

} // EraseSPIFlashPage

boolean EraseSPIFlashSector(uint8 devSel, uint32 a) {
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = spiErrors;

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);

	oxo = SPITransfer(s, ERASE_SECTOR_FL);
	SendSPIFlashAddress(s, a);

	SPISelect(devSel, false);

	uSTimer(MemReady, 6500000);

	return (r == spiErrors);

} // EraseSPIFlashSector


boolean EraseSPIFlashXXX(uint8 devSel) {
	SPI_TypeDef * s;
	uint32 r;

	while (uSClock() < uS[MemReady]) {
		// BLOCKING
	};

	r = spiErrors;

	s = SPISetBaudRate(devSel, false);

	SPISelect(devSel, true);
	oxo = SPITransfer(s, 0xc7);
	oxo = SPITransfer(s, 0x94);
	oxo = SPITransfer(s, 0x80);
	oxo = SPITransfer(s, 0x9a);
	SPISelect(devSel, false);

	uSTimer(MemReady, 208000000);

	return (r == spiErrors);

} // Erase

boolean EraseSPIFlash(void) {
	uint16 i;
	boolean r = true;
#if defined(USE_EXT_MEM_ERASE)
	uint32 p;
	uint32 a;
	uint8 d;
	int8 v[256];
#endif

#if defined(USE_EXT_MEM_ERASE)

	if (F.HaveNVMem) {

		for (p = 0; p < (SPIFLASH_SIZE) / SPIFLASH_PAGE_SIZE; p++) {
			a = p * SPIFLASH_PAGE_SIZE;
			// THIS CAN TAKE A VERY LONG TIME ~200Sec.
			SPIFlashReadPage(memSel, a, SPIFLASH_PAGE_SIZE, v);
			d = v[0];
			for (i = 1; i < SPIFLASH_PAGE_SIZE; i++)
			d &= v[i];

			if (d != 0xff) {
				TxChar(0,'*');
				r &= ErasePageSPIFlash(memSel, a);

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

	return (r);
} // EraseSPIFlash
