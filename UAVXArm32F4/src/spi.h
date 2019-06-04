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


#ifndef _SPI_h
#define _SPI_h

typedef struct {
	boolean ClockHigh;
	uint32 ReadRate;
	uint32 WriteRate;
} SPIDefStruct;


extern const SPIDefStruct SPIDef[];

void SPISelect(uint8 spiDev, boolean Sel);
void SPIClearSelects(void);
SPI_TypeDef * SPISetBaudRate(uint8 spiDevS, boolean R);
void SPISetDivisor(SPI_TypeDef *SPIx, uint16 d);

uint8 SPITransfer(SPI_TypeDef *SPIx, uint8 d);

boolean SPIReadBlock(uint8 spiDev, uint8 reg, uint8 len,
		uint8 * data);
boolean SPIWriteBlock(uint8 spiDev, uint8 reg, uint8 len,
		uint8 * data);

extern uint32 spiErrors;

#endif

