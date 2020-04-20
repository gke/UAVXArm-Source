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

//    UAVX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

#include "UAVX.h"

uint32 SPIErrors = 0;

// imu0Sel,imu1Sel,baroSel,magSel,memSel,gpsSel,rfSel,asSel,flowSel,escSel
// 20 20 20 8 20 20 unused MBaud

const SPIDefStruct SPIDef[] = {
#if defined(USE_21MHZ_SPI)
		{	false, spi_21, spi_0_65625}, // imu0sel
		{	false, spi_21, spi_0_65625}, // imu1sel
		{	false, spi_10_5, spi_10_5}, // baroSel
		{	true, spi_10_5, spi_10_5}, // magSel
#else
		{ false, spi_10_5, spi_0_65625 }, // imu0sel
		{ false, spi_10_5, spi_0_65625 }, // imu1sel
		{ false, spi_10_5, spi_10_5 }, // baroSel
		{ true, spi_5_250, spi_5_250 }, // magSel
#endif
		{ false, spi_10_5, spi_10_5 }, //
		{ false, spi_10_5, spi_10_5 }, //
		{ false, spi_10_5, spi_10_5 }, //
		{ false, spi_10_5, spi_10_5 }, //
		{ false, spi_10_5, spi_10_5 } };

SPI_TypeDef * SPISetBaudRate(uint8 spiDev, boolean R) {
	// It would be good if there was some consistency with SPI protocols!!!
	// All of this for the HMC5983.

	//static uint16 lastDevClockHigh = false;
	uint16 devRate;
	SPI_TypeDef * SPIx;

	SPIx = SPIPorts[busDev[spiDev].busNo].SPIx;

	SPI_Cmd(SPIx, DISABLE);

	devRate = R ? SPIDef[spiDev].ReadRate : SPIDef[spiDev].WriteRate;

	SPIx->CR1 = (SPIx->CR1 & 0b1111111111000111) | devRate;
	Delay1uS(5);
	SPI_Cmd(SPIx, ENABLE);

	return (SPIx);

} // SPISetBaudRate


void SPISelect(uint8 spiDev, boolean Sel) {

	if (Sel) {
		Delay1uS(1);
		DigitalWrite(&busDev[spiDev].P, 0);
	} else
		DigitalWrite(&busDev[spiDev].P, 1);

} // SPISelect


void SPIClearSelects(void) {
	idx i;

	for (i = 0; i < maxDevSel; i++)
		if (busDev[i].Used && (busDev[i].useSPI)) {
			SPISelect(i, false); // TODO do it again but why is this being changed?
			Delay1mS(100);
		}

} // SPIClearSelects


uint8 SPISend(SPI_TypeDef * SPIx, uint8 d) {

	while (!(SPIx->SR & SPI_I2S_FLAG_TXE)) {
	};

	SPIx->DR = d;

	while (!(SPIx->SR & SPI_I2S_FLAG_RXNE)) {
	};

	while (SPIx->SR & SPI_I2S_FLAG_BSY) {
	};

	return SPIx->DR;
} // SPISend


boolean SPIReadBlock(uint8 spiDev, uint8 d, uint8 len, uint8* data) {
	idx i;
	SPI_TypeDef * s;
	uint32 r;
	uint8 Prefix;

	r = SPIErrors;

	// KLUDGE

	if ((spiDev == imu0Sel) || (spiDev == imu1Sel))
		Prefix = 0x80;
	else if (spiDev == magSel) {
		Prefix = (len > 1) ? 0xc0 : 0x80;
	} else
		Prefix = 0;

	s = SPISetBaudRate(spiDev, true);

	SPISelect(spiDev, true);
	SPISend(s, Prefix | d); // MANY devices do not use Read if MSB set so do not OR in here

	for (i = 0; i < len; i++)
		data[i] = SPISend(s, 0);

	SPISelect(spiDev, false);

	return (r == SPIErrors);

} // SPIReadBlock


boolean SPIWriteBlock(uint8 spiDev, uint8 d, uint8 len, uint8 *data) {
	idx i;
	SPI_TypeDef * s;
	uint32 r;

	r = SPIErrors;

	s = SPISetBaudRate(spiDev, false);

	SPISelect(spiDev, true);
	SPISend(s, d);

	for (i = 0; i < len; i++)
		SPISend(s, data[i]);

	SPISelect(spiDev, false);

	return (r == SPIErrors);

} // SPIWriteBlock

