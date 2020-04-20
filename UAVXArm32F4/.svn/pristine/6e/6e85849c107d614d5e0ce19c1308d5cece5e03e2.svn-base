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

uint32 spiErrors = 0;

// imu0Sel,imu1Sel,baroSel,magSel,memSel,gpsSel,rfSel,asSel,flowSel,escSel
// 20 20 20 8 20 20 unused MBaud

#define SPI_CLOCK_SLOW          128 //00.65625 MHz
#define SPI_CLOCK_STANDARD      8   //10.50000 MHz
#define SPI_CLOCK_FAST          4   //21.00000 MHz
#define SPI_CLOCK_ULTRAFAST     2    //42.00000 MHz
//#define SPI_BaudRatePrescaler_32        ((uint16_t)0x0020)
//#define SPI_BaudRatePrescaler_64        ((uint16_t)0x0028)

const SPIDefStruct SPIDef[] = { { false, spi_21, spi_0_65625 }, // imu0sel
		{ false, spi_21, spi_0_65625 }, // imu1sel
		{ false, spi_10_5, spi_10_5 }, // baroSel
		{ true, spi_10_5, spi_10_5 }, // magSel
		{ false, spi_10_5, spi_10_5 }, //
		{ false, spi_10_5, spi_10_5 }, //
		{ false, spi_10_5, spi_10_5 }, //
		{ false, spi_10_5, spi_10_5 }, //
		{ false, spi_10_5, spi_10_5 } };


SPI_TypeDef * SPISetBaudRate(uint8 spiDev, boolean R) {
	// It would be good if there was some consistency with SPI protocols!!!
	// All of this for the HMC5983.

	static uint16 currRate = 0;
	static uint16 lastDevClockHigh = false;
	uint16 devRate;
	SPI_TypeDef * SPIx;

	SPIx = SPIPorts[busDev[spiDev].busNo].SPIx;
	devRate = R ? SPIDef[spiDev].ReadRate : SPIDef[spiDev].WriteRate;

	if (devRate != currRate) {
		currRate = devRate;

		SPI_Cmd(SPIx, DISABLE);

		SPIx->CR1 = (SPIx->CR1 & 0b1111111111000111) | devRate;

		if (SPIDef[spiDev].ClockHigh != lastDevClockHigh) {
			lastDevClockHigh = SPIDef[spiDev].ClockHigh;

			SPIx->CR1 = (SPIx->CR1 & 0b1111111111111100)
					| ((SPIDef[spiDev].ClockHigh) ? SPI_CPOL_High
					& SPI_CPHA_2Edge : SPI_CPOL_Low & SPI_CPHA_1Edge);

			Delay1uS(5);
		}

		SPI_Cmd(SPIx, ENABLE);

	}
	return (SPIx);

} // SPISetBaudRate


void SPISelect(uint8 spiDev, boolean Sel) {

	if (Sel)
		DigitalWrite(&busDev[spiDev].P, 0);
	else {
		DigitalWrite(&busDev[spiDev].P, 1);
		Delay1uS(1);
	}

} // SPISelect


void SPIClearSelects(void) {
	idx i;

	for (i = 0; i < maxDevSel; i++)
		if (busDev[i].Used && (busDev[i].useSPI)) {
			SPISelect(i, false);
			Delay1mS(100);
		}

} // SPIClearSelects

uint8 SPITransfer(SPI_TypeDef *SPIx, uint8 data) {
	uint16 spiTimeout;

	spiTimeout = 0x1000;
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET)
		if ((spiTimeout--) == 0) {
			setStat(SIOFailS, ++spiErrors);
			return (0);
		}

	SPI_I2S_SendData(SPIx, data);

	spiTimeout = 0x1000;
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET)
		if ((spiTimeout--) == 0) {
			setStat(SIOFailS, ++spiErrors);
			return (0);
		}

	return ((uint8) SPI_I2S_ReceiveData(SPIx));
}

boolean SPIReadBlock(uint8 spiDev, uint8 d, uint8 len, uint8* data) {
	idx i;
	SPI_TypeDef * s;
	uint32 r;
	uint8 Prefix;

	r = spiErrors;

	// KLUDGE

	if ((spiDev == imu0Sel) || (spiDev == imu1Sel))
		Prefix = 0x80;
	else if (spiDev == magSel) {
		Prefix = (len > 1) ? 0xc0 : 0x80;
	} else
		Prefix = 0;

	s = SPISetBaudRate(spiDev, true);

	SPISelect(spiDev, true);
	SPITransfer(s, Prefix | d); // MANY devices do not use Read if MSB set so do not OR in here

	for (i = 0; i < len; i++)
		data[i] = SPITransfer(s, 0);

	SPISelect(spiDev, false);

	return (r == spiErrors);

} // SPIReadBlock


boolean SPIWriteBlock(uint8 spiDev, uint8 d, uint8 len, uint8 *data) {
	idx i;
	SPI_TypeDef * s;
	uint32 r;

	r = spiErrors;

	s = SPISetBaudRate(spiDev, false);

	SPISelect(spiDev, true);
	SPITransfer(s, d);

	for (i = 0; i < len; i++)
		SPITransfer(s, data[i]);

	SPISelect(spiDev, false);

	return (r == spiErrors);

} // SPIWriteBlock

void spiBlockTransfer(SPI_TypeDef *SPIx, GPIO_TypeDef* CS_GPIO, uint16 CS_PIN,
		uint8 reg, uint8 len, uint8 * data, boolean read) {
	uint8 i;

	GPIO_ResetBits(CS_GPIO, CS_PIN);

	if (read) {
		SPITransfer(SPIx, reg | 0x80);
		for (i = 0; i < len; i++)
			data[i] = SPITransfer(SPIx, 0x00);
	} else {
		SPITransfer(SPIx, reg);
		for (i = 0; i < len; i++)
			SPITransfer(SPIx, data[i]);
	}

	GPIO_SetBits(CS_GPIO, CS_PIN);
} // spiBlockTransfer


void spiBlockTransfer_i16(SPI_TypeDef *SPIx, GPIO_TypeDef* CS_GPIO,
		uint16 CS_PIN, uint8 reg, uint8 len, int16 * data, boolean read,
		boolean swap) {
	uint8 i, t;

	GPIO_ResetBits(CS_GPIO, CS_PIN);

	if (read) {
		SPITransfer(SPIx, reg | 0x80);
		for (i = 0; i < len; i++)
			if (swap)
				data[i] = (int16) SPITransfer(SPIx, 0x00) << 8 | SPITransfer(
						SPIx, 0x00);
			else {
				t = SPITransfer(SPIx, 0x00);
				data[i] = (int16) SPITransfer(SPIx, 0x00) << 8 | t;
			}
	} else {
		SPITransfer(SPIx, reg);
		for (i = 0; i < len; i++)
			if (swap) {
				SPITransfer(SPIx, data[i] >> 8);
				SPITransfer(SPIx, data[i] & 0xff);
			} else {
				SPITransfer(SPIx, data[i] & 0xff);
				SPITransfer(SPIx, data[i] >> 8);
			}
	}

	GPIO_SetBits(CS_GPIO, CS_PIN);
} // spiBlockTransfer_i16


void spiWrite(SPI_TypeDef *SPIx, GPIO_TypeDef* CS_GPIO, uint16 CS_PIN,
		uint8 reg, uint8 data) {

	GPIO_ResetBits(CS_GPIO, CS_PIN);
	SPITransfer(SPIx, reg); // Accel +/- 4 G Full Scale
	SPITransfer(SPIx, data);
	GPIO_SetBits(CS_GPIO, CS_PIN);

	Delay1uS(1);

} // spiWrite


uint8 spiRead(SPI_TypeDef *SPIx, GPIO_TypeDef* CS_GPIO, uint16 CS_PIN,
		uint8 reg) {
	uint8 data;

	GPIO_ResetBits(CS_GPIO, CS_PIN);

	SPITransfer(SPIx, reg | 0x80); // Accel +/- 4 G Full Scale
	data = SPITransfer(SPIx, 0x00);

	GPIO_SetBits(CS_GPIO, CS_PIN);

	Delay1uS(1);

	return data;
} // spiRead
