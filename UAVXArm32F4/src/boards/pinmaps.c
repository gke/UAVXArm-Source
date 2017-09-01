// DO NOT FORMAT  DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT
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

// mpu60xxSel, ms56xxSel, hmc5xxxSel, memSel, gpsSel, rfSel, escSel, flowSel, asSel
// 20 20 8 20 20 unused MBaud

const uint8 spiMap[] = {2, 2, 2, 2, 2, 2, 2, 2, 2}; // SPI2


const spiDefStruct spiDef[] = {
#if defined(GO_FAST) && defined(V4_BOARD)
		{false, spi_21, spi_0_65625},
		{false, spi_10_5, spi_10_5},
		{true, spi_10_5, spi_10_5},
#else
		{false, spi_10_5, spi_0_65625},
		{false, spi_10_5, spi_10_5},
		{true, spi_5_250, spi_5_250},
#endif
		{false, spi_10_5, spi_10_5},
		{false, spi_10_5, spi_10_5},
		{false, spi_10_5, spi_10_5},
		{false, spi_10_5, spi_10_5},
		{false, spi_10_5, spi_10_5}
		};

// mpu60xxSel, ms56xxSel, hmc5xxxSel, memSel, gpsSel, rfSel, escSel, flowSel, assel
#if defined(V4_BOARD)
const uint8 i2cMap[] = {2, 2, 2, 2, 2, 2, 2, 2, 2}; // I2C2
#else
const uint8 i2cMap[] = {2, 2, 2, 2, 2, 2, 2, 2, 2}; // always I2C2
#endif

#if defined(V4_BOARD)
const boolean spiDevUsed[] = {true, true, true, true, false, false, false, false, false};
#include "pinsf4v4.h"
#elif defined(V3_BOARD)
const boolean spiDevUsed[] = {false, false, false, false, false, false, false, false, false};
#include "pinsf4v3.h"
#elif defined(V2_BOARD)
const boolean spiDevUsed[] = {false, false, false, false, false, false, false, false, false};
#include "pinsf4v2.h"
#elif defined(V2_F1_BOARD)
const boolean spiDevUsed[] = {false, false, false, false, false, false, false, false, false};
#include "pinsf1v2.h"
#else
// mbed
#endif
