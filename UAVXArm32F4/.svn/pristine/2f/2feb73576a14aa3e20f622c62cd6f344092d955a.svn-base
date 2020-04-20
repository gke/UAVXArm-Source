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

boolean SIOReadBlock(uint8 sioDev, uint8 reg, uint8 len, uint8 * data) {

	if (busDev[sioDev].useSPI)
		return (SPIReadBlock(sioDev, reg, len, data));
	else
		return (I2CReadBlock(sioDev, busDev[sioDev].i2cId, reg, len, data));

} // SIOReadBlock


boolean SIOWriteBlock(uint8 sioDev, uint8 reg, uint8 len, uint8 * data) {

	if (busDev[sioDev].useSPI)
		return (SPIWriteBlock(sioDev, reg, len, data));
	else
		return (I2CWriteBlock(sioDev, busDev[sioDev].i2cId, reg, len, data));

} // SIOWriteBlock


// derivative


uint8 SIORead(uint8 sioDev, uint8 reg) {
	uint8 v;

	SIOReadBlock(sioDev, reg, 1, &v);
	return (v);
} // I2CRead

boolean SIOWrite(uint8 sioDev, uint8 reg, uint8 data) {

	return (SIOWriteBlock(sioDev, reg, 1, &data));
} // SIOWrite


uint8 SIOReadataddr(uint8 sioDev, uint8 a) {
	uint8 v;

	SIOReadBlock(sioDev, a, 1, &v);

	return (v);
} // readataddr


boolean SIOReadBlockataddr(uint8 sioDev, uint8 a, uint8 l, uint8 *S) {

	if (busDev[sioDev].useSPI)
		return (SIOReadBlock(sioDev, a, l, S)); // d & 0xfe,
	else
		return (I2CReadBlock(busDev[sioDev].busNo, busDev[sioDev].i2cId & 0xfe,
				a, l, S)); //
} // blockreadataddr


boolean SIOReadBlocki16v(uint8 sioDev, uint8 l, int16 *v, boolean h) {
	uint8 ll, b;
	uint8 S[32];
	boolean r;

	ll = l * 2;
	r = SIOReadBlock(sioDev, 0, ll, S);

	if (h) {
		for (b = 0; b < ll; b += 2)
			v[b >> 1] = ((int16) S[b] << 8) | S[b + 1];
	} else {
		for (b = 0; b < ll; b += 2)
			v[b >> 1] = ((int16) S[b + 1] << 8) | S[b];
	}

	return (r);
} // readi16v

boolean SIOReadBlocki16vataddr(uint8 sioDev, uint8 a, uint8 l, int16 *v,
		boolean h) {
	uint8 b, ll;
	uint8 S[32];

	ll = l * 2;
	SIOReadBlockataddr(sioDev, a, ll, S);

	if (h) {
		for (b = 0; b < ll; b += 2)
			v[b >> 1] = ((int16) S[b] << 8) | S[b + 1];
	} else {
		for (b = 0; b < ll; b += 2)
			v[b >> 1] = ((int16) S[b + 1] << 8) | S[b];
	}

	return (true);
} // readi16v

boolean SIOWritebyte(uint8 sioDev, uint8 v) {

	return (SIOWrite(sioDev, 0, v));
} // writeataddr

boolean SIOWriteataddr(uint8 sioDev, uint8 a, uint8 v) {

	return (SIOWrite(sioDev, a, v));
} // writeataddr


boolean SIOWriteBlockataddr(uint8 sioDev, uint8 a, uint8 l, uint8 *S) {
	idx i;
	boolean r = true;

	for (i = 0; i < l; i++)
		r &= SIOWrite(sioDev, a++, *S++);

	return (r);
} // SIOWriteBlockataddr

boolean SIOResponse(uint8 sioDev) {
	// returns true unless there is an I2C timeout????
	uint8 v;

	v = 77;
	return (SIOReadBlock(sioDev, 0, 1, &v) && (v != 77));

} // response


