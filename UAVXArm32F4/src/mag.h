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


#ifndef _magnetometer_h
#define _magnetometer_h


#define HMC5XXX_CONFIG_A 	0x00
#define HMC5XXX_CONFIG_B 	0x01
#define HMC5XXX_MODE 		0x02
#define HMC5XXX_DATA 		0x03
#define HMC5XXX_TEMP 		0x31
#define HMC5XXX_STATUS 		0x09
#define HMC5XXX_TAG 		0x0a

enum MagSensors {
	MX, MZ, MY
};

extern int16 RawMag[];
extern real32 MagSamples[][3];
extern uint16 Population[2][2][2];

// HMC5XXX Honeywell Magnetometer

#define IST8310_ID  (0x1c*2)
#define HMC5XXX_ID 	(0x1e*2)

void ReadMagnetometer(void);
void GetMagnetometer(void);
real32 CalculateMagneticHeading(void);

void CalibrateMagnetometer(uint8 s);
void CheckMagnetometerIsCalibrated(void);

void InitMagnetometer(void);
void InitMagnetometerBias(void);

void TrackMaxMin(void);

void MagnetometerTest(uint8 s);
void CheckMagnetometerActive(void);

void WriteMagCalNV(void);
void UpdateMagHist(void);

extern real32 MagVariation, MagVariationWMM2010;
extern real32 MagLockE, MagHeading, InitialMagHeading, Heading, CompassOffset;
extern uint8 CompassType;

extern real32 Mag[];
extern real32 MagTemperature;
extern real32 MagdT;
extern uint16 mm;

#endif

