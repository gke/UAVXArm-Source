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

#ifndef _altitude_h
#define _altitude_h

//#define MS56XX_ID 0xee
#define MS56XX_0_ID (0x77*2)
#define MS56XX_1_ID (0x76*2) //external off i2c port

// baro 20mS acc 2 for V3
#define MAF_ACCU_LEN 10
#define MAF_BARO_LEN 1

#define MAF_ROC_LEN 10

//#define MS56XX_CYCLE_MS 	10
//#define MS56XX_HZ			(1000.0f/MS56XX_CYCLE_MS)

typedef const struct {
	uint16 intervalmS;
	real32 minAlt;
	real32 maxAlt;
} RFStruct;

extern uint8 CurrRFSensorType;
extern uint8 CurrAltSource;

extern RFStruct RF[];

// Measurement Specialities Baro

//#define MS56XX_ID 0xee
#define MS56XX_ID (0x77*2)

void ReadBaroCalibration(void);
real32 CompensateBaro(uint32 ut, uint32 up);
void StartBaro(boolean ReadPressure);
boolean BaroCheckCRC(void);
void GetBaro(void);
real32 GetBaroVariance(void);
void InitBaro(void);

#define MAXSONAR_ID 0xe0
#define SRFSONAR_ID 0xe0

enum RangeFinders {
	MaxSonarcm,
	SRFI2Ccm,
	MaxSonarI2Ccm,
	SharpIRGP2Y0A02YK,
	SharpIRGP2Y0A710K,
	noRF
};


void GetRangefinderAltitude(void);
void InitRangefinder(void);

extern real32 RangefinderAltitude, RangefinderROC;

void UpdateAltitudeEstimates(void);

extern uint16 ms56xx_c[];

void InitAltitude(void);
void SetDesiredAltitude(real32 Desired);
void CaptureDesiredAltitude(real32 Desired);
void ZeroAltitude(void);
void UpdateAltitudeAndROC(void);
void SelectAltitudeSensor(void);
void TrackOriginAltitude(void);
void BaroTest(uint8 s);

extern boolean DEBUGNewBaro;

extern timeuS BaroUpdateTimeuS;
extern real32 BaroOddsEvensFrac;
extern uint32 BaroTempVal, BaroPressVal, BaroVal;
extern real32 BaroPressure, BaroTemperature, CompensatedBaroPressure;
extern boolean AcquiringPressure;
extern real32 DesiredAlt, BaroVariance, OriginAltitude, RawAlt, RawDensityAltitude, DensityAltitude;
extern real32 BaroROC, ROC, ROCTrack;
extern uint8 BaroType;
extern real32 AltdT;
extern uint16 ms56xx_ManufacturersData;

extern filterStruct ROCTrackLPF, ROCLPF, AccUBumpLPF, AccUMAF;
extern filterM3Struct BaroM3F, RangefinderM3F;

#endif

