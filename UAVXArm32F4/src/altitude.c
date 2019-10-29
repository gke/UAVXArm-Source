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

// Barometers

#include "UAVX.h"

boolean DEBUGNewBaro = false; // zzz
uint32 LSBBaro[256] = { 0, };

#define MS56XX_PROM 		0xA0
#define MS56XX_PRESS 		0x40
#define MS56XX_TEMP 		0x50
#define MS56XX_RESET 		0x1E

// OSR (Over Sampling Ratio) constants
#define MS56XX_OSR_256 		0x00 //0.065 mBar  0.6mS
#define MS56XX_OSR_512 		0x02 //0.042 1.17mS
#define MS56XX_OSR_1024 	0x04 //0.027 2.28mS
// DO NOT USE #define MS56XX_OSR_2048 	0x06 //0.018 4.54mS
#define MS56XX_OSR_4096 	0x08 //0.012 9.04mS
const uint32 ms56xxSampleIntervaluS[] = { 1000, 1500, 2500, 5000, 10000 };
//const uint32 ms56xxSampleIntervaluS[] = { 600, 1170, 2280, 4540, 9040 };
timeuS ms56IntervaluS;

#define MS56XXOSR MS56XX_OSR_4096
uint16 ms56xx_c[8];
int64 M[7];

uint16 ms56xx_ManufacturersData;
boolean AcquiringPressure;

timeuS BaroUpdateTimeuS;
real32 BaroOddsEvensFrac = 0.0f;
uint32 BaroTempVal, BaroPressVal, BaroVal;
int16 BaroWarmupCycles;
real32 BaroTemperature, BaroPressure, CompensatedBaroPressure;
timeuS NextBaroUpdateuS;

uint8 CurrRFSensorType;
real32 SensorAltitudeP;
boolean WasUsingRF = false;

real32 OriginAltitude, RawAlt, RawDensityAltitude, DensityAltitude,
		DensityAltitudeP, SensorAltitude, SensorAltitudeP;
real32 ROC, ROCF;

real32 Airspeed;

timemS AltitudemS;

real32 BarodT, AltdT;

uint8 BaroType = ms5611Baro;

real32 FakeDensityAltitude;

real32 CalculateDensityAltitude(boolean FullMath, real32 P) {

	if (FullMath) {
		//return (1.0f - powf((P / 101325.0f), 0.190295f)) * 44330.0f;
		return ((1.0f - powf((P / 101325.0f), 0.19022256f)) * (BaroTemperature
				+ 273.15f)) * 153.846153846f;
	} else
		return (101325.0f - P) * 0.08406f; // ~calibration to 200M // 0.25uS

} // CalculateDensityAltitude


void TrackOriginAltitude(void) {

	WasUsingRF = false;
	OriginAltitude = DensityAltitudeP = DensityAltitude;
	if (F.GPSValid && (GPS.vAcc < GPSMinvAcc))
		GPS.originAltitude = GPS.altitude;

} // TrackOriginAltitude

// -----------------------------------------------------------

// Measurement Specialities MS5611/5607 Barometers


void ReadBaroCalibration(void) {
	idx i;
	uint8 B[2] = { 0, 0 };

	// clumsy but device does not have address auto increment

	for (i = 0; i < 8; i++) {
		SIOReadBlock(baroSel, MS56XX_PROM + i * 2, 2, B);
		ms56xx_c[i] = ((uint16) B[0] << 8) | B[1];
	}

	M[5] = (int64) ms56xx_c[5] << 8;

	M[2] = (int64) ms56xx_c[2] << 16;
	M[4] = (int64) ms56xx_c[4] >> 7;

	M[1] = (int64) ms56xx_c[1] << 15;
	M[3] = (int64) ms56xx_c[3] >> 8;

} // ReadBaroCalibration


real32 CompensateBaro(uint32 ut, uint32 up) {
	int64 off2, sens2, t, off, sens, ms56xxt, ms56xxdt;

	ms56xxdt = ut - M[5];

	//off = M[2] + M[4] * ms56xxdt;
	//sens = M[1] + M[3] * ms56xxdt;
	off = M[2] + (((int64) ms56xx_c[4] * ms56xxdt) >> 7);
	sens = M[1] + (((int64) ms56xx_c[3] * ms56xxdt) >> 8);

	ms56xxt = 2000 + (((int64) ms56xx_c[6] * ms56xxdt) >> 23);

	if (ms56xxt < 2000) { // < 20C

		t = Sqr(ms56xxt - 2000);
		off2 = (5 * t) / 2;
		sens2 = (10 * t) / 8;

		if (ms56xxt < -1500) { // < -15C
			t = Sqr(ms56xxt + 1500);
			off2 += 7 * t;
			sens2 += (11 * t) / 2;
		}

		off -= off2;
		sens -= sens2;
		ms56xxt -= Sqr(ms56xxdt) >> 31;
	}

	BaroTemperature = (real32) ms56xxt * 0.01f;
	CompensatedBaroPressure = (real32) (((((int64) up * sens) >> 21) - off)
			>> 11) * 0.0625f;

	return (CompensatedBaroPressure);

} // CompensateBaro


real32 CompensateBaro2(uint32 ut, uint32 up) {
	int64 ms56xxdt, off, sens, off2, sens2, t;
	int32 ms56xxt;

	ms56xxdt = (int32) ut - ((uint32) ms56xx_c[5] << 8);
	ms56xxt = 2000 + (int32) (((int64) ms56xxdt * ms56xx_c[6]) >> 23);

	if (BaroType == ms5611Baro) {

		off = ((int64) ms56xx_c[2] << 16) + (((int64) ms56xx_c[4] * ms56xxdt)
				>> 7);
		sens = ((int64) ms56xx_c[1] << 15) + (((int64) ms56xx_c[3] * ms56xxdt)
				>> 8);

		if (ms56xxt < 2000) {
			t = Sqr(ms56xxt - 2000);
			off2 = (5 * t) / 2;
			sens2 = (10 * t) / 8;

			if (ms56xxt < -1500) { // < -15C
				t = Sqr(ms56xxt + 1500);
				off2 += 7 * t;
				sens2 += (11 * t) / 2;
			}

			off -= off2;
			sens -= sens2;
			ms56xxt -= Sqr(ms56xxdt) >> 31;
		}

	} else { // MS5607Baro

		off = ((int64) ms56xx_c[2] << 17) + (((int64) ms56xx_c[4] * ms56xxdt)
				>> 6);
		sens = ((int64) ms56xx_c[1] << 16) + (((int64) ms56xx_c[3] * ms56xxdt)
				>> 7);

		if (ms56xxt < 2000) {
			t = Sqr((int64) ms56xxt - 2000);
			off2 = (61 * t) >> 4;
			sens2 = 2 * t;

			if (ms56xxt < -1500) {
				t = Sqr(ms56xxt + 1500);
				off2 += 15 * t;
				sens2 += 8 * t;
			}

			ms56xxt -= Sqr(ms56xxdt) >> 31;
			off -= off2;
			sens -= sens2;
		}
	}

	BaroTemperature = (real32) ms56xxt * 0.01f;
	CompensatedBaroPressure = (real32) (((((int64) up * sens) >> 21) - off)
			>> 11) * 0.0625f;

	return (CompensatedBaroPressure);
} // CompensateBaro2


void StartBaro(boolean ReadPressure) {
	uint8 d = 0;

	if (ReadPressure)
		SIOWriteBlock(baroSel, (MS56XX_PRESS | MS56XXOSR), 0, &d);
	else
		SIOWriteBlock(baroSel, (MS56XX_TEMP | MS56XXOSR), 0, &d);

} // StartBaro


void GetBaro(void) {
	static timeuS LastUpdateuS = 0;
	timeuS IntervaluS, NowuS;
	static real32 BaroPressureP = 0.0f;
	uint8 B[3];

	if (F.BaroActive) {

		if (uSTimeout(NextBaroUpdate)) { // don't use uSTimeout
			uSTimer(NextBaroUpdate, ms56IntervaluS);

			SIOReadBlock(baroSel, 0, 3, B);
			NowuS = uSClock();

			BaroVal = ((uint32) B[0] << 16) | ((uint32) B[1] << 8) | B[2];

			if (AcquiringPressure) {
				LSBBaro[B[2]]++;
				DEBUGNewBaro = true;

				BaroPressVal = BaroVal;
				BaroPressure = CompensateBaro(BaroTempVal, BaroPressVal);

#if defined(UAVXF4V4)
				// TODO: KLUDGE TO FIX "SOME" V4 BOARDS - NOT SIMPLE SIGN REVERSAL THOUGH?
				F.BaroFailure = BaroPressure < 0.0f;
				BaroPressure = F.BaroFailure ? BaroPressureP : BaroPressure;
#endif

				RawAlt = F.Emulation ? FakeAltitude : CalculateDensityAltitude(
						true, BaroPressure);

				if (!isnan(RawAlt)) {

					IntervaluS
							= Limit(NowuS - LastUpdateuS, ms56IntervaluS*2, ms56IntervaluS*3);
					BarodT = (real32) IntervaluS * 0.000001f;
					LastUpdateuS = NowuS;

					RawDensityAltitude = LPFn(&AltitudeLPF, RawAlt, BarodT);

					F.NewBaroValue = true;
				}

				AcquiringPressure = false;
			} else {
				if (BaroVal != 0)
					BaroTempVal = (BaroTempVal == 0) ? BaroVal : ((BaroTempVal
							* 7 + BaroVal) >> 3);

				AcquiringPressure = true;

			}
			StartBaro(AcquiringPressure);
		}
	} else
		RawAlt = 0.0f;

} // GetBaro


boolean BaroCheckCRC(void) {
	uint16 crc = 0, zerocheck = 0;
	idx i, k;
	uint8 crc_save;

	crc_save = (uint8) (ms56xx_c[7] & 0xf); // Save last 4 bit
	ms56xx_c[7] = 0xff00 & ms56xx_c[7]; // Last byte must be cleared for crc calculation
	for (i = 0; i < 16; i++) {
		zerocheck = zerocheck | ms56xx_c[i >> 1]; // To check for empty buffer
		if (i % 2 == 1)
			crc ^= (ms56xx_c[i >> 1]) & 0x00ff;
		else
			crc ^= ms56xx_c[i >> 1] >> 8;
		for (k = 8; k > 0; k--)
			if (crc & (0x8000))
				crc = (crc << 1) ^ 0x3000;
			else
				crc = crc << 1;

	}
	crc = (crc >> 12) & 0xf; // crc = CRC code
	return (crc_save == (uint8) crc && zerocheck != 0);

} // BaroCheckCRC


void CheckBaroActive(void) {
	uint8 B[2] = { 0, 0 };
	uint8 i = 0;

	if (busDev[baroSel].Used) {
		if (busDev[baroSel].type == ms5611Baro) {
			SIOWriteBlock(baroSel, MS56XX_RESET, 0, &i);

			Delay1mS(30); // was 3

			SIOReadBlock(baroSel, MS56XX_PROM, 2, B);
			ms56xx_ManufacturersData = ((uint16) B[0] << 8) | B[1];

			F.BaroActive = true; // startup delay problem zzz ms56xx_ManufacturersData != 0;
		} else
			F.BaroActive = true; // TODO: risky?
	} else
		F.BaroActive = false;

} // CheckBaroActive

void UpdateBaroVariance(real32 Alt) {
	const real32 Alt_K = 0.99f;
	static real32 B[128];
	static idx i = 0;

	if (State != InFlight) {
		B[i++] = Alt;

		if (i >= 128) {
			BaroVariance = BaroVariance * Alt_K + CalculateVariance(B, 128)
					* (1.0f - Alt_K);
			i = 0;
		}
	}

} // UpdateBaroVariance

real32 GetBaroVariance(void) {
	real32 B[128];
	idx i;

	do {
		GetBaro();
		if (F.NewBaroValue) {
			F.NewBaroValue = false;
			B[i++] = RawDensityAltitude;
		}
	} while (i < 128);

	return CalculateVariance(B, 128);

} // GetBaroVariance


void InitBaro(void) {

	ms56IntervaluS = ms56xxSampleIntervaluS[MS56XXOSR >> 1];

	BaroTempVal = BaroPressVal = BaroVal = 0;

	CheckBaroActive();

	if (F.BaroActive) {

		ReadBaroCalibration();

		AcquiringPressure = false; // temperature must be first
		StartBaro(AcquiringPressure);
		NextBaroUpdateuS = uSClock() + ms56IntervaluS;

		BaroWarmupCycles = 0;
		F.NewBaroValue = false;

		do { // just warming up the ms56xx!
			GetBaro();
			if (F.NewBaroValue) {
				F.NewBaroValue = false;
				BaroWarmupCycles++;
			}
		} while (BaroWarmupCycles < 200);

	} else
		RawDensityAltitude = 0.0f;

	AccZBias = 0.0f;
	OriginAltitude = DensityAltitude = DensityAltitudeP = RawDensityAltitude;
	SetDesiredAltitude(0.0f);

	BaroVariance = GetBaroVariance();

} // InitBaro


//____________________________________________________________________________

// rangefinder


RFStruct RF[] = { { 50, 0.4f, 5.0 }, //6.45f }, // MaxSonarcm
		{ 70, 0.2f, 6.45f }, // SRFI2Ccm
		{ 100, 0.2f, 7.65f }, // MaxSonarI2Ccm
		{ 1, 0.2f, 1.5f }, // SharpIRGP2Y0A02YK was 10mS
		{ 1, 1.0f, 5.5f }, // SharpIRGP2Y0A710K was 10mS
		{ 0, 0, 0 } };

void GetRangefinderAltitude(void);
void InitRangefinder(void);

real32 RangefinderAltitude;
timeuS LastRangefinderUpdateuS = 0;
real32 RFdT, RFdTR;
real32 MaxSonarAltitude;

void ReadMaxSonarI2C(void) {
	uint8 B[2];

	SIOReadBlock(rfSel, 0, 2, B); // 0
	RangefinderAltitude = (real32) (((int32) B[0] << 8) + B[1]) * 0.01f;
	SIOWrite(rfSel, 81, 1); // start read cm

} // ReadMaxSonarI2C

void ReadSRFI2C(void) {
	uint8 B[2];

	SIOReadBlock(rfSel, 2, 2, B); // 2 and 3
	RangefinderAltitude = (real32) (((int32) B[0] << 8) + B[1]) * 0.01f;
	SIOWrite(rfSel, 81, 1); // start read cm

} // ReadSRFI2C

real32 SharpRFLookup(real32 r, uint8 Sel) {
#define MEDIAN_LEN 31
	const real32 A[] = { 15.048f, 95.299 };
	const real32 B[] = { -1.16f, -1.9197 };
	//const real32 M[] = { 0.757, 0.666 };

	static int16 s = 0;
	real32 a[MEDIAN_LEN];
	real32 v = 0.0f;

	// MAJOR PROBLEM AS DISTANCE CURVE IS NOT MONOTONIC
	//	if (RF[CurrRFSensorType].Min)

	if (s >= MEDIAN_LEN) {
		v = median(a, 31);
		v = A[Sel] * powf(v, B[Sel]);
		s = 0;
	} else
		a[s++] = r;

	//if (r > M[Sel])
	//	r = M[Sel];

	return (v); // http://www.bot-thoughts.com/2012/09/curve-fitting-sharp-ir-rangers.html

} //  SharpGP2Y0A02YKLookup


void ReadRangefinder(void) {

	switch (CurrRFSensorType) {
	case MaxSonarcm:
		RangefinderAltitude = analogRead(RangefinderAnalogSel) * 10.24f;
		break;
	case SRFI2Ccm:
		ReadSRFI2C();
		break;
	case MaxSonarI2Ccm:
		ReadMaxSonarI2C();
		break;
	case SharpIRGP2Y0A02YK:
		RangefinderAltitude
				= SharpRFLookup(analogRead(RangefinderAnalogSel), 0);
		break;
	case SharpIRGP2Y0A710K:
		RangefinderAltitude
				= SharpRFLookup(analogRead(RangefinderAnalogSel), 1);
		break;
	default:
		RangefinderAltitude = 0.0f;
		break;
	} // switch

} // ReadRangefinder

void GetRangefinderAltitude(void) {
#define RF_BUCKET
#define RF_BUCKET_SIZE 10 // x 50mS
	static int16 RFBucket = 0;
	static boolean RFInRange = false;
	static real32 RangefinderAltitudeP = 0.0f;

	if (F.RangefinderActive) {

		if (mSTimeout(RangefinderUpdate)) {
			mSTimer(RangefinderUpdate, RF[CurrRFSensorType].intervalmS);

			ReadRangefinder();

			RFInRange = RangefinderAltitude <= RF[CurrRFSensorType].maxAlt;

			//RFInRange = (RangefinderAltitude >= RF[CurrRFSensorType].minAlt)
			//		&& (RangefinderAltitude <= RF[CurrRFSensorType].maxAlt);

			//&& F.NearLevel;

#if defined(RF_BUCKET)
			if (RFInRange) {
				RFBucket = Limit(RFBucket + 1, 0, RF_BUCKET_SIZE);
				F.UsingRangefinderAlt = RFBucket >= RF_BUCKET_SIZE;
				RangefinderAltitudeP = RangefinderAltitude;
			} else { // otherwise distrust - returns zero if there is a misrange fault or 1024 out of range
				if ((RFBucket >= (RF_BUCKET_SIZE - 4)) && F.UsingRangefinderAlt) {
					RangefinderAltitude = RangefinderAltitudeP;
					RFBucket -= 2;
				} else {
					RFBucket = 0;
					F.UsingRangefinderAlt = false;
				}
			}
#else
			F.UsingRangefinderAlt = RFInRange;
#endif

		}
	} else
		F.UsingRangefinderAlt = false;

} // GetRangefinderAltitude


void InitRangefinder(void) {

	F.RangefinderActive = true; // optimistic

	if (F.Emulation)
		F.RangefinderActive = false;
	else {
		switch (CurrRFSensorType) {
		case MaxSonarcm:
			F.RangefinderActive = true; // analogRead(RangefinderAnalogSel) > 0.5f; // open circuit?
			break;
		case SRFI2Ccm:
			if ((busDev[rfSel].Used) && !busDev[rfSel].useSPI)
				SIOWrite(rfSel, 81, 1);
			else
				F.RangefinderActive = false;
			break;
		case MaxSonarI2Ccm:
			if ((busDev[rfSel].Used) && !busDev[rfSel].useSPI)
				SIOWrite(rfSel, 81, 1);
			else
				F.RangefinderActive = false;
			break;
		case SharpIRGP2Y0A02YK:
			break;
		case SharpIRGP2Y0A710K:
			break;
		case noRF:
		default:
			F.RangefinderActive = false;
			break;
		} // switch
		mSTimer(RangefinderUpdate, RF[CurrRFSensorType].intervalmS);
	}

	RangefinderAltitude = SensorAltitude, SensorAltitudeP = 0.0f;
	GetRangefinderAltitude();

} // InitRangefinder


//___________________________________________________________________

// altitude

void SetDesiredAltitude(real32 Desired) {
	Alt.P.Desired = Desired;
	Alt.P.IntE = Alt.R.IntE = Sl = 0.0f;
} //SetDesiredAltitude

real32 SelectFromGPSOrBaro(void) {
	return F.UsingGPSAltitude && F.OriginValid ? GPS.altitude
			- GPS.originAltitude : Altitude;
} // SelectFromGPSOrBaro

real32 AltitudeSensor(void) {
	const real32 RFlb = RF[CurrRFSensorType].maxAlt * 0.4f;
	const real32 RFub = RF[CurrRFSensorType].maxAlt * 0.6f;

	enum Strategies {
		Recapture, TrackOrigin, DropHold
	} Strategy;

	const real32 RF_CF = 0.85f;
	real32 AltOffset;

	Strategy = Recapture;

	switch (Strategy) {
	case DropHold: // solves cliff problem
		SensorAltitudeP = SensorAltitude;
		if (F.UsingRangefinderAlt) {
			SensorAltitude = RangefinderAltitude;
			if (F.HoldingAlt && !WasUsingRF)
				F.HoldingAlt = false;
			WasUsingRF = true;
		} else {
			SensorAltitude = SelectFromGPSOrBaro();
			if (F.HoldingAlt && WasUsingRF)
				F.HoldingAlt = false;
			WasUsingRF = false;
		}
		break;
	case TrackOrigin:
		// does not solve cliff problem and can give runaway altitude
		// DO NOT TAKE OFF AT A CLIFF EDGE
		SensorAltitudeP = SensorAltitude;
		if (F.UsingRangefinderAlt) {
			SensorAltitude = RangefinderAltitude;
			if ((RangefinderAltitude > RFlb) && (RangefinderAltitude < RFub)) {
				AltOffset = Altitude - RangefinderAltitude;
				OriginAltitude = OriginAltitude * RF_CF + ((OriginAltitude
						+ AltOffset) * (1.0f - RF_CF));
			}
		} else
			SensorAltitude = SelectFromGPSOrBaro();
		break;
	case Recapture: // solves cliff problem
		SensorAltitudeP = SensorAltitude;
		if (F.UsingRangefinderAlt) {
			SensorAltitude = RangefinderAltitude;
			if (F.HoldingAlt && !WasUsingRF)
				SetDesiredAltitude(Altitude);
			WasUsingRF = true;
		} else {
			SensorAltitude = SelectFromGPSOrBaro();
			if (F.HoldingAlt && WasUsingRF)
				SetDesiredAltitude(SensorAltitude);
			WasUsingRF = false;
		}
		break;
	default:
		break;
	} // switch

	return SensorAltitude;

} // AltitudeSensor


void UpdateAltitudeEstimates(void) {
	static timeuS LastUpdateuS = 0;
	timeuS NowuS;

	GetBaro();
	GetRangefinderAltitude();

	if (F.NewBaroValue) {
		F.NewBaroValue = false;

		AltitudemS = mSClock();
		AltdT = BarodT;

		UpdateBaroVariance(RawDensityAltitude); // 2.188uS

		AccZ = ConditionAccZ(AltdT); // 2.438uS
		AltitudeKF(RawDensityAltitude, AccZ, AltdT); // 1.438uS
		Altitude = DensityAltitude - OriginAltitude;

		ROCF = LPFn(&FROCLPF, ROC, AltdT); // used for landing and cruise throttle tracking

		StatsMax(AltitudeS, Altitude);
		StatsMinMax(MinROCS, MaxROCS, ROC * 100.0f);
/*
		if ((State == InFlight) && (CurrTelType == FusionTelemetry)) {
			BlackBoxEnabled = true;
			SetTelemetryBaudRate(TelemetrySerial, 115200);
			SendFusionPacket(TelemetrySerial);
			BlackBoxEnabled = false;
		}
*/
		if (State == InFlight)
			Altitude = AltitudeSensor();
		else {
			SensorAltitude = 0.0f;
			WasUsingRF = false;
		}

		F.NewAltitudeValue = true;
	}

} // UpdateAltitudeEstimates

