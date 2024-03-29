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

real32 BaroOddsEvensFrac = 0.0f;
uint32 BaroTempVal, BaroPressVal, BaroVal;
real32 BaroTemperature, BaroPressure, CompensatedBaroPressure;

uint8 CurrRFSensorType;
uint8 CurrAltSource;
boolean WasUsingRF = false;

real32 DesiredAlt, OriginAltitude, RawAlt, DensityAltitude, RawDensityAltitude,
		DensityAltitudeP, SensorAltitude, RFAltitudeP, BaroGPSCFAltitudeP;

real32 RFAltitudeOffset, GPSAltitudeOffset;

real32 BaroROC, ROC, ROCTrack;

filterStruct ROCTrackLPF, ROCLPF, DensityAltLPF, AccUBumpLPF;
filterM3Struct BaroM3F, RangefinderM3F;
filterStruct AccUMAF, BaroMAF;

real32 Airspeed;
real32 BarodT, AltdT;

uint8 BaroType = ms5611Baro;

real32 FakeDensityAltitude;

real32 CalculateDensityAltitude(boolean FullMath, real32 P) {

	if (FullMath) {
		//return (1.0f - powf((P / 101325.0f), 0.190295f)) * 44330.0f;
		return ((1.0f - powf((P / 101325.0f), 0.19022256f))
				* (BaroTemperature + 273.15f)) * 153.846153846f;
	} else
		return (101325.0f - P) * 0.08406f; // ~calibration to 200M // 0.25uS

} // CalculateDensityAltitude

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

		off = ((int64) ms56xx_c[2] << 16)
				+ (((int64) ms56xx_c[4] * ms56xxdt) >> 7);
		sens = ((int64) ms56xx_c[1] << 15)
				+ (((int64) ms56xx_c[3] * ms56xxdt) >> 8);

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

		off = ((int64) ms56xx_c[2] << 17)
				+ (((int64) ms56xx_c[4] * ms56xxdt) >> 6);
		sens = ((int64) ms56xx_c[1] << 16)
				+ (((int64) ms56xx_c[3] * ms56xxdt) >> 7);

		if (ms56xxt < 2000) {
			t = Sqr((int64 ) ms56xxt - 2000);
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

	uSTimer(BaroUpdateuS, ms56IntervaluS);

} // StartBaro

// altitude pressure
// 0m   101325Pa
// 100m   100129Pa delta = 1196
// 1000m    89875Pa
// 1100m    88790Pa delta = 1085

void GetBaro(void) {
	static timeuS LastUpdateuS = 0;
	timeuS IntervaluS, NowuS;
	static boolean BaroPrimed = false;
	real32 RawBaroPressure;
	static idx s = 0;
	idx i;
	uint8 B[3];

	if (F.BaroActive) {

		if (SIOTokenFree && uSTimeout(BaroUpdateuS)) { // don't use uSTimeout

			SIOTokenFree = false;

			SIOReadBlock(baroSel, 0, 3, B);
			NowuS = uSClock();

			BaroVal = ((uint32) B[0] << 16) | ((uint32) B[1] << 8) | B[2];

			if (AcquiringPressure) {

				// could do 10:1 P/T interleaving but adds lag 5 cycle or 50mS lag - not worth it

				AcquiringPressure = false;
				StartBaro(AcquiringPressure);

				IntervaluS = Limit(NowuS - LastUpdateuS,
						ms56IntervaluS * 2 - 2000, ms56IntervaluS * 2 + 2000);
				BarodT = (real32) IntervaluS * 0.000001f;
				LastUpdateuS = NowuS;

				if (F.Emulation)
					RawDensityAltitude = FakeAltitude
							+ SensorNoise(BARO_NOISE * BarodT);
				else {

					BaroPressVal = BaroVal;
					RawBaroPressure = CompensateBaro(BaroTempVal, BaroPressVal);

#if defined(UAVXF4V4)
					// TODO: KLUDGE TO FIX "SOME" V4 BOARDS - NOT SIMPLE SIGN REVERSAL THOUGH? POOSIBLE SPI FAIL msb
					//		F.BaroFailure = BaroPressure < 0.0f;
					//		BaroPressure = F.BaroFailure ? BaroPressureP : BaroPressure;
#endif

					BaroPressure = real32Median3Filter(&BaroM3F,
							RawBaroPressure);

					//BaroPressure = MAF(&BaroMAF, BaroPressure);

					RawDensityAltitude = CalculateDensityAltitude(true,
							BaroPressure);

				}

				F.NewBaroValue = true;

			} else {

				AcquiringPressure = true;
				StartBaro(AcquiringPressure);

				if (BaroVal != 0)
					BaroTempVal = BaroVal;
			}

		}
	} else
		RawDensityAltitude = 0.0f;

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
#define BARO_HIST_VAR_LENGTH 32
	const real32 Alt_K = 0.85f; // was 0.995
	static real32 B[BARO_HIST_VAR_LENGTH] = { 0.0f, };
	static idx i = 0;

	if (F.Hovering) {

		B[i++] = Alt;

		if (i >= BARO_HIST_VAR_LENGTH) {
			TrackBaroVariance = TrackBaroVariance * Alt_K
					+ CalculateVariance(B, BARO_HIST_VAR_LENGTH)
							* (1.0f - Alt_K);
			i = 0;
		}
	}
} // UpdateBaroVariance

void InitBaro(void) {
	int16 BaroWarmupCycles;

	static boolean First = true;

	ms56IntervaluS = ms56xxSampleIntervaluS[MS56XXOSR >> 1];

	BaroTempVal = BaroPressVal = BaroVal = 0;

	CheckBaroActive();

	if (F.BaroActive) {

		ReadBaroCalibration();
		Delay1mS(100);

		AcquiringPressure = false; // temperature must be first
		StartBaro(AcquiringPressure);

		BaroWarmupCycles = 0;
		F.NewBaroValue = false;

		do { // just warming up the ms56xx!
			GetBaro();
			SIOTokenFree = true;
			if (F.NewBaroValue) {
				F.NewBaroValue = false;

				if (First) {
					OriginAltitude = RawDensityAltitude;
					First = false;
				} else
					OriginAltitude = OriginAltitude * 0.9f
							+ RawDensityAltitude * 0.1f;

				BaroWarmupCycles++;
			}
		} while (BaroWarmupCycles < 200);

	} else {
		TxString(TelemetrySerial, "Baro Inactive");
		TxNextLine(TelemetrySerial);
		RawDensityAltitude = 0.0f;
	}

	SetDesiredAltitude(0.0f);

} // InitBaro

void TrackOriginAltitude(void) {

	WasUsingRF = false;
	OriginAltitude = KFDensityAltitude;
	SetDesiredAltitude(0.0f);
	if (F.GPSValid && (GPS.vAcc < GPSMinvAcc))
		GPS.originAltitude = GPS.altitude;

} // TrackOriginAltitude

//____________________________________________________________________________

// rangefinder

RFStruct RF[] = { { 50, 0.4f, 5.0f }, //6.45f }, // MaxSonarcm
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
	const real32 A[] = { 15.048f, 95.299 };
	const real32 B[] = { -1.16f, -1.9197 };
	//const real32 M[] = { 0.757, 0.666 };

	real32 v = 0.0f;

	// MAJOR PROBLEM AS DISTANCE CURVE IS NOT MONOTONIC
	//	if (RF[CurrRFSensorType].Min)

	v = real32Median3Filter(&RangefinderM3F, r);
	v = A[Sel] * powf(v, B[Sel]);

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
		RangefinderAltitude = SharpRFLookup(analogRead(RangefinderAnalogSel),
				0);
		break;
	case SharpIRGP2Y0A710K:
		RangefinderAltitude = SharpRFLookup(analogRead(RangefinderAnalogSel),
				1);
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

		if (mSTimeout(RangefinderUpdatemS)) {
			mSTimer(RangefinderUpdatemS, RF[CurrRFSensorType].intervalmS);

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
				if ((RFBucket >= (RF_BUCKET_SIZE - 4))
						&& F.UsingRangefinderAlt) {
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
		mSTimer(RangefinderUpdatemS, RF[CurrRFSensorType].intervalmS);
	}

	RangefinderAltitude = SensorAltitude = 0.0f;
	GetRangefinderAltitude();

} // InitRangefinder

//___________________________________________________________________

// altitude

void SetDesiredAltitude(real32 Desired) {

	Alt.P.Desired = DesiredAlt = Desired;
	Alt.P.IntE = Alt.R.IntE = Sl = 0.0f;
} //SetDesiredAltitude

void CheckForRFSensor(real32 SensorAltitude) {

	if (F.UsingRangefinderAlt) {
		if (F.HoldingAlt && !WasUsingRF) {
			SetDesiredAltitude(RangefinderAltitude);
			WasUsingRF = true;
		}
		Altitude = RangefinderAltitude;
	} else if (F.HoldingAlt && WasUsingRF) {
		SetDesiredAltitude(Altitude);
		WasUsingRF = false;
	}

} // CheckForRFSensor

//----------------------------------------------------------------

// ChatGBT 3.4


#define Q 0.01  // Process noise covariance
#define R 0.1   // Measurement noise covariance
#define INNOVATION_THRESHOLD 5.0  // Innovation threshold to filter outliers

typedef struct {
	double x;  // State (true altitude)
	double v;  // Vertical velocity
	double P;  // Error covariance
} KalmanState;

void kalmanFilterUpdate(KalmanState *state, double z, double u, double dt) {

	static boolean FirstPass = true;

	if (FirstPass) {
		state->x = z;
		FirstPass = false;
	}

	// Prediction
	state->x += state->v * dt + 0.5 * u * dt * dt;
	state->v += u * dt;
	state->P += Q;

	// Calculate innovation
	double innovation = z - state->x;

	// Check if the innovation is within the threshold
	if (fabs(innovation) < INNOVATION_THRESHOLD) {
		// Update
		double K = state->P / (state->P + R);
		state->x += K * innovation;
		state->v += K * (innovation / dt);  // Update vertical velocity
		state->P *= (1 - K);
	}

}

void UpdateAltitudeEstimatesGBT(real32 RawDensityAltitude, real32 AccU,
		real32 AltdT) {
	// Initial state
	KalmanState kalmanState = { 0.0, 0.0, 1.0 }; // Assuming initial vertical velocity is 0

	// Kalman filter

	if (F.NewBaroValue) {
		// Use density altitude as the measurement, and vertical acceleration as the control input
		kalmanFilterUpdate(&kalmanState, RawDensityAltitude, AccU, AltdT);

		// Use GPS altitude as an additional measurement
		kalmanFilterUpdate(&kalmanState, GPS.altitude, 0.0, AltdT);

		KFDensityAltitude = kalmanState.x;
		KFROC = kalmanState.v;
	}

}

void UpdateAltitudeEstimates(void) {
	static boolean First = true;
	const real32 BARO_GPS_CF = 0.85f;
	static timeuS LastUpdateuS = 0;
	timeuS NowuS;
	static boolean FirstAltEst = true;

	GetBaro();
	GetRangefinderAltitude();

	if (F.NewBaroValue) { // altitude is synchronised to baro regardless of actual sensor used!
		F.NewBaroValue = false;

		AltdT = BarodT;

		// do all for logging purposes

		// Baro
		DensityAltitude = LPFn(&DensityAltLPF,
				RawDensityAltitude, AltdT);
		if (First) {
			DensityAltitudeP = DensityAltitude;
			First = false;
		} else {
			BaroROC = LPFn(&ROCLPF,
					(DensityAltitude - DensityAltitudeP) / AltdT, AltdT);
			DensityAltitudeP = DensityAltitude;
		}


		// BaroAccKF
		UpdateBaroVariance(RawDensityAltitude);
		UpdateAccUVariance(AccU);

//#define USE_CHATGBT
#if defined(USE_CHATGBT)
		BROKEN AltitudeKFChatGBT(RawDensityAltitude, GPS.altitude, AccU, AltdT);
#else
		AltitudeKF(RawDensityAltitude, GPS.altitude, AccU, AltdT);

#endif


#ifdef USE_BARO_ALT
		Altitude = DensityAltitude - OriginAltitude;
		ROC = BaroROC;
#else
		Altitude = KFDensityAltitude - OriginAltitude;
		ROC = KFROC;
#endif

		CheckForRFSensor(Altitude);

		ROCTrack = LPFn(&ROCTrackLPF, ROC, AltdT); // used for landing and cruise throttle tracking
		F.AccUBump = LPFn(&AccUBumpLPF, AccU, AltdT) > ACCU_LANDING_MPS_S;

		if (CurrBBLogType == logAltitude)
			SendAltitudeControlPacket(TelemetrySerial);

		F.NewAltitudeValue = true;
	}

} // UpdateAltitudeEstimates

void InitAltitude(void) {

	const uint8 AccBumpLPFOrder = 2;

	const uint8 DensityAltLPFOrder = 2;
	const uint8 ROCLPFOrder = 2;
	const uint8 ROCTrackLPFOrder = 2;

	Altitude = DensityAltitudeP = RFAltitudeP = BaroGPSCFAltitudeP = ROC =
			ROCTrack = AccU = 0.0f;

	initLPFn(&AccUBumpLPF, AccBumpLPFOrder, 1.0f);
	initLPFn(&DensityAltLPF, DensityAltLPFOrder, CurrAltLPFHz);
	initLPFn(&ROCLPF, ROCLPFOrder, CurrAltLPFHz);
	initLPFn(&ROCTrackLPF, ROCTrackLPFOrder, CurrAltLPFHz * 0.25f);

	BaroM3F.initialised = RangefinderM3F.initialised = false;

#if !defined(ALT_KF_TESTING)
	InitBaro();
#endif

} // InitAltitude

