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

// Compass

#include "UAVX.h"

#define HMC5XXX_CONFIG_A 	0x00
#define HMC5XXX_CONFIG_B 	0x01
#define HMC5XXX_MODE 		0x02
#define HMC5XXX_DATA 		0x03
#define HMC5XXX_TEMP 		0x31
#define HMC5XXX_STATUS 		0x09
#define HMC5XXX_TAG 		0x0a

real32 MagTemperature = 0.0f;
real32 MagVariation = 0.0f;
real32 MagVariationWMM2010 = 0.0f;
real32 MagLockE, MagHeading, Heading, CompassOffset;
uint8 MagnetometerType;
real32 MagdT;

real32 Mag[3];
int16 RawMag[3];
real32 MagScale[3] = {1.0,};
real32 d[MAG_CAL_SAMPLES][3];
uint16 SphereIterations;

void WriteVerifyMag(uint8 a, uint8 v) {
	// idea due to Bill Nesbitt from AQ - seems he also found
	// write transactions to the HMC5XXX to be problematic
	// although it may have just been MPU6500 feed through
	// low cost so leave TODO:
	uint8 r;

	do {
		Delay1mS(10);
		sioWrite(SIOMag, HMC5XXX_ID, a, v);
		Delay1mS(10);
		r = sioRead(SIOMag, HMC5XXX_ID, a);
	} while (r != v);

} // WriteVerifyMag

boolean ReadMagnetometer(void) {
	int16 RawTemp = 0;
	boolean r;

	sioReadBlocki16vataddr(SIOMag, HMC5XXX_ID, HMC5XXX_DATA, 3, RawMag, true);

	if (spiDevUsed[SIOMag])
		sioReadBlocki16vataddr(SIOMag, HMC5XXX_ID, HMC5XXX_TEMP, 1, &RawTemp,
				true);

	MagTemperature = (real32) RawTemp * 0.0078125 + 25.0f;

	r = (RawMag[0] != -4096) && (RawMag[1] != -4096) && (RawMag[2] != -4096);
	if (!r)
		incStat(CompassFailS);

	return (r);
} // ReadMagnetometer


void GetMagnetometer(void) {
	uint32 NowmS;
	static uint32 LastMagUpdateuS = 0;
	int32 a;

	NowmS = mSClock();
	if (NowmS >= mS[MagnetometerUpdate]) {
		mSTimer(NowmS, MagnetometerUpdate, MAG_TIME_MS);

		RawMag[MX] = -4096;

		MagdT = dTUpdate(uSClock(), &LastMagUpdateuS);

		if (ReadMagnetometer()) {

			if (F.InvertMagnetometer) {
				Mag[BF] = (real32) RawMag[MX] * MagScale[MX]; // -
				Mag[LR] = -(real32) RawMag[MY] * MagScale[MY];
				Mag[UD] = (real32) RawMag[MZ] * MagScale[MZ];
			} else {
				Mag[BF] = (real32) RawMag[MY] * MagScale[MY];
				Mag[LR] = (real32) RawMag[MX] * MagScale[MX];
				Mag[UD] = -(real32) RawMag[MZ] * MagScale[MZ];
			}

			for (a = X; a <= Z; a++)
				Mag[a] -= NV.MagCal.Bias[a];

			real32 NormR = invSqrt(Sqr(Mag[0]) + Sqr(Mag[1]) + Sqr(Mag[2]));
			for (a = X; a <= Z; a++)
				Mag[a] *= NormR;

			F.NewMagValues = true;
		} else
			F.NewMagValues = false;
	}

	F.MagnetometerFailure = !F.MagnetometerCalibrated;

} // GetMagnetometer


void CalculateMagneticHeading(void) {
	real32 xh, yh;
	real32 cR, sR, cP, sP;

	if (F.NewMagValues) {
		F.NewMagValues = false;

		cR = cosf(-A[Roll].Angle);
		sR = sinf(-A[Roll].Angle);
		cP = cosf(A[Pitch].Angle);
		sP = sinf(A[Pitch].Angle);

		xh = Mag[BF] * cP + sP * (Mag[UD] * cR - Mag[LR] * sR);
		yh = Mag[LR] * cR + Mag[UD] * sR;

		MagHeading = -atan2f(yh, xh); // filtering is difficult because of 360deg discontinuity

	}

} // CalculateMagneticHeading


#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

void InitMagnetometer(void) {
	uint32 FatalTimeoutmS;

	const uint8 Samples = 20;
	uint8 ActualSamples;
	idx i, a;

	FatalTimeoutmS = mSClock() + 10000;
	do {
		Delay1mS(10);
		F.MagnetometerActive = MagnetometerIsActive();
	} while (!(mSClock() > FatalTimeoutmS) && !F.MagnetometerActive);

	MagScale[BF] = MagScale[LR] = MagScale[UD] = 0.0f;

	if (F.MagnetometerActive) {

		WriteVerifyMag(HMC5XXX_CONFIG_A, 0x11); // Set positive bias
		Delay1mS(50);
		WriteVerifyMag(HMC5XXX_CONFIG_B, 0xA0); // Gain
		Delay1mS(100);

		sioWrite(SIOMag, HMC5XXX_ID, HMC5XXX_MODE, 0x01); // Perform single conversion
		Delay1mS(50);
		ReadMagnetometer(); // discard initial values

		ActualSamples = 0;

		for (i = 0; i < Samples; i++) {
			sioWrite(SIOMag, HMC5XXX_ID, HMC5XXX_MODE, 0x01); // Perform single conversion
			Delay1mS(50);

			if (ReadMagnetometer()) {
				MagScale[MX] += Abs(1264.4f / RawMag[MX]);
				MagScale[MY] += Abs(1264.4f / RawMag[MY]);
				MagScale[MZ] += Abs(1177.2f / RawMag[MZ]);
				ActualSamples++;
			}

			Delay1mS(20);
		}

		if (ActualSamples > 0) {
			for (a = X; a <= Z; a++)
				MagScale[a] /= (real32) (Samples * 2);

			Delay1mS(50);

			if (spiDevUsed[SIOMag])
				WriteVerifyMag(HMC5XXX_CONFIG_A, 0xf8); // Temp Comp, 75Hz normal mode 8 sample/measurement
			else
				WriteVerifyMag(HMC5XXX_CONFIG_A, 0x78); // 75Hz normal mode 8 sample/measurement
			Delay1mS(50);
			WriteVerifyMag(HMC5XXX_CONFIG_B, 0x00); // 0.88Ga increase gain with large dip angles
			Delay1mS(50);
			sioWrite(SIOMag, HMC5XXX_ID, HMC5XXX_MODE, 0x00); // Set continuous mode
			Delay1mS(50);

			mSTimer(mSClock(), MagnetometerUpdate, MAG_TIME_MS);

			F.NewMagValues = false;

			do {
				GetMagnetometer();
			} while (!F.NewMagValues);

			CheckMagnetometerIsCalibrated();

		} else
			F.NewMagValues = F.MagnetometerActive = false;

	} else
		F.NewMagValues = false;

} // InitMagnetometer

void CheckMagnetometerIsCalibrated(void) {

	F.MagnetometerCalibrated = NV.MagCal.CalSamples == MAG_CAL_SAMPLES;

} // CheckMagnetometerIsCalibrated

void InitMagnetometerBias(void) {
	idx a;

	// Nominal x/y 766, z 660 @1Ga
	for (a = X; a <= Z; a++)
		NV.MagCal.Bias[a] = 0.0f;

	NV.MagCal.CalSamples = 0;

} // InitMagnetometerBias

void GetMagSample(int16 i) {

	if (F.InvertMagnetometer) {
		d[i][BF] = (real32) RawMag[MX] * MagScale[MX]; // -
		d[i][LR] = -(real32) RawMag[MY] * MagScale[MY];
		d[i][UD] = (real32) RawMag[MZ] * MagScale[MZ];
	} else {
		d[i][BF] = (real32) RawMag[MY] * MagScale[MY];
		d[i][LR] = (real32) RawMag[MX] * MagScale[MX];
		d[i][UD] = -(real32) RawMag[MZ] * MagScale[MZ];
	}
} // GetMagSample

#define I2(i) (i>0?1:0)

void CalibrateHMC5XXX(uint8 s) {
	real32 MagOrigin[3];
	uint16 Population[2][2][2];
	uint16 a, ss;

	LEDOn(LEDBlueSel);

	if (F.MagnetometerActive) {

		InitMagnetometerBias();

		memset(&Population, 0, sizeof(Population));

		ss = 0;
		while (ss < MAG_CAL_SAMPLES) {

			mSTimer(mSClock(), MagnetometerUpdate, MAG_TIME_MS);
			while (mSClock() < mS[MagnetometerUpdate]) {
			};

			LEDToggle(LEDBlueSel);

			if (ReadMagnetometer()) {
				GetMagSample(ss);

				if (Population[I2(d[ss][X])][I2(d[ss][Y])][I2(d[ss][Z])]
						< (MAG_CAL_SAMPLES / 7)) { // should be 8
					LEDOff(LEDYellowSel);
					LEDOn(LEDGreenSel);
					Population[I2(d[ss][X])][I2(d[ss][Y])][I2(d[ss][Z])]++;
					ss++;
				} else {
					LEDOn(LEDYellowSel);
					LEDOff(LEDGreenSel);
				}
			}
		}


		// Actually it will be an ellipsoid due to hard iron effects
		SphereIterations = SphereFit(d, MAG_CAL_SAMPLES, 200, 0.01f, MagOrigin,
				&NV.MagCal.Magnitude);

		NV.MagCal.CalSamples = MAG_CAL_SAMPLES;

		for (a = X; a <= Z; a++)
			NV.MagCal.Bias[a] = MagOrigin[a];

		NVChanged = true;
		UpdateNV();

		DoBeep(8, 1);

		LEDsOff();

		F.MagnetometerCalibrated = true;
	}

#if defined(V4_BOARD)
	SendAckPacket(s, UAVXMiscPacketTag, F.MagnetometerCalibrated);
#endif
} // CalibrateHMC5XXX


boolean MagnetometerIsActive(void) {
	boolean r;
	uint8 v = 0;

	sioReadBlock(SIOMag, HMC5XXX_ID, HMC5XXX_TAG, 1, &v);

	r = v == 'H';

	return (r);
} //  MagnetometerActive


