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
real32 MagLockE, MagHeading, InitialMagHeading, Heading, CompassOffset;
uint8 MagnetometerType;
real32 MagdT;

real32 Mag[3];
int16 RawMag[3];
real32 MagScale[3] = { 1.0, };
real32 MagSample[MAG_CAL_SAMPLES][3];
uint16 SphereIterations;

void WriteVerifyMag(uint8 a, uint8 v) {
	// idea due to Bill Nesbitt from AQ - seems he also found
	// write transactions to the HMC5XXX to be problematic
	// although it may have just been MPU6500 feed through
	// low cost so leave TODO:
	uint8 r;

	do {
		Delay1mS(10);
		SIOWrite(magSel, a, v);
		Delay1mS(10);
		r = SIORead(magSel, a);
	} while (r != v);

} // WriteVerifyMag

boolean ReadMagnetometer(void) {
	//int16 RawTemp = 0;
	boolean r;

	SIOReadBlocki16vataddr(magSel, HMC5XXX_DATA, 3, RawMag, true);

	//if (spiDevUsed[magSel])
	//	SIOReadBlocki16vataddr(magSel, HMC5XXX_TEMP, 1, &RawTemp,
	//true);

	//MagTemperature = (real32) RawTemp * 0.0078125 + 25.0f;

	RotateSensor(&RawMag[X], &RawMag[Y], MagQuadrant);

	r = (RawMag[0] != -4096) && (RawMag[1] != -4096) && (RawMag[2] != -4096);
	if (!r)
		incStat(CompassFailS);

	return (r);
} // ReadMagnetometer


void GetMagnetometer(void) {
	static timeuS LastMagUpdateuS = 0;
	int32 a;

	if (F.MagnetometerActive) {
		if (mSTimeout(MagnetometerUpdate)) {
			mSTimer(MagnetometerUpdate, MAG_TIME_MS);

			RawMag[MX] = -4096;

			MagdT = dTUpdate(&LastMagUpdateuS);

			if (ReadMagnetometer()) {

				if (F.InvertMagnetometer) {
					Mag[BF] = -(real32) RawMag[MY] * MagScale[MY];
					Mag[LR] = (real32) RawMag[MX] * MagScale[MX];
					Mag[UD] = (real32) RawMag[MZ] * MagScale[MZ];
				} else {
					Mag[BF] = (real32) RawMag[MY] * MagScale[MY];
					Mag[LR] = (real32) RawMag[MX] * MagScale[MX];
					Mag[UD] = -(real32) RawMag[MZ] * MagScale[MZ];
				}

				for (a = X; a <= Z; a++)
					Mag[a] -= Config.MagCal.Bias[a];

				real32 NormR = invSqrt(Sqr(Mag[0]) + Sqr(Mag[1]) + Sqr(Mag[2]));
				for (a = X; a <= Z; a++)
					Mag[a] *= NormR;

				F.NewMagValues = true;
			} else
				F.NewMagValues = false;
		}
	} else
		F.NewMagValues = false;

} // GetMagnetometer


void CalculateInitialMagneticHeading(void) {
	real32 xh, yh;
	real32 cR, sR, cP, sP;

//	if (F.NewMagValues) {
//		F.NewMagValues = false;
	if (F.MagnetometerActive) {

		cR = cosf(-Angle[Roll]);
		sR = sinf(-Angle[Roll]);
		cP = cosf(Angle[Pitch]);
		sP = sinf(Angle[Pitch]);

		xh = Mag[BF] * cP + sP * (Mag[UD] * cR - Mag[LR] * sR);
		yh = Mag[LR] * cR + Mag[UD] * sR;

		InitialMagHeading = -atan2f(yh, xh);
	} else
		InitialMagHeading = 0.0f;

} // CalculateInitialMagneticHeading


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

uint8 HMC5XXXSetReset[] = { 0x11, // Set positive bias
		0xA0, // Gain
		0x01 // Perform single conversion
		};

uint8 HMC5883LConfig[] = { 0x78, // Temp Comp, 75Hz normal mode 8 sample/measurement
		0x78, // 75Hz normal mode 8 sample/measurement
		0x00, // 0.88Ga increase gain with large dip angles
		0x00 //  continuous};
		};

uint8 HMC5983Config[] = { 0xf8, // Temp Comp, 75Hz normal mode 8 sample/measurement
		0x78, 0x00, 0x00 };

void InitMagnetometer(void) {
	const uint8 Samples = 20;
	idx a, s;

	MagScale[BF] = MagScale[LR] = MagScale[UD] = 0.0f;

	CheckMagnetometerActive();

	if (F.MagnetometerActive) {

		ReadMagnetometer(); // discard initial values

		for (s = 0; s < Samples; s++) {
			LEDToggle(ledRedSel);

			SIOWrite(magSel, HMC5XXX_MODE, 0x01); // Perform single conversion
			Delay1mS(50);

			ReadMagnetometer();

			MagScale[MX] += Abs(1264.4f / RawMag[MX]);
			MagScale[MY] += Abs(1264.4f / RawMag[MY]);
			MagScale[MZ] += Abs(1177.2f / RawMag[MZ]);

			Delay1mS(20);
		}

		for (a = X; a <= Z; a++)
			MagScale[a] /= (real32) (Samples * 2.0f);

		Delay1mS(50);

		if (busDev[magSel].useSPI)
			SIOWriteBlockataddr(magSel, HMC5XXX_CONFIG_A, 4, HMC5983Config);
		else
			SIOWriteBlockataddr(magSel, HMC5XXX_CONFIG_A, 4, HMC5883LConfig);

		mSTimer(MagnetometerUpdate, MAG_TIME_MS);
		Delay1mS(MAG_TIME_MS * 2);

		F.NewMagValues = false;
		do {
			GetMagnetometer();
		} while (!F.NewMagValues);

		CalculateInitialMagneticHeading();

		CheckMagnetometerIsCalibrated();

		F.MagnetometerFailure = !F.MagnetometerCalibrated;

	} else {
		F.MagnetometerFailure = true;
		F.NewMagValues = false;
	}

} // InitMagnetometer


void CheckMagnetometerIsCalibrated(void) {

	F.MagnetometerCalibrated = Config.MagCal.Calibrated == 1;

} // CheckMagnetometerIsCalibrated

void InitMagnetometerBias(void) {
	idx a;

	// Nominal x/y 766, z 660 @1Ga
	for (a = X; a <= Z; a++)
		Config.MagCal.Bias[a] = 0.0f;

	Config.MagCal.Calibrated = 0xff;
	F.MagnetometerCalibrated = false;

	ConfigChanged = true;

} // InitMagnetometerBias

void GetMagSample(int16 ss) {

	if (F.InvertMagnetometer) {
		MagSample[ss][BF] = -(real32) RawMag[MY] * MagScale[MY];
		MagSample[ss][LR] = (real32) RawMag[MX] * MagScale[MX];
		MagSample[ss][UD] = (real32) RawMag[MZ] * MagScale[MZ];
	} else {
		MagSample[ss][BF] = (real32) RawMag[MY] * MagScale[MY];
		MagSample[ss][LR] = (real32) RawMag[MX] * MagScale[MX];
		MagSample[ss][UD] = -(real32) RawMag[MZ] * MagScale[MZ];
	}
} // GetMagSample

#define I2(i) (i>0?1:0)

void CalibrateHMC5XXX(uint8 s) {
	real32 MagOrigin[3];
	uint16 Population[2][2][2];
	idx a;
	uint16 ss;

	LEDsOff();
	LEDOn(ledBlueSel);

	if (F.MagnetometerActive) {

		InitMagnetometerBias();

		memset(&Population, 0, sizeof(Population));
		memset(&MagSample, 0, sizeof(MagSample));

		ss = 0;
		mSTimer(MagnetometerUpdate, MAG_TIME_MS);

		while (ss < MAG_CAL_SAMPLES)
			if (mSTimeout(MagnetometerUpdate)) {
				mSTimer(MagnetometerUpdate, MAG_TIME_MS);

				if (ReadMagnetometer()) {

					GetMagSample(ss);

					if (Population[I2(MagSample[ss][X])][I2(MagSample[ss][Y])][I2(MagSample[ss][Z])]
							< (MAG_CAL_SAMPLES / 7)) {
						Population[I2(MagSample[ss][X])][I2(MagSample[ss][Y])][I2(MagSample[ss][Z])]++;

						ss++;
						LEDOff(ledYellowSel);
						LEDOn(ledGreenSel);
					} else {
						LEDOn(ledYellowSel);
						LEDOff(ledGreenSel);
					}
				}
			}
		LEDsOff();
		LEDOn(ledYellowSel);

		// Actually it will be an ellipsoid due to hard iron effects
		SphereIterations = SphereFit(MagSample, MAG_CAL_SAMPLES, 200, 0.01f,
				MagOrigin, &Config.MagCal.Magnitude);

		for (a = X; a <= Z; a++)
			Config.MagCal.Bias[a] = MagOrigin[a];


		Config.MagCal.Calibrated = 1;
		F.MagnetometerCalibrated = true;

		ConfigChanged = true;
		UpdateConfig();

		DoBeep(8, 1);

		LEDsOff();

		Config.MagCal.Calibrated = 1;
		F.MagnetometerCalibrated = true;
	}

	SendAckPacket(s, UAVXMiscPacketTag, F.MagnetometerCalibrated);

} // CalibrateHMC5XXX


void CheckMagnetometerActive(void) {

	if (busDev[magSel].Used && (busDev[magSel].type == hmc5xxxMag)) {
		uint8 v = 0;

		SIOWriteBlockataddr(magSel, HMC5XXX_CONFIG_A, 3, HMC5XXXSetReset);
		Delay1mS(50);

		SIOReadBlock(magSel, HMC5XXX_TAG, 1, &v);
		F.MagnetometerActive = v == 'H';

	} else
		F.MagnetometerActive = false;

} //  CheckMagnetometerActive


