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

const real32 fConvGauss[] = { 1264.4f, 1264.4f, 1177.2f };

real32 MagTemperature = 0.0f;
real32 MagVariation = 0.0f;
real32 MagVariationWMM2010 = 0.0f;
real32 MagLockE, MagHeading, InitialMagHeading, Heading, CompassOffset;
uint8 MagnetometerType;
real32 MagdT;

real32 Mag[3];
int16 RawMag[3];
real32 MagSamples[MAG_CAL_SAMPLES][3];
uint16 Population[2][2][2];
uint16 mm;

uint16 SphereIterations;

real32 CalculateMagneticHeading(void) {
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

		return -atan2f(yh, xh);
	} else
		return 0.0f;

} // CalculateMagneticHeading

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

void WriteVerifyMag(uint8 a, uint8 v) {
	// idea due to Bill Nesbitt from AQ - seems he also found
	// write transactions to the HMC5XXX to be problematic
	// although it may have just been MPU6500 feed through
	// low cost so leave TODO:
	uint8 r;

	do {
		Delay1mS(10);
		SIOWrite(CurrMagSel, a, v);
		Delay1mS(10);
		r = SIORead(CurrMagSel, a);
	} while (r != v);

} // WriteVerifyMag

void ReadMagnetometer(void) {
	idx i;
	uint8 buf[6];

	SIOReadBlockataddr(CurrMagSel, HMC5XXX_DATA, 6, buf);
	mSTimer(MagnetometerUpdatemS, MAG_TIME_MS);

	for (i = 0; i < 3; i++)
		RawMag[i] = ((int16) buf[i * 2] << 8) + buf[i * 2 + 1];

	F.MagnetometerFailure = (RawMag[0] != -4096) && (RawMag[1] != -4096)
			&& (RawMag[2] != -4096);

	RotateSensor(&RawMag[X], &RawMag[Y], MagQuadrant);

} // ReadMagnetometer

void GetMagnetometer(void) {
	static timeuS LastMagUpdateuS = 0;
	real32 TempMag[3];
	int32 a;

	if (SIOTokenFree && mSTimeout(MagnetometerUpdatemS)) {

		SIOTokenFree = false;

		ReadMagnetometer();

		for (a = X; a <= Z; a++)
			TempMag[a] = (real32) (RawMag[a] + Config.MagCal.Bias[a]) * fConvGauss[a];

		if (F.InvertMagnetometer) {
			Mag[BF] = -TempMag[MY];
			Mag[LR] = TempMag[MX];
			Mag[UD] = TempMag[MZ];
		} else {
			Mag[BF] = TempMag[MY];
			Mag[LR] = TempMag[MX];
			Mag[UD] = -TempMag[MZ];
		}

		MagdT = dTUpdate(&LastMagUpdateuS);

		F.NewMagValues = !F.MagnetometerFailure;
	}

} // GetMagnetometer

void InitMagnetometer(void) {
	uint8 buf[4];
	idx a, s;

	CheckMagnetometerActive();

	if (F.MagnetometerActive) {

		Delay1mS(50);

		if (busDev[CurrMagSel].useSPI)
			SIOWriteBlockataddr(CurrMagSel, HMC5XXX_CONFIG_A, 4, HMC5983Config);
		else
			SIOWriteBlockataddr(CurrMagSel, HMC5XXX_CONFIG_A, 4,
					HMC5883LConfig);

		Delay1mS(100);

		ReadMagnetometer(); // discard initial values

		mSTimer(MagnetometerUpdatemS, 0);
		Delay1mS(100);
		SIOTokenFree = true;

		GetMagnetometer();

		InitialMagHeading = CalculateMagneticHeading();

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

} // InitMagnetometerBias

#define I2(i) (i>0?1:0)

void CalibrateMagnetometer(uint8 s) {
	real32 MagOrigin[3];
	timemS TimeoutmS;
	boolean Timeout;

	uint8 QX, QY, QZ;
	idx a;

	LEDOn(ledBlueSel);

	if (F.MagnetometerActive) {

		InitMagnetometerBias();

		memset(&MagSamples, 0, sizeof(MagSamples));
		memset(&Population, 0, sizeof(Population));

		mm = 0;

		TimeoutmS = mSClock() + 60000;
		mSTimer(MagnetometerUpdatemS, MAG_TIME_MS);

		do {
			if (mSTimeout(MagnetometerUpdatemS)) {

				ReadMagnetometer();

				MagSamples[mm][X] = RawMag[X];
				MagSamples[mm][Y] = RawMag[Y];
				MagSamples[mm][Z] = RawMag[Z];

				QX = I2(MagSamples[mm][X]);
				QY = I2(MagSamples[mm][Y]);
				QZ = I2(MagSamples[mm][Z]);

				if (Population[QX][QY][QZ] < MAG_CAL_BIN_MAX) {

					Population[QX][QY][QZ]++;

					mm++;

					SendCalibrationPacket(TelemetrySerial);

					LEDOff(ledYellowSel);
					LEDOn(ledGreenSel);
				} else {
					LEDOn(ledYellowSel);
					LEDOff(ledGreenSel);
				}
			}

			Timeout = mSClock() > TimeoutmS;

		} while ((mm <= MAG_CAL_SAMPLES) && !Timeout);

		LEDsOff();
		if (Timeout) {
			LEDOn(ledRedSel);
			DoBeeps(8);
		} else {
			LEDOn(ledYellowSel);
			DoBeeps(2);
		}

		// Actually it will be an ellipsoid due to hard iron effects
		SphereIterations = SphereFit(MagSamples, mm, 200, 0.01f, MagOrigin,
				&Config.MagCal.Magnitude);

		for (a = X; a <= Z; a++)
			Config.MagCal.Bias[a] = MagOrigin[a];

		Config.MagCal.Calibrated = 1;
		F.MagnetometerCalibrated = true;

		SendAckPacket(s, UAVXMiscPacketTag, 1);

		DoBeep(8, 1);

		ConfigChanged = true;
		RefreshConfig();

	} else
		SendAckPacket(s, UAVXMiscPacketTag, 255);

	LEDOff(ledBlueSel);

} // CalibrateHMC5XXX

void CheckMagnetometerActive(void) {

	if (busDev[CurrMagSel].Used && (busDev[CurrMagSel].type == hmc5xxxMag)) {
		uint8 v = 0;

		SIOWriteBlockataddr(CurrMagSel, HMC5XXX_CONFIG_A, 3, HMC5XXXSetReset);
		Delay1mS(50);

		v = 77;
		SIOReadBlock(CurrMagSel, HMC5XXX_TAG, 1, &v);

		F.MagnetometerActive = v == 'H';

		//	F.MagnetometerActive = v != 0x48;

	} else
		F.MagnetometerActive = false;

} //  CheckMagnetometerActive

