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

// Gyros

#include "UAVX.h"

// ITG3200  1.214142088
const real32 GyroScale[] = { //
		13.0834, // MLX90609
				6.98131, // ADXRS613/150
				15.8666, // ST-AY530 0.0041
				17.4532, // ADXRS610/300
				0.001064225154f, // UAVXArm32IMU
				0.001064225154f, // FreeIMU
				0.001064225154f };

uint8 CurrAttSensorType = UAVXArm32IMU;

real32 GyroBias[3];
real32 Acc[3], Rate[3], Angle[3];
real32 RateEnergySum;
uint32 RateEnergySamples;
boolean GyrosErected = false;
int32 GyroErectionSamples;
const boolean UsingPavelFilter = false; // don't bother using

// NED
// P,R,Y
// BF, LR, UD


void ScaleRateAndAcc(uint8 imuSel) {
	idx a;

	UpdateGyroTempComp(imuSel);

	if (VTOLMode) {

		Rate[Pitch] = (RawGyro[X] - GyroBias[X]) * GyroScale[CurrAttSensorType];
		Rate[Yaw] = -(RawGyro[Y] - GyroBias[Y]) * GyroScale[CurrAttSensorType];
		Rate[Roll] = -(RawGyro[Z] - GyroBias[Z]) * GyroScale[CurrAttSensorType];

		Acc[UD] = -(RawAcc[Y] - Config.AccCal.Bias[Y]) * Config.AccCal.Scale[Y];
		Acc[LR] = (RawAcc[X] - Config.AccCal.Bias[X]) * Config.AccCal.Scale[X];
		Acc[BF] = -(RawAcc[Z] - Config.AccCal.Bias[Z]) * Config.AccCal.Scale[Z]
				- 1.0f;

	} else {

		Rate[Pitch] = (RawGyro[X] - GyroBias[X]) * GyroScale[CurrAttSensorType];
		Rate[Roll] = (RawGyro[Y] - GyroBias[Y]) * GyroScale[CurrAttSensorType];
		Rate[Yaw] = -(RawGyro[Z] - GyroBias[Z]) * GyroScale[CurrAttSensorType];

		Acc[BF] = (RawAcc[Y] - Config.AccCal.Bias[Y]) * Config.AccCal.Scale[Y];
		Acc[LR] = (RawAcc[X] - Config.AccCal.Bias[X]) * Config.AccCal.Scale[X];
		Acc[UD] = -(RawAcc[Z] - Config.AccCal.Bias[Z]) * Config.AccCal.Scale[Z];
	}

} // ScaleRateAndAcc

void ErectGyros(uint8 imuSel) {

	static boolean First = true;
	idx a;
	const int32 NoOfSamples = 200;
	static real32 gMax[3], gMin[3], g[3], t;
	static int32 samples;
	boolean Moving;

	LEDOn(ledBlueSel);

	if (GyroErectionSamples == 0) {
		for (a = X; a <= Z; a++)
			gMax[a] = gMin[a] = g[a] = RawGyro[a];
		t = 0.0f;
	}

	for (a = X; a <= Z; a++) {
		g[a] += RawGyro[a];
		if (RawGyro[a] > gMax[a])
			gMax[a] = RawGyro[a];
		else if (RawGyro[a] < gMin[a])
			gMin[a] = RawGyro[a];
	}
	t += MPU6XXXTemperature;


	if (++GyroErectionSamples >= NoOfSamples) {
		LEDOff(ledRedSel);
		LEDOff(ledBlueSel);

		Moving = false;
		for (a = X; a <= Z; a++) {
			g[a] /= NoOfSamples;
			gMax[a] -= g[a];
			gMin[a] -= g[a];
			Moving |= Max(Abs(gMax[a]), Abs(gMin[a])) > GYRO_MAX_SHAKE_RAW;
		}

		if (Moving) {
			DoBeep(2, 8);
			LEDToggle(ledRedSel);
		} else {
			for (a = X; a <= Z; a++)
				Config.GyroCal.C[a] = g[a];
			Config.GyroCal.TRef = t / NoOfSamples;
			UpdateConfig();

			if (CurrTelType == UAVXTelemetry)
				SendCalibrationPacket(TelemetrySerial);
		}

		GyrosErected = true;
	}
} // ErectGyros

/// Initialise all inertial filters in one place


real32 CurrAccLPFHz = 20.0f;
real32 CurrGyroLPFHz = 100.0f;
real32 CurrYawLPFHz = 75.0f;
real32 CurrServoLPFHz = 20.0f;
real32 CurrAltLPFHz = 20.0f;

filterM3Struct AccUM3F;
filterStruct AccF[3], GyroF[3], SensorTempF;

void InitInertialFilters(void) {

	const idx GyroLPFOrder = 2;
	const idx YawLPFOrder = 2;
	const idx AccLPFOrder = 2;

	const idx DerivLPFOrder = 1;

	idx a;

	const real32 rKF = 88;

	LPF1DriveK = initLPF1Coefficient(CurrGyroLPFHz, CurrPIDCycleS);
	LPF1ServoK = initLPF1Coefficient(CurrServoLPFHz, CurrPIDCycleS);

	CurrAccLPFHz = MPUAccLPFHz[CurrAccLPFSel];
	CurrGyroLPFHz = MPUGyroLPFHz[CurrGyroLPFSel];

	for (a = X; a <= Z; a++) {

		initLPFn(&AccF[a], AccLPFOrder, CurrAccLPFHz);

		if (a != Z) {
			initLPFn(&GyroF[a], GyroLPFOrder, CurrGyroLPFHz);
			initLPFn(&A[a].R.RateF, DerivLPFOrder, CurrGyroLPFHz * 0.6f); // derivative
		} else {
			initLPFn(&GyroF[a], GyroLPFOrder, CurrYawLPFHz);
			initLPFn(&A[a].R.RateF, DerivLPFOrder, CurrYawLPFHz * 0.6f); // derivative
		}

		// Pavel Holoborodko, see http://www.holoborodko.com/pavel/numerical-methods/
		// numerical-derivative/smooth-low-noise-differentiators/
		const real32 C[] = { 0.375f, 0.5f, -0.5f, -0.75, 0.125f, 0.25f };
		if (UsingPavelFilter)
			initFIRF(&A[a].R.RateDF, 6, C);
		else
			initMAF(&A[a].R.RateDF, 4);
	}

	AccUM3F.initialised = false;
	initMAF(&AccUMAF, MAF_ACCU_LEN);

	initLPFn(&SensorTempF, 2, 0.5f); // TODO: problem for SPI MPUxxx

} // InitInertialFilters

void InitIMU(uint8 imuSel) {
	idx a;
	boolean r;

	InitInertialFilters();

	memset(&Rate, 0, sizeof(Rate[3]));
	memset(&Acc, 0, sizeof(Acc[3]));
	Acc[Z] = -GRAVITY_MPS_S;

	InitMPU6XXX(imuSel);

	F.AccCalibrated = F.IMUCalibrated = Config.AccCal.Calibrated == 1;

} // InitIMU


