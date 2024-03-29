// ===============================================================================================
// =                                UAVX Quadrocopter ContRoller                                 =
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

//    You should have received a copy of the GNU General Public License aint32 with this program.
//    If not, see http://www.gnu.org/licenses/

#include "UAVX.h"

// 0.97, 2.9, 3.9, 5.9, 9.9, 17.85, 33.48ms
const uint16 MPUGyroLPFHz[] = { 250, 184, 98, 41, 20, 10, 5, 3600 };
uint8 CurrGyroLPFSel = 2; // 0 => 250Hz forces 8Khz sampling before DLPF

// 1.94, 5.8, 7.8, 11.8, 19.8, 35.7, 66.96, 1.94ms
const uint16 MPUAccLPFHz[] = { 480, 184, 92, 41, 20, 10, 5, 460 };
uint8 CurrAccLPFSel = 4; // V4 Board acc is always 1Khz sampling

const uint8 MPUDLPFMask[] = { MPU_RA_DLPF_BW_256, MPU_RA_DLPF_BW_188,
MPU_RA_DLPF_BW_98, MPU_RA_DLPF_BW_42, MPU_RA_DLPF_BW_20,
MPU_RA_DLPF_BW_10, MPU_RA_DLPF_BW_5 };

const char * DHPFName[] = { "Reset/0Hz", "5Hz", "2.5Hz", "1.25Hz", "0.63Hz",
		"?", "?", "Hold" };

uint8 MPU6XXXId;
uint8 MPU6XXXRev;
uint8 MPU6XXXDLPF = 0;
uint8 MPU6000DLPF = 0;
uint8 MPU6XXXDHPF = 0;

const real32 MPU6XXXRefTemperature = 21.0f;

real32 MPU6XXXTemperature = 25.0f;
int16 RawMPU6XXXTemperature;
timeuS mpu6xxxLastUpdateuS = 0;

real32 RawAcc[3], RawGyro[3];

real32 SlewBand;
int32 BP[3] = { 0, };

// Roll Right +, Pitch Up +, Yaw ACW +

void UpdateMPU6XXXTemperature(int16 T, real32 TempdT) {
	real32 R;

	switch (busDev[imuSel].type) {
	case mpu6050IMU:
		R = ((real32) T + 12456.0f) * 0.002941f;
		break;
	case icm20689IMU:
		R = (real32) T * (1.0f / 333.87f) + 21.0f;
		break;
	case mpu6000IMU:
	default:
		R = (real32) T * (1.0f / 333.87f) + 21.0f;
		break;
	}

	MPU6XXXTemperature = LPFn(&SensorTempF, R, TempdT);

} // UpdateMPU6XXXTemperature

void ReadRawAccAndGyro(void) {
	idx i;
	uint8 buf[14];
	int16 B[7];
	boolean r;
	idx a;

	r = SIOReadBlockataddr(imuSel, MPU_RA_ACC_XOUT_H, 14, buf);

	for (i = 0; i < 7; i++)
		B[i] = ((int16)buf[i*2]<<8) + buf[i*2+1];


	RotateSensor(&B[X], &B[Y], IMUQuadrant);

	for (a = X; a <= Z; a++) {
		RawAcc[a] = (real32) B[a];
		RawGyro[a] = (real32) B[a + 4];
	}

	RawMPU6XXXTemperature = B[3];

} // ReadRawAccAndGyro

void ReadFilteredGyroAndAcc(void) {
	static timeuS LastUpdateuS = 0;
	timeuS NowuS;
	real32 dT;
	uint8 a;

	NowuS = uSClock();
	dT = (NowuS - LastUpdateuS) * 0.000001f;
	LastUpdateuS = NowuS;

	ReadRawAccAndGyro();

	for (a = X; a <= Z; a++)
		switch (CurrIMUFilterType) {
		case MPUFilt:
			// using hardware DLPFs
			break;
		case ABGFilt:
			//ABGLPF(&ABGAccF[a], RawAcc[a], dT);
			RawAcc[a] = LPFn(&AccF[a], RawAcc[a], dT);
			ABGLPF(&ABGGyroF[a], RawGyro[a], dT);
			break;
		case LPFilt:
		default:
			RawAcc[a] = LPFn(&AccF[a], RawAcc[a], dT);
			RawGyro[a] = LPFn(&GyroF[a], RawGyro[a], dT);
			break;
		}

	UpdateMPU6XXXTemperature(RawMPU6XXXTemperature, dT);

} //ReadFilteredGyroAndAcc

void CalibrateAccZeros(uint8 s) {
	// Simple recalibration of accelerometers
	const int16 Samples = 300;
	const real32 SamplesR = 1.0f / (real32) Samples;

	int16 i;
	uint8 c;
	real32 a[3], g[3];
	real32 t;

	LEDOn(ledBlueSel);

	for (c = X; c <= Z; c++) {
		a[c] = g[c] = 0.0f;
		Config.AccCal.Scale[c] = DEF_ACC_SCALE;
		Config.AccCal.Bias[c] = Config.GyroCal.TempGradient[c] =
				Config.GyroCal.Bias[c] = 0.0f;
	}
	t = 0.0f;

	Delay1uS(2000);
	ReadFilteredGyroAndAcc();
	for (i = 0; i < Samples; i++) {
		Delay1uS(2000);
		ReadFilteredGyroAndAcc();
		t += MPU6XXXTemperature;
		RawAcc[Z] -= MPU_1G;
		for (c = X; c <= Z; c++) {
			g[c] += RawGyro[c];
			a[c] += RawAcc[c];
		}
	}

	for (c = X; c <= Z; c++) {
		a[c] *= SamplesR;
		g[c] *= SamplesR;
	}
	t *= SamplesR;

	Config.AccCal.ReferenceTemp = Config.GyroCal.ReferenceTemp = t;

	for (c = X; c <= Z; c++) {
		Config.AccCal.Bias[c] = a[c];
		Config.GyroCal.Bias[c] = g[c];
	}

	Config.AccCal.Calibrated = 1;
	F.IMUCalibrated = true;

	ConfigChanged = true;
	RefreshConfig();

	DoBeep(8, 1);
	LEDOff(ledBlueSel);

	SendAckPacket(s, UAVXMiscPacketTag, 1);

} // CalibrateAccZero

void CalibrateAccAndGyro(uint8 s) {
	// (C) G.K. Egan 2012
	// Basic idea from MEMSIC #AN-00MX-002 Ricardo Dao 4 Nov 2002
	// gyro and acc temperature calibration using linear compensation

	// PROBABLY NOT WORTH THE EFFORT
	// DOES NOT DO ACC SCALING - results in about a degree error at 45 degrees bank
	// GYRO DRIFT should be compensated by acc correction through Madgwick

	const int32 IntervaluS = 2000;
	const real32 RangeT = 10.0f;
	const int16 Samples = 20; // was 300 // number of samples to be used.
	const real32 SamplesR = 1.0f / (real32) Samples;

	int16 i;
	uint8 ts, c;
	real32 a[2][3], g[2][3];
	real32 t[2];
	real32 ThresholdT, TempDiff;

	LEDOn(ledBlueSel);

	for (c = X; c <= Z; c++) {
		for (i = 0; i <= 1; i++)
			a[i][c] = g[i][c] = t[i] = 0.0f;
		Config.AccCal.Scale[c] = DEF_ACC_SCALE;
		Config.AccCal.Bias[c] = Config.GyroCal.TempGradient[c] =
				Config.GyroCal.Bias[c] = 0.0f;
	}

	ts = 0;
	ThresholdT = -100.0f;
	do {
		Delay1uS(IntervaluS);
		ReadFilteredGyroAndAcc();
		if (MPU6XXXTemperature > ThresholdT) {
			for (i = 0; i < Samples; i++) {
				Delay1mS(2);
				ReadFilteredGyroAndAcc();
				t[ts] += MPU6XXXTemperature;
				RawAcc[Z] -= MPU_1G;
				for (c = X; c <= Z; c++) {
					g[ts][c] += RawGyro[c];
					a[ts][c] += RawAcc[c];
				}
			}
			ts++;
			ThresholdT = MPU6XXXTemperature + RangeT;
			if (ts < 2)
				DoBeep(1, 1);
		} else {
			Delay1mS(100);
			LEDToggle(ledBlueSel);
		}
	} while (ts <= 1);

	for (ts = 0; ts <= 1; ts++) {
		for (c = X; c <= Z; c++) {
			a[ts][c] *= SamplesR;
			g[ts][c] *= SamplesR;
		}
		t[ts] *= SamplesR;
	}

	Config.GyroCal.ReferenceTemp = t[0];
	TempDiff = t[1] - t[0];

	for (c = X; c <= Z; c++) {
		Config.AccCal.Bias[c] = (a[1][c] + a[0][c]) * 0.5f; // not worth the effort

		Config.GyroCal.TempGradient[c] = (g[1][c] - g[0][c]) / TempDiff;
		Config.GyroCal.Bias[c] = g[0][c]; // use starting temperature
	}

	F.IMUCalibrated = Abs(TempDiff) < (RangeT * 2.0f); // check if too fast!!!
	if (F.IMUCalibrated) {

		Config.AccCal.Calibrated = 1;

		DoBeep(8, 1);
		LEDOff(ledBlueSel);
		SendAckPacket(s, UAVXMiscPacketTag, 1);
	} else {
		Catastrophe();
		Config.AccCal.Calibrated = 0xff;
		SendAckPacket(s, UAVXMiscPacketTag, 255);
	}

} // CalibrateAccAndGyro

void UpdateGyroTempComp(void) {
	int32 a;

	for (a = X; a <= Z; a++)
#if defined(UAVXF4V4) // SPI temperature "unreliable"
		GyroBias[a] = Config.GyroCal.Bias[a];
#else
		GyroBias[a] = Config.GyroCal.Bias[a]
				+ Config.GyroCal.TempGradient[a]
						* (MPU6XXXTemperature - Config.GyroCal.ReferenceTemp);
#endif
} // UpdateGyroTempComp

void InitMPU6XXX(void) {

	// THE MPU6000? USED ON UAVXARM32F4V4 NANO BOARDS ARE VERY VERY NOISY.
	// MUCH MORE SO THAN ON OMNIBUS CLONES AND THE V3 I2C MPU6050 - CHECK WITH KEN.

	// VERY IMPORTANT: ACCELEROMETER MAX SAMPLING RATE IS 1KHZ. IF READ FASTER THEN VALUES REPEAT.
	// GYROS ARE SAMPLED AT 1KHZ UNLESS DISABLED WHEN THE SAMPLING RATE IS 8KHZ
	// THE UPDATING OF REGISTERS IS ASYNCHRONOUS
	// THE MPU6050 HAS COMMON DLPF CONFIG FOR ACC/GYRO, THE MPU6500 HAS A SEPARATE DLPF CONFIG FOR ACC

	CheckMPU6XXXActive();
	Delay1mS(100); // was 5

	uint8 DisableAccDLPF = CurrIMUFilterType == MPUFilt ? 1:0;
	uint8 DisableGyroDLPF = CurrIMUFilterType == MPUFilt ? 1:0;

	switch (busDev[imuSel].type) {
	case mpu6050IMU:

		SIOWrite(imuSel, MPU_RA_PWR_MGMT_1, 1 << MPU_RA_PWR1_DEVICE_RESET_BIT);
		Delay1mS(100);

		SIOWrite(imuSel, MPU_RA_FIFO_EN, 0); // DISABLE FIFOs

		SIOWrite(imuSel, MPU_RA_SMPLRT_DIV, 0); // NO sampling rate division - full speed
		SIOWrite(imuSel, MPU_RA_PWR_MGMT_1, MPU_RA_CLOCK_PLL_XGYRO);

		MPU6XXXRev = SIORead(imuSel, MPU_RA_PRODUCT_ID);

		SIOWrite(imuSel, MPU_RA_GYRO_CONFIG, (MPU_RA_GYRO_FS_2000 << 3));
		SIOWriteataddr(imuSel, MPU_RA_ACC_CONFIG,
				(MPU_RA_ACC_FS_4 << 3) | MPU_RA_DHPF_1P25);

		SIOWriteataddr(imuSel, MPU_RA_ACC_CONFIG2,
				(DisableAccDLPF << 3) & MPUDLPFMask[CurrAccLPFSel]);

		SIOWrite(imuSel, MPU_RA_INT_PIN_CFG,
				(1 << MPU_RA_INTCFG_I2C_BYPASS_EN_BIT));

		Delay1mS(100);

		SIOWriteataddr(imuSel, MPU_RA_CONFIG,
				(DisableGyroDLPF << 3) & MPUDLPFMask[CurrGyroLPFSel]);

		Delay1mS(100);

		MPU6XXXDLPF = SIOReadataddr(imuSel, MPU_RA_CONFIG) & 0x07;
		MPU6XXXDHPF = SIOReadataddr(imuSel, MPU_RA_ACC_CONFIG) & 0x07;

		Delay1mS(100); // added to prevent apparent SPI hang

		break;
	case icm20689IMU:
	case mpu6000IMU:

		SIOWrite(imuSel, MPU_RA_PWR_MGMT_1, 1 << MPU_RA_PWR1_DEVICE_RESET_BIT);
		Delay1mS(100);

		SIOWrite(imuSel, MPU_RA_SIGNAL_PATH_RESET, 7); // reset gyro, acc, temp
		Delay1mS(100);

		SIOWrite(imuSel, MPU_RA_FIFO_EN, 0); // DISABLE FIFOs

		SIOWrite(imuSel, MPU_RA_SMPLRT_DIV, 0); // NO sampling rate division - full speed
		SIOWrite(imuSel, MPU_RA_PWR_MGMT_1, MPU_RA_CLOCK_PLL_XGYRO);

		MPU6XXXRev = SIORead(imuSel, MPU_RA_PRODUCT_ID);

		SIOWrite(imuSel, MPU_RA_GYRO_CONFIG, (MPU_RA_GYRO_FS_2000 << 3));

		SIOWriteataddr(imuSel, MPU_RA_ACC_CONFIG,
				(MPU_RA_ACC_FS_4 << 3) | MPU_RA_DHPF_1P25);

		SIOWriteataddr(imuSel, MPU_RA_ACC_CONFIG2,
				(DisableAccDLPF << 3) & MPUDLPFMask[CurrAccLPFSel]);

		Delay1mS(100);

		SIOWriteataddr(imuSel, MPU_RA_CONFIG,
				(DisableGyroDLPF << 3) & MPUDLPFMask[CurrGyroLPFSel]);

		Delay1mS(100);

		MPU6XXXDLPF = SIOReadataddr(imuSel, MPU_RA_CONFIG) & 0x07;
		MPU6XXXDHPF = SIOReadataddr(imuSel, MPU_RA_ACC_CONFIG) & 0x07;
		MPU6000DLPF = SIOReadataddr(imuSel, MPU_RA_ACC_CONFIG2) & 0x07;

		break;
	} // switch

} // InitMPU6XXX

boolean MPU6XXXReady(void) {

	return (SIORead(imuSel, MPU_RA_INT_STATUS) && 1) != 0;
} // MPU6XXXReady

void CheckMPU6XXXActive(void) {
	boolean r;

	Delay1mS(35);

	MPU6XXXId = SIORead(imuSel, MPU_RA_WHO_AM_I);
	if (busDev[imuSel].useSPI)
		r = MPU6XXXId > 0;
	else
		r = MPU6XXXId == ((busDev[imuSel].i2cId >> 1) & 0xfe);

	F.IMUActive = r;

} // CheckMPU6XXXActive

