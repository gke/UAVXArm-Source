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
const uint8 CurrGyroLPFSel = 2; // 0 => 250Hz forces 8Khz sampling before DLPF

// 1.94, 5.8, 7.8, 11.8, 19.8, 35.7, 66.96, 1.94ms
const uint16 MPUAccLPFHz[] = { 480, 184, 92, 41, 20, 10, 5, 460 };
const uint8 CurrAccLPFSel = 4; // V4 Board acc is always 1Khz sampling

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

real32 MPU6XXXTemperature = 25.0f;
uint8 MPU_ID = MPU_0x68_ID;
uint32 mpu6xxxLastUpdateuS = 0;

real32 RawAcc[3], RawGyro[3];

uint32 Noise[8];
uint32 gyroGlitches;
uint32 mpuReads;

uint16 SlewLimitGyroClicks = 64;
uint16 SlewHistScale = 8;

void ComputeMPU6XXXTemperature(int16 T) {

#if defined(V4_BOARD)
	MPU6XXXTemperature = (real32) (T + 0) * (1.0f/333.87f) + 21.0f;
#else
	MPU6XXXTemperature = ((real32) T + 12456.0f) * 0.002941f;
#endif

} // ComputeMPU6XXXTemperature

void ReadAccAndGyro(boolean UseSelectedAttSensors) { // Roll Right +, Pitch Up +, Yaw ACW +
	int16 B[7];
	idx a;
	static int16 BP[7] = { 0, 0, 0, 0, 0, 0, 0 };

	sioReadBlocki16vataddr(SIOIMU, MPU_ID, MPU_RA_ACC_XOUT_H, 7, B, true);

	mpu6xxxLastUpdateuS = uSClock();
	mpuReads++;

	if ((CurrAttSensorType == InfraRedAngle) && !IsMulticopter) {

		// scale reading for angle arcsin(adc)

		// z axis computed from x,y?

	} else {
		RawAcc[0] = (real32) B[0];
		RawAcc[1] = (real32) B[1];
		RawAcc[2] = (real32) B[2];
	}

	ComputeMPU6XXXTemperature(B[3]);

	for (a = 4; a <= 6; a++) {
#if !defined(INC_DFT)
		Noise[Limit(Abs(B[a] - BP[a]) / SlewHistScale, 0, 7)]++;
#endif
		if (P(GyroSlewRate) > 0)
			B[a] = SensorSlewLimit(GyroFailS, &BP[a], B[a], SlewLimitGyroClicks);
	}

	if (UseSelectedAttSensors)
		switch (CurrAttSensorType) {
		case InfraRedAngle:
		case UAVXArm32IMU:
		case FreeIMU:
			RawGyro[X] = (real32) B[4];
			RawGyro[Y] = (real32) B[5];
			RawGyro[Z] = (real32) B[6];
			break;
		default: // MLX90609Gyro, ADXRS150Gyro, LY530Gyro, ADXRS300Gyro,
			RawGyro[Pitch] = analogRead(PitchAnalogSel);
			RawGyro[Roll] = analogRead(RollAnalogSel);
			RawGyro[Yaw] = analogRead(YawAnalogSel);
		}
	else { // MPU6xxx
		RawGyro[X] = (real32) B[4];
		RawGyro[Y] = (real32) B[5];
		RawGyro[Z] = (real32) B[6];
	}

	//	CurrAttSensorType == InfraRed

} // ReadAccAndGyro


void CalibrateAccAndGyro(uint8 s) {
	// (C) G.K. Egan 2012
	// Basic idea from MEMSIC #AN-00MX-002 Ricardo Dao 4 Nov 2002
	// gyro and acc temperature calibration using linear compensation
	const real32 RangeT = 10.0f;
	const int16 Samples = 300; // number of samples to be used.
	const real32 SamplesR = 1.0f / (real32) Samples;

	int16 i;
	uint8 ts, c;
	real32 a[2][3], g[2][3];
	real32 t[2];
	real32 ThresholdT, TempDiff;

	LEDOn(LEDBlueSel);

	for (c = X; c <= Z; c++) {
		for (i = 0; i < 2; i++)
			a[i][c] = g[i][c] = t[i] = 0.0f;
		GyroBias[c] = 0.0f;
	}

	ts = 0;
	ThresholdT = -100.0f;
	do {
		ReadAccAndGyro(false);
		if (MPU6XXXTemperature > ThresholdT) {
			for (i = 0; i < Samples; i++) {
				ReadAccAndGyro(false);
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
			LEDToggle(LEDBlueSel);
		}
	} while (ts < 2);

	for (ts = 0; ts < 2; ts++) {
		for (c = X; c <= Z; c++) {
			a[ts][c] *= SamplesR;
			g[ts][c] *= SamplesR;
		}
		t[ts] *= SamplesR;
	}

	NV.GyroCal.TRef = t[0];
	TempDiff = t[1] - t[0];

	for (c = X; c <= Z; c++) {
		NV.AccCal.Scale[c] = DEF_ACC_SCALE;
		NV.AccCal.Bias[c] = (a[0][c] + a[1][c]) * 0.5f;
		NV.GyroCal.M[c] = (g[1][c] - g[0][c]) / TempDiff;
		GyroBias[c] = NV.GyroCal.C[c] = g[0][c]; // use starting temperature
	}

	F.IMUCalibrated = Abs(TempDiff) < (RangeT * 2.0f); // check if too fast!!!
	if (F.IMUCalibrated) {

		NVChanged = true;
		UpdateNV();
		UpdateGyroTempComp();
		DoBeep(8, 1);
		LEDOff(LEDBlueSel);
		SendAckPacket(s, UAVXMiscPacketTag, 1);
	} else {
		Catastrophe();
		SendAckPacket(s, UAVXMiscPacketTag, 255);
	}

} // CalibrateAccAndGyro


void UpdateGyroTempComp(void) {
	int32 a;

	if (!F.UsingAnalogGyros) // keep using erection bias if analog gyros
		for (a = X; a <= Z; a++)
			GyroBias[a] = NV.GyroCal.C[a] + NV.GyroCal.M[a]
					* (MPU6XXXTemperature - NV.GyroCal.TRef);

} // UpdateGyroTempComp


void InitMPU6XXX(void) {

	// VERY IMPORTANT: ACCELEROMETER MAX SAMPLING RATE IS 1KHZ. IF READ FASTER THEN VALUES REPEAT.
	// GYROS ARE SAMPLED AT 1KHZ UNLESS DISABLED WHEN THE SAMPLING RATE IS 8KHZ
	// THE UPDATING OF REGISTERS IS ASYNCHRONOUS
	// THE MPU6050 HAS COMMON DLPF CONFIG FOR ACC/GYRO, THE MPU6500 HAS A SEPARATE DLPF CONFIG FOR ACC

#if defined(USE_MPU_DLPF)
	const uint8 DisableGyroDLPF = 0;
#else
	const uint8 DisableGyroDLPF = 1; // Gyro 1 -> 8800Hz 2 -> 3600Hz
#endif


	CheckMPU6XXXActive();
	Delay1mS(100); // was 5

	sioWrite(SIOIMU, MPU_ID, MPU_RA_PWR_MGMT_1, 1
			<< MPU_RA_PWR1_DEVICE_RESET_BIT);
	Delay1mS(100);

	if (spiDevUsed[SIOIMU]) {
		sioWrite(SIOIMU, MPU_ID, MPU_RA_SIGNAL_PATH_RESET, 7); // reset gyro, acc, temp
		Delay1mS(100);
	}

	sioWrite(SIOIMU, MPU_ID, MPU_RA_FIFO_EN, 0); // DISABLE FIFOs

	sioWrite(SIOIMU, MPU_ID, MPU_RA_SMPLRT_DIV, 0); // NO sampling rate division - full speed
	sioWrite(SIOIMU, MPU_ID, MPU_RA_PWR_MGMT_1, MPU_RA_CLOCK_PLL_XGYRO);

	MPU6XXXRev = sioRead(SIOIMU, MPU_ID, MPU_RA_PRODUCT_ID);

	sioWrite(SIOIMU, MPU_ID, MPU_RA_GYRO_CONFIG, (MPU_RA_GYRO_FS_2000 << 3));

	uint8 MPUAccFS = MPU_RA_ACC_FS_4; // +/-4g
	sioWriteataddr(SIOIMU, MPU_ID, MPU_RA_ACC_CONFIG, (MPUAccFS << 3)
			| MPU_RA_DHPF_1P25);

#if defined(V4_BOARD) // MPU6500

#if defined(USE_MPU_DLPF)
	const uint8 DisableAccDLPF = 0;  // Acc 1KHz
#else
	const uint8 DisableAccDLPF = 1;  // Acc 1KHz
#endif
	sioWriteataddr(SIOIMU, MPU_ID, MPU_RA_ACC_CONFIG2, (DisableAccDLPF << 3) & MPUDLPFMask[CurrAccLPFSel]);
#else

	// Enable I2C master mode
	uint8 v = sioReadataddr(SIOIMU, MPU_ID, MPU_RA_USER_CTRL);
	bitClear(v, MPU_RA_USERCTRL_I2C_MST_EN_BIT);
	sioWrite(SIOIMU, MPU_ID, MPU_RA_USER_CTRL, v);

	// Allow bypass access to slave I2C devices (Magnetometer)
	v = sioReadataddr(SIOIMU, MPU_ID, MPU_RA_INT_PIN_CFG);
	bitSet(v, MPU_RA_INTCFG_I2C_BYPASS_EN_BIT);
	sioWrite(SIOIMU, MPU_ID, MPU_RA_INT_PIN_CFG, v);

#endif

	Delay1mS(100);

	sioWriteataddr(SIOIMU, MPU_ID, MPU_RA_CONFIG, (DisableGyroDLPF << 3)
			& MPUDLPFMask[CurrGyroLPFSel]);

	Delay1mS(100);

	MPU6XXXDLPF = sioReadataddr(SIOIMU, MPU_ID, MPU_RA_CONFIG) & 0x07;
	MPU6XXXDHPF = sioReadataddr(SIOIMU, MPU_ID, MPU_RA_ACC_CONFIG) & 0x07;
#if defined(V4_BOARD)
	MPU6000DLPF = sioReadataddr(SIOIMU, MPU_ID, MPU_RA_ACC_CONFIG2) & 0x07;
#endif

	Delay1mS(100); // added to prevent apparent SPI hang

} // InitMPU6XXX


void CheckMPU6XXXActive(void) {
	boolean r;

#if defined(BRICE)
	MPU_ID = MPU_0x69_ID;
	MPU6XXXId = sioRead(SIOIMU, MPU_ID, MPU_RA_WHO_AM_I);
	r = true;
#else
#if defined(V4_BOARD)
	MPU6XXXId = sioRead(SIOIMU, MPU_ID, MPU_RA_WHO_AM_I);
	r = MPU6XXXId == 0x70;
#else
	Delay1mS(35);
	MPU_ID = MPU_0x69_ID;
	MPU6XXXId = sioRead(SIOIMU, MPU_ID, MPU_RA_WHO_AM_I);
	r = MPU6XXXId == ((MPU_ID >> 1) & 0xfe);
	if (!r) {
		Delay1mS(35);
		MPU_ID = MPU_0x68_ID;
		MPU6XXXId = sioRead(SIOIMU, MPU_ID, MPU_RA_WHO_AM_I);
		r = MPU6XXXId == ((MPU_ID >> 1) & 0xfe);
	}
#endif
#endif

	F.IMUActive = r;

} // CheckMPU6XXXActive


