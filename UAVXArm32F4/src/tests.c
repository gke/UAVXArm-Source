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

#include "UAVX.h"

void ShowSIODeviceName(uint8 s, uint8 d) {

	// could be a full table lookup?

	TxChar(s, ' ');
	switch (d) {
	case MPU_0x68_ID:
	case MPU_0x69_ID:
		TxString(s, "Invensense Gyro/IMU");
		break;
	case HMC5XXX_ID:
		TxString(s, "HMC5XXX Mag");
		break;
	case IST8310_ID:
		TxString(s, "IST8310 Mag");
		break;
	case TMP100_ID:
		TxString(s, "Temperature");
		break;
	case EEPROM_ID:
		TxString(s, "EEPROM");
		break;
	default:
		break;
	} // switch
	TxChar(s, ' ');

} // ShowSIODeviceName

uint8 ScanSIOBus(uint8 s, uint8 bus) {
	uint8 nd, d, v;

	nd = 0;

	for (d = 0x10; d <= 0xf6; d += 2) {
		v = 0;
		if (I2CReadBlock(bus, d, 0, 1, &v)) {
			nd++;
			TxString(s, "\t0x");
			TxValH(s, d);
			TxChar(s, ' ');
			TxVal32(s, v, 0, ' ');
			ShowSIODeviceName(s, d);
			TxNextLine(s);
		}
		Delay1mS(1);
	}

	return (nd);
} // ScanSIOBus

void DoTesting(void) {

	//#define COMMISSIONING_TEST
	//#define TEMP_COMP_TESTING
	//#define USB_TESTING
	//#define KF_TESTING
	//#define BARO_TESTING
	//#define BARO_RAW_TESTING
	//#define MAG_CAL_TESTING
	//#define CURRENT_TESTING
	//#define EXTMEM_TESTING

	idx i, c, a;
	int32 d;
	static int16 MaxShadow[7];
	static int16 MinShadow[7];
	static int16 PrevShadow[7];
	static int32 MaxDelta[7];
	uint8 buf[32];

#if defined(EXTMEM_TESTING)

	for (i = 0; i < 100; i++) {
		Delay1mS(2);
		InitNVMem();
	}
	while (true) {
	};

#elif defined(MADGWICK_TESTING)

	i = 0;
	d = 0;

	static timeuS LastUpdateuS = 0;

	TxString(TelemetrySerial,"Angle, IntRate ");
	TxNextLine(TelemetrySerial);

	KpAccBase = 2.0f;
	KiAccBase = 0.0f;
	KpMag = 0.5f;

	while (true)

	if (uSTimeout(CycleUpdateuS)) {
		uSTimer(CycleUpdateuS, CurrPIDCycleuS);

		//	Marker();

		dT = dTUpdate(&LastUpdateuS);
		dTOn2 = 0.5f * dT;
		dTR = 1.0f / dT;
		dTROn2 = dTR * 0.5f;

		if (++d == 2000) {
			d = 0;
			IntRate[Pitch] = Angle[Pitch];
		}

		UpdateInertial();

		if (++i == 10) {
			i = 0;

			//TxVal32(TelemetrySerial, Rate[Pitch] * 1000.0f, 3, ',');
			//TxVal32(TelemetrySerial, Acc[BF]* 1000.0f, 3, ',');
			TxVal32(TelemetrySerial, Angle[Yaw]* 1000.0f, 3, ',');
			TxVal32(TelemetrySerial, IntRate[Yaw]* 1000.0f, 3, ',');
			TxNextLine(TelemetrySerial);

		}

	}

#elif defined(ACC_TESTING)

	// max/min acc for scaling check

	int32 Max[3], Min[3];

	for (a = X; a <= Z; a++) {
		Max[a] = -32767;
		Min[a] = 32768;
	}

	while (true) {

#define KPACC 0.99f

		Delay1mS(2);

		ReadFilteredGyroAndAcc(imuSel);

		for (a = X; a <= Z; a++) {
			if (RawAcc[a] > Max[a])
			Max[a] = Max[a] * KPACC + RawAcc[a] * (1.0f - KPACC);
			else if (RawAcc[a] < Min[a])
			Min[a] = Min[a] * KPACC + RawAcc[a] * (1.0f - KPACC);
			//TxVal32(TelemetrySerial, a, 0, ',');
			//TxVal32(TelemetrySerial, Min[a], 0, ',');
			//TxVal32(TelemetrySerial, Max[a], 0, ',');
			TxVal32(TelemetrySerial, Max[a] - Min[a], 0, ',');
			TxVal32(TelemetrySerial, (Max[a] + Min[a])/2, 0, ',');

		}
		TxNextLine(TelemetrySerial);

	}

#elif defined(MAG_TESTING)

	uint8 v = 0;

//	while(1) {
//		SIOReadBlock(CurrMagSel, HMC5XXX_TAG, 1, &v);
//		Delay1mS(1);
//	}
//	SIOWriteBlockataddr(CurrMagSel, HMC5XXX_CONFIG_A, 1, tx);


	timemS NextUpdatemS = 0;

	mSTimer(MagnetometerUpdatemS, MAG_TIME_MS);
	while (true) {
		if (mSTimeout(MagnetometerUpdatemS)) {

			mSTimer(MagnetometerUpdatemS, MAG_TIME_MS);

			GetMagnetometer();

			if (mSClock() > NextUpdatemS) {

				NextUpdatemS = mSClock() + 1000;
				for (i = 0; i < 3; i++)
					TxVal32(TelemetrySerial, RawMag[i], 0, ' ');
				TxNextLine(TelemetrySerial);

			}
		}

	}


#elif defined(RC_TESTING)

	static timemS LastUpdatemS = 0;
	timemS NowmS;

	CurrRxType = FutabaSBusRx;
	InitRC();

	while (true) {
		CheckRC();

		if (F.RCNewValues) {

			//	if (RCInp[0].Raw < 2000) {

			NowmS = mSClock();

			TxVal32(TelemetrySerial, NowmS - LastUpdatemS, 0, ' ');
			LastUpdatemS = NowmS;

			for (i = 0; i < 4; i++)
			TxVal32(TelemetrySerial, RCInp[i].Raw, 0, ',');

			if (SBusSignalLost)
			TxString(TelemetrySerial, "LOST ");
			if (SBusFailsafe)
			TxString(TelemetrySerial, "FS ");

			TxNextLine(TelemetrySerial);
			//}

			F.RCNewValues = false;

		}

	}

#elif defined(TEMP_COMP_TESTING)

	//CalibrateAccAndGyro(TelemetrySerial, imuSel);
	LEDsOff();

	for (i = 0; i < 7; i++) {
		MaxShadow[i] = -32000;
		MinShadow[i] = 32000;
		MaxDelta[i] = 0;
	}

	c = 0;
	while (true) {
		Delay1mS(1);
		ReadFilteredGyroAndAcc(imuSel);
		ScaleRateAndAcc(imuSel);

		for (i = 0; i < 7; i++) {
			if (ShadowRawIMU[i] > MaxShadow[i])
			MaxShadow[i] = ShadowRawIMU[i];
			else if (ShadowRawIMU[i] < MinShadow[i])
			MinShadow[i] = ShadowRawIMU[i];

			MaxDelta[i] = ShadowRawIMU[i] - PrevShadow[i];
			PrevShadow[i] = ShadowRawIMU[i];

		}

		if (++c >= 2)
		{
			c = 0;

			// for (i = 0; i < 3; i++) {
			// TxVal32(TelemetrySerial, ShadowRawIMU[i], 0, ',');
			// TxVal32(TelemetrySerial, RawAcc[i] * 10.0f, 1, ',');
			// TxVal32(TelemetrySerial, MinShadow[i], 0, ',');
			// TxVal32(TelemetrySerial, MaxShadow[i], 0, ',');
			// }

			//for (i = 0; i < 3; i++) {
			i = 0;
			//}
			TxVal32(TelemetrySerial, ShadowRawIMU[i + 4], 0, ',');
			TxVal32(TelemetrySerial, RawGyroX[i] * 10.0f, 1, ',');
			TxVal32(TelemetrySerial, RawGyro[i] * 10.0f, 1, ',');
			//TxVal32(TelemetrySerial, MinShadow[i + 4], 0, ',');
			//TxVal32(TelemetrySerial, MaxShadow[i + 4], 0, ',');
			//TxVal32(TelemetrySerial, MaxDelta[i + 4], 0, ',');
			//}

			//TxVal32(TelemetrySerial, ShadowRawIMU[4], 0, ',');
			//TxVal32(TelemetrySerial, MaxShadow[4], 0, ',');
			//TxVal32(TelemetrySerial, MPU6XXXTemperature * 1000.0f, 3, ',');

			TxNextLine(TelemetrySerial);
			LEDToggle(ledGreenSel);
		}
	};

#elif defined(USB_TESTING)

	LEDOn(ledGreenSel);

	//USBTxString("starting USB Test (! to force restart)\n");

	while (true) {

		//LEDChaser();

		Delay1mS(500);
		//SendExecutionTimeStatus(TelemetrySerial);

		//SendUAVXTelemetry(USBSerial);
		//SendSerialPortStatus(USBSerial);
		SendMinPacket(USBSerial);

		//if (SerialAvailable(USBSerial)) {
		//	LEDToggle(ledYellowSel);
		//	uint8 ch = RxChar(USBSerial);
		//	TxChar(USBSerial, ch);
		//}
		//TxChar(USBSerial, '?');

	}

#elif defined(COMMISSIONING_TEST)

	ReadBlockNV(0, sizeof(NV), (int8 *) (&NV));

	Config.CurrPS = 0;
	for (i = 0; i < MAX_PARAMETERS; i++)
	SetP(DefaultParams[i].tag, DefaultParams[i].p[0]);

	CommissioningTest(0);

#elif defined(BARO_TESTING)

	int16 kkk, cycles;
	timeuS start = uSClock();

	for (kkk = 0; kkk < 256; kkk++)
	LSBBaro[kkk] = 0;
	LEDsOff();

	cycles = 10;
	for (kkk = 0; kkk < 3000; kkk++) {
		while (!DEBUGNewBaro) {
			GetBaro();
			SIOTokenFree = true;
		}
		DEBUGNewBaro = false;

		if (cycles-- <= 0) {
			LEDToggle(ledGreenSel);
			cycles = 10;
			TxVal32(TelemetrySerial, BaroTempVal, 0, ',');
			TxVal32(TelemetrySerial, BaroPressVal, 0, ',');
			TxVal32(TelemetrySerial, BaroTemperature * 1000, 3, ',');
			TxVal32(TelemetrySerial, BaroPressure * 100, 2, ',');
			TxVal32(TelemetrySerial, DensityRawAltitude * 1000, 3, ',');
			TxVal32(TelemetrySerial, DensityAltitude * 1000, 3, ',');
			TxNextLine(TelemetrySerial);
		}
	}

	TxVal32(TelemetrySerial, (uSClock() - start) / 3000, 3, ',');
	TxNextLine(TelemetrySerial);

	for (kkk = 0; kkk < 256; kkk++) {
		TxVal32(TelemetrySerial, kkk, 0, ',');
		TxNextLine(TelemetrySerial);
	}

	LEDsOn();
	while (true) {
	};

#elif defined(BARO_RAW_TESTING)
	int16 kkk;

	for (kkk = 0; kkk < 10000; kkk++) {
		while (!DEBUGNewBaro) {
			GetBaro();
			SIOTokenFree = true;
		}
		DEBUGNewBaro = false;

		LEDToggle(ledRedSel);

		//TxVal32(TelemetrySerial, BaroTempVal, 0, ',');
		TxVal32(TelemetrySerial, BaroPressVal, 0, ',');
		//TxVal32(TelemetrySerial, BaroTemperature * 1000, 3, ',');
		TxVal32(TelemetrySerial, BaroPressure * 100, 2, ',');
		TxNextLine(TelemetrySerial);

	}

	LEDOn(ledGreenSel);

	while(1) {};

#elif defined(KF_TESTING)

	const real32 SampleHz = 1000.0f;
	const real32 NyquistHz = 500.0f;
	const real32 PIDHz = 500;
	const real32 GyroHz = 100.0f;

	real32 OverSampledT = 1.0f / SampleHz;
	real32 PIDdT = 1.0f/ PIDHz;

	filterStruct OSLPF, FinalLPF;
	filterStructABG ABGF;

	int kkk;
	real32 AB, OSRC, RC, RN;

	timeuS NowuS = uSClock();
	timemS NextmS = mSClock();

	timeuS PrevuS = NowuS;

	kkk = 0;

	const real32 GyroSignalHz = 50.0f;
	const real32 s2Hz = 150.0f;
	const real32 s3Hz = 230.0f;
	const real32 s4Hz = 330.0f;

	real32 TimeS = 0.0f;
	real32 NextTimeS = 0.0f;

	initABGLPF(&ABGF, 500.0f, 1.0f/GyroHz, UNDER_DAMPED);
	initLPFn(&OSLPF, 2, NyquistHz);
	initLPFn(&FinalLPF, 1, GyroHz);

	TxString(0, "TimemS, Noise, Raw, AB, RC, RC2");
	TxNextLine(TelemetrySerial);
	do {

		// mix signals with offsets - could add some noise
		RN = 1.0f * (real32) rand()/RAND_MAX;
		real32 v =
		sinf(TWO_PI * GyroSignalHz * TimeS)
		+ sinf(TWO_PI * s2Hz * TimeS)
		+ sinf(TWO_PI * s3Hz * TimeS) + sinf(TWO_PI * s4Hz * TimeS)
		+ RN;

		AB = ABGLPF(&ABGF, v);

		OSRC = LPFn(&OSLPF, v, OverSampledT);

		if (TimeS > NextTimeS) {
			NextTimeS = TimeS + PIDdT;
			RC = LPFn(&FinalLPF, OSRC, GyroHz);
		}

		TxVal32(TelemetrySerial, TimeS * 1000000, 3, ',');
		TxVal32(TelemetrySerial, RN * 100, 2, ',');
		TxVal32(TelemetrySerial, v * 100, 2, ',');
		TxVal32(TelemetrySerial, AB * 100, 2, ',');
		TxVal32(TelemetrySerial, OSRC * 100, 2, ',');
		TxVal32(TelemetrySerial, RC * 100, 2, ',');
		TxNextLine(TelemetrySerial);

		TimeS += OverSampledT;

	}while (TimeS < 1.0f);

	LEDsOn();
	while (true) {
	};
#elif defined(MAG_CAL_TESTING)

	int16 ii, jj;

	CalibrateHMC5XXX(0, false);

	for (ii = 0; ii < MAG_CAL_SAMPLES; ii++) {

		TxVal32(TelemetrySerial, ii, 0, ',');

		for (jj = 0; jj <= 2; jj++)
		TxVal32(TelemetrySerial, MagSample[ii][jj], 0, ',');

		TxNextLine(TelemetrySerial);
	}

	TxNextLine(TelemetrySerial);
	for (jj = 0; jj <= 2; jj++)
	TxVal32(TelemetrySerial, Config.MagCal.Bias[jj], 0, ',');
	TxNextLine(TelemetrySerial);

	while (1) {
		Delay1mS(200);
		LEDToggle(ledRedSel);
	}
#elif defined(CURRENT_TESTING)

	real32 A;

	BatteryCurrentADCZero = 0.5f;

	for (i = 0; i < 1000; i++) {
		Delay1mS(10);
		BatteryCurrentADCZero = BatteryCurrentADCZero * 0.99f + analogRead(BattCurrentAnalogSel) * 0.01f;
		LEDToggle(ledYellowSel);
	}

	BatteryCurrent = 0.0f;

	while (1) {

		Delay1mS(10);

		A = analogRead(BattCurrentAnalogSel);

		BatteryCurrent = BatteryCurrent * 0.99f + ((A - BatteryCurrentADCZero) * (3.3f / 0.04f)) * 0.01f;

		TxVal32(TelemetrySerial, BatteryCurrentADCZero * 1000.0f, 3, ' ');
		TxVal32(TelemetrySerial, A * 1000.0f, 3, ' ');
		TxVal32(TelemetrySerial, BatteryCurrent * 1000.0f, 3, ' ');
		TxNextLine(TelemetrySerial);

		LEDToggle(ledRedSel);
	}

#elif defined(TESTING_OPTICAL)

	adns3080init();

	while (1) {
		adns3080update_position(Angle[Roll], Angle[Pitch], cosf(Angle[Yaw]),
				sinf(Angle[Yaw]), Altitude);
		adns3080print_pixel_data(TelemetrySerial);
	}

#else

	// NO TESTS
#endif

} // DoTesting

#if defined(COMMISSIONING_TEST)

index i;

void OK(uint8 s, boolean b) {
	if (b)
	TxString(s, " OK ");
	else
	TxString(s, " FAIL ");
} // OK

void Calibrated(uint8 s, boolean b) {
	if (!b)
	TxString(s, " NOT calibrated ");
} // Calibrated

void CommissioningTest(uint8 s) {
	timeuS Timeout;
	index k, i;
	int8 pattern[32];
	uint8 info[32];

#if defined(UAVXF4V4)
	for (i = 0; i < 4; i++)
	SPISelect(i, false); // do it again but why is this being changed?
#endif

	LEDOn(ledRedSel);
	Delay1mS(10000);
	LEDOn(ledBlueSel);

	LEDsAndBuzzer(s);

#define FULL_TEST
#if defined(FULL_TEST)

	InitIMU();

#define HMC5XXX_CONFIG_A 	0x00
#define HMC5XXX_CONFIG_B 	0x01
#define HMC5XXX_MODE 		0x02
#define HMC5XXX_DATA 		0x03
#define HMC5XXX_STATUS 		0x09
#define HMC5XXX_TAG 		0x0a

	TxString(s, "MAG:");

	InitMagnetometer();

	OK(s, F.MagnetometerActive);
	SIOReadBlock(SIOMag, HMC5XXX_ID, 0, 3, info);
	for (k = 0; k < 3; k++) {
		TxString(s, " 0x");
		TxValH(s, info[k]);
	}
	Delay1mS(50);
	SIOReadBlock(SIOMag, HMC5XXX_ID, HMC5XXX_TAG, 3, info);
	TxString(s, " Id = ");
	for (k = 0; k < 3; k++)
	TxChar(s, info[k]);
	TxNextLine(s);

	TxString(s, "MEM:");
	InitNVMem();

	OK(s, F.HaveNVMem);
	ShowStatusNVMem(s);

	for (k = 0; k < 8; k++)
	TxChar(s, 'A' + k);

	for (k = 0; k < 8; k++)
	pattern[k] = 'A' + k;
	WriteNVMemBlock(0, 8, pattern);
	for (k = 0; k < 8; k++)
	pattern[k] = '?';

	TxString(s, "-> ");

	ReadBlockNVMem(0, 8, pattern);
	for (k = 0; k < 8; k++)
	TxChar(s, pattern[k]);

	TxNextLine(s);

	InitIMU();
	ReadFilteredGyroAndAcc(); ScaleRateAndAcc();

	if (false)
	InertialTest(s);
	else {
		TxString(s, "IMU:");
		OK(s, F.IMUActive);
		TxString(s, "WhoAmI = 0x");
		TxValH(s, MPU6XXXId);
		TxChar(s, ' ');
		TxVal32(s, MPU6XXXTemperature * 10.0f, 1, 'C');
		TxChar(s, ' ');
		for (i = 0; i < 3; i++)
		TxVal32(s, Rate[i] * GyroScale[CurrAttSensorType] * 100.0, 2, ' ');
		TxString(s, "/ ");
		for (i = 0; i < 3; i++)
		TxVal32(s, Acc[i] * AccScale * 100.0, 2, ' ');
	}

	TxNextLine(s);

	InitBarometer();

	TxString(s, "Baro: ");
	OK(s, F.BaroActive);
	TxChar(s, ' ');
	TxVal32(s, ms56xx_ManufacturersData, 0, ' ');
	TxString(s, " C={");
	for (i = 0; i < 6; i++)
	TxVal32(s, ms56xx_cal[i], 0, ' ');
	TxString(s, "} ");
	TxVal32(s, BaroPressure, 2, ' ');
	TxVal32(s, RawDensityAltitude * 10.0f, 1, 'M');
	TxNextLine(s);

	TxNextLine(s);

	DoBeeps(3);
	DoBeep(8, 2);

	Timeout = mSClock();

	while (true) {
		Delay1mS(1);

		ReadFilteredGyroAndAcc(); ScaleRateAndAcc();

		F.BaroActive = true; // force
		if (uSTimeout(BaroUpdateuS)) {
			uSTimer(BaroUpdateuS, 4000);
			GetBaro();
			SIOTokenFree = true;
		}

		GetMagnetometer();
		SioTokenFree = true;

		if (mSClock() > Timeout) {
			Timeout += 500;

			TxString(s, " B: ");
			OK(s, F.BaroActive);
			TxChar(s, ' ');
			TxValH16(s, ms56xx_ManufacturersData);
			TxChar(s, ' ');
			TxVal32(s, BaroTemperature * 10.0, 1, 'C');
			TxChar(s, ' ');
			TxVal32(s, BaroPressure, 2, ' ');
			TxVal32(s, RawDensityAltitude * 10.0f, 1, ' ');

			TxString(s, "M:");
			OK(s, F.MagnetometerActive);
			TxChar(s, ' ');
			TxVal32(s, MagTemperature * 10.0f, 1, 'C');
			TxChar(s, ' ');

			/*
			 SIOReadBlock(SIOMag, HMC5XXX_ID, HMC5XXX_TAG, 3, info);
			 TxString(s, " Id = ");
			 for (k = 0; k < 3; k++)
			 TxChar(s, info[k]);
			 TxChar(s, ' ');
			 */

			for (i = 0; i < 3; i++)
			TxVal32(s, RawMag[i], 0, ' ');

			TxString(s, "IMU:");
			OK(s, F.IMUActive);
			TxVal32(s, MPU6XXXTemperature * 10.0f, 1, 'C');
			TxChar(s, ' ');
			for (i = 0; i < 3; i++)
			TxVal32(s, A[i].GyroADC * GyroScale[CurrAttSensorType] * 1000.0, 3, ' ');
			TxString(s, "/ ");
			for (i = 0; i < 3; i++)
			TxVal32(s, A[i].AccADC * AccScale * 100.0, 2, ' ');

			CheckBatteries();
			TxString(s, "V:");
			TxVal32(s, BatteryVolts * 10.0f, 1, ' ');

			TxString(s, "ADC:");
			for (i = 0; i < ANALOG_CHANNELS; i++)
			TxVal32(s, analogRead(i) * 100.0f, 2, ' ');

			/*
			 TxString(s, " i2CE:");
			 OK(s, !F.i2cFatal);
			 TxVal32(s, currStat(I2CFailS), 0, ' ');
			 TxString(s, "spiE:");
			 OK(s, !F.spiFatal);
			 TxVal32(s, currStat(SPIFailS), 0, ' ');
			 */

			TxNextLine(s);
			LEDChaser();
		}

		F.HoldingAlt = true;

	}

#else

	Timeout = 0;
	InitIMU();

	while (true) {
		Delay1mS(2);

		dT = dTUpdate(&LastInertialUpdateuS);
		dTOn2 = 0.5f * dT;
		dTR = 1.0f / dT;
		dTROn2 = dTR * 0.5f;

		ReadFilteredGyroAndAcc(); ScaleRateAndAcc();
		DoMadgwickAttitude(false);

		if (mSClock() > Timeout) {
			Timeout += 500;

			OK(s, F.IMUActive);
			TxChar(s, ',');
			TxVal32(s, MPU6XXXTemperature * 10.0f, 1, ',');
			for (i = 0; i < 3; i++) {
				TxChar(s, ',');
				TxChar(s, 'X' + i);
				TxChar(s, ',');

				TxVal32(s, RawGyro[Y] * GyroScale[CurrAttSensorType] * 1000.0, 3, ',');
				TxVal32(s, Config.GyroCal.Bias[i] * GyroScale[CurrAttSensorType] * 10000.0,
						4, ',');
				TxVal32(s, A[i].GyroADC * GyroScale[CurrAttSensorType] * 1000.0, 3, ',');
			}
			TxString(s, ",|,");
			for (i = 0; i < 3; i++) {
				TxChar(s, ',');
				TxChar(s, 'X' + i);
				TxChar(s, ',');
				TxVal32(s, RawAcc[i] * AccScale * 1000.0, 3, ',');
				TxVal32(s, Config.AccCal.Bias[i] * AccScale * 10000.0, 4, ',');
				TxVal32(s, A[i].AccADC * AccScale * 1000.0, 3, ',');

				TxVal32(s, A[i].Acc * 1000.0, 3, ',');
				TxVal32(s, A[i].Angle * 1000.0, 3, ',');
			}

			TxNextLine(s);
			LEDChaser();
		}

		F.HoldingAlt = true;
	}

#endif

} // CommissioningTest

void GPSDebug(uint8 s) {

	while (true) {
		if (F.GPSValid && F.OriginValid) {
			F.GPSValid = false;
			TxVal32(s, mSClock(), 3, ASCII_HT);
			TxVal32(s, GPSdT, 6, ASCII_HT);
			TxVal32(s, GPS.fix, 0, ASCII_HT);
			TxVal32(s, GPS.noofsats, 0, ASCII_HT);
			TxVal32(s, GPS.hAcc, 2, ASCII_HT);
			TxVal32(s, (GPS.Raw[NorthC] - GPS.OriginRaw[NorthC]) * 1000.0f,
					3, ASCII_HT);
			TxVal32(s, (GPS.Raw[EastC] - GPS.OriginRaw[EastC]) * 1000.0f,
					3, ASCII_HT);
			TxVal32(s, GPS.heading * 1000.0, 3, ASCII_HT);
			TxVal32(s, GPS.gspeed * 1000.0, 3, ASCII_HT);
			TxNextLine(s);
		}
	}

} // GPSDebug

#define IM 139968
#define IA 3877
#define IC 29573

void SphereFitTest(void) {
	uint16 OK;

	const real32 x0 = 0.0f;
	const real32 y0 = 0.1f;
	const real32 z0 = 0.2f;
	real32 radius;
	timeuS StartTime;
	real32 r, u, v, A, d[1000][3], O[3], R;
	uint16 Population[2][3];
	uint16 c, i;

	//real32 w, sphere_x, sphere_y, sphere_z, sphere_radius;

	for (i = 0; i < 1000; i++) {
		// http://mathworld.wolfram.com/SpherePointPicking.html
		// http://repository.upenn.edu/cgi/viewcontent.cgi?article=1188&context=cis_reports

		u = gen_random(42, 1.0f);
		v = gen_random(67, 2.0f) - 1.0f;

		A = 2.0f * PI * u;
		radius = 1.0;

		r = sqrtf(1.0f - Sqr(v));
		d[i][X] = x0 + gen_random(131, 0.2f) - 0.1f + radius * r * cosf(A);
		d[i][Y] = y0 + gen_random(153, 0.2f) - 0.1f + radius * r * sinf(A);
		d[i][Z] = z0 + gen_random(203, 0.2f) - 0.1f + radius * v;
	}

	/*
	 for (i = 0; i < 1000; i++) {
	 for (c = X; c <= Z; c++)
	 TxVal32(0, d[i][c] * 100.0, 2, ',');
	 TxNextLine(0);
	 Delay1mS(10);
	 }
	 */

	TxString(0, "Starting\r\n");

	StartTime = uSClock();
	OK = SphereFit(d, 1000, 5, 0.001f, Population, O, &R);
	TxVal32(0, uSClock() - StartTime, 0, 0);
	TxString(0, "uS\r\n");
	TxNextLine(0);

	TxVal32(0, OK, 0, ',');
	for (c = X; c <= Z; c++)
	TxVal32(0, O[c] * 10000.0f, 4, ',');
	TxVal32(0, R * 10000.0f, 4, 0);
	TxNextLine(0);

	TxString(0, "Population\r\n");
	for (c = X; c <= Z; c++) {
		TxChar(0, 'X' + c);
		TxChar(0, ',');

		for (i = 0; i < 2; i++)
		TxVal32(0, Population[i][c], 0, ',');
		TxNextLine(0);
	}
	/*
	 TxString(0, "Starting\r\n");

	 StartTime = uSClock();
	 OK = sphere_fit_least_squares(d, 1000, 5, 0.001f, &sphere_x, &sphere_y,
	 &sphere_z, &sphere_radius);
	 TxVal32(0, uSClock() - StartTime, 0, 0);
	 TxString(0, "uS\r\n");
	 TxNextLine(0);
	 TxVal32(0, OK, 0, ',');
	 TxVal32(0, sphere_x * 10000.0, 4, ',');
	 TxVal32(0, sphere_y * 10000.0, 4, ',');
	 TxVal32(0, sphere_z * 10000.0, 4, ',');
	 TxVal32(0, sphere_radius * 10000.0, 4, ',');
	 */
	while (1)
	;

} // SphereFitTest

void MagnetometerTest(uint8 s) {
	int32 a;
	MagCalStruct * M;
	uint8 info[3];
	uint8 k;

	TxString(s, "\r\nMagnetometer Test: ");
	OK(s, F.MagnetometerActive);
	if (!F.MagnetometerCalibrated)
	TxString(s, " NOT CALIBRATED");

	SIOReadBlock(SIOMag, HMC5XXX_ID, HMC5XXX_TAG, 3, info);
	TxString(s, "\r\nID: ");
	for (k = 0; k < 3; k++)
	TxChar(s, info[k]);

	if (F.MagnetometerActive) {

		F.NewMagValues = false;
		while (!F.NewMagValues) {
			GetMagnetometer();
			SIOTokenFree = true;
		}

		TxString(s, "\r\nTemperature: ");
		TxVal32(s, MagTemperature * 10.0f, 1, 'C');
		TxString(s, "\r\nMagnitude: ");
		TxVal32(s, Config.MagCal.Magnitude * 10.0f, 1, 0);
		TxString(s,
				"\r\n\tRaw\tMag \tNeg \tPos \tBias \tScale \tBias2 \tScale2\r\n");

		for (a = BF; a <= UD; a++) {
			M = &Config.MagCal;
			TxChar(s, ASCII_HT);
			TxVal32(s, RawMag[a] * 10.0f, 1, ASCII_HT);
			TxVal32(s, Mag[a] * 10.0f, 1, ASCII_HT);
			TxVal32(s, M->Population[0][a], 0, ASCII_HT);
			TxVal32(s, M->Population[1][a], 0, ASCII_HT);
			TxVal32(s, M->Bias[a] * 10.0f, 1, ASCII_HT);
			TxVal32(s, M->Scale[a] * 1000.0f, 3, ASCII_HT);
			TxVal32(s, M->Bias2[a] * 10.0f, 1, ASCII_HT);
			TxVal32(s, M->Scale2[a] * 1000.0f, 3, ASCII_HT);
			TxNextLine(s);
		}
	} else
	TxString(s, "\r\nFAIL\r\n");

	TxString(s, "\r\nMag Var (deg.):");

	TxString(s, "\r\n\tUsing   ");
	TxVal32(s, RadiansToDegrees(MagVariation) * 100.0f, 2, 0);

	TxString(s, "\t (WMM2010 ");
	MagVariationWMM2010 = ComputeMagVar();

	if (GPS.year != 0) {
		TxVal32(s, RadiansToDegrees(MagVariationWMM2010) * 100.0f, 2, '@');
		TxVal32(s, GPS.day, 0, '/');
		TxVal32(s, GPS.month, 0, '/');
		TxVal32(s, GPS.year, 0, ' ');
	} else
	TxString(s, " - no GPS yet!");
	TxString(s, ")\r\n");

} // MagnetometerTest

void BaroTest(uint8 s) {
	index i;
	TxString(s, "\r\nAltitude test\r\n");

	TxVal32(s, ms56xx_ManufacturersData, 0, ' ');
	TxString(s, " ms56xx[");
	for (i = 1; i < 8; i++)
	TxVal32(s, ms56xx_c[i], 0, ',');
	TxString(s, "]/r/n");

	if (F.BaroActive) {
		F.NewBaroValue = false;
		while (!F.NewBaroValue) {
			GetBaro();
			SIOTokenFree = true;
		}

		TxString(s, "\r\nBaro. Temp.: ");
		TxVal32(s, (int32) BaroTemperature * 100, 2, ' ');
		TxString(s, "C\r\n");
		TxString(s, "Pressure: ");
		TxVal32(s, BaroPressure, 2, 0);
		TxString(s, " hPa\r\n");

		TxString(s, "Origin Alt.: ");
		TxVal32(s, OriginAltitude * 100.0f, 2, ' ');
		TxString(s, "M\r\n");

		TxString(s, "Abs.Density Alt.: ");
		TxVal32(s, RawDensityAltitude * 100.0f, 2, ' ');
		TxString(s, "M\r\n");

		TxString(s, "ROC: ");
		TxVal32(s, ROC * 100.0f, 2, 0);
		TxString(s, " M/S\r\n");

		GetRangefinderAltitude();
		TxString(s, "\r\nR.Finder: ");
		if (!F.RangefinderActive)
		TxString(s, "Inactive ");
		TxVal32(s, (int32) RangefinderAltitude * 100.0f, 2, ' ');
		TxString(s, "M\r\n");
	} else
	TxString(s, "\r\n Baro Inactive ");

} // BaroTest

void BatteryTest(uint8 s) {

	TxString(s, "\r\nBattery test\r\n");

	// Battery
	CheckBatteries();
	TxString(s, "Battery:\t");
	TxVal32(s, BatteryVolts * 10.0f, 1, 'V');
	TxString(s, " Limit is ");
	TxVal32(s, BatteryVoltsLimit * 10.0f, 1, 'V');
	if (F.LowBatt)
	TxString(s, " LOW");
	TxNextLine(s);

} // BatteryTest

void LEDsAndBuzzer(uint8 s) {
	int8 f, m;

	TxString(s, "\r\nLED/Beeper test\r\n");
	for (m = 0; m < MAX_LEDS; m++) {
		LEDOff(m);
		for (f = 0; f < 10; f++) { // 10 FLASHes (count MUST be even!)
			LEDToggle(m);
			Delay1mS(100);
		}
	};

	BeeperOn();
	Delay1mS(100);
	BeeperOff();
	TxString(TelemetrySerial, "Test Finished\r\n");
} // LEDsAndBuzzer

void InertialTest(uint8 s) {
	real32 AccMag;
	int32 a;

	TxString(s, "\r\nInertial test\r\n");

	ReadAccAndGyro();

	TxString(s, "\r\nSIO: 0x");
	TxValH(s, MPU_ID);
	TxString(s, " ID: 0x");
	TxValH(s, MPU6XXXId);
	TxString(s, " Rev: 0x");
	TxValH(s, MPU6XXXRev);
	UpdateAccAndGyroBias();

	TxString(s, "\r\nIC Temp: ");
	TxVal32(s, MPU6XXXTemperature * 10.0f, 1, 0);
	TxChar(s, 'C');

	TxString(s, "\r\nHPF: ");
	TxString(s, DHPFName[MPU6XXXDHPF]);

	TxString(s, " LPF: ");
	TxVal32(s, MPULPFHz[MPU6XXXDLPF], 0, 0);
	TxString(s, "Hz");

	TxString(s, " AccLPF: (MPU6500) ");
	TxVal32(s, MPUAccLPFHz[MPU6000DLPF], 0, 0);
	TxString(s, "Hz");

	ErectGyros(20);

	ReadFilteredGyroAndAcc(); ScaleRateAndAcc();

	TxString(s, "\r\nMPU6XXX Acc. Cal. @ ");
	TxVal32(s, (Config.AccCal.TRef) * 10.0f, 1, 0);
	TxChar(s, 'C');
	if (!F.IMUCalibrated)
	TxString(s, " NOT CALIBRATED");
	TxString(s, "\r\n\t\tAcc\tComp\tBias\t/C\r\n");
	for (a = X; a <= Z; a++) {
		TxChar(s, ASCII_HT);
		TxChar(s, 'X' + a);
		TxChar(s, ASCII_HT);
		TxVal32(s, RawAcc[a], 0, ASCII_HT);
		TxVal32(s, Config.AccCal.Bias[a] * 10.0f, 1, ASCII_HT);
		TxVal32(s, Config.AccCal.Scale[a] * 1000.0f, 3, 0);
		TxNextLine(s);
	}

	TxString(s, "\r\n");
	ShowAttSensorType(s, CurrAttSensorType);
	TxString(s, " Gyro Cal. @ ");
	TxVal32(s, (Config.GyroCal.TRef) * 10.0f, 1, 0);
	TxString(s, "C\r\n\t\tRate\tComp\tBias\t/C\r\n");
	for (a = X; a <= Z; a++) {
		TxChar(s, ASCII_HT);
		TxChar(s, 'X' + a);
		TxChar(s, ASCII_HT);
		TxVal32(s, RawGyro[a], 0, ASCII_HT);
		TxVal32(s, GyroBias[a] * 10.0f, 1, ASCII_HT);
		TxVal32(s, Config.GyroCal.Bias[a] * 10.0f, 1, ASCII_HT);
		TxVal32(s, Config.GyroCal.TemperatureGradient[a] * 1000.0f, 3, 0);
		TxNextLine(s);
	}

	if (F.IMUCalibrated) {

		TxString(s, "\r\nRaw Acc. UD: ");
		TxVal32(s, A[UD].AccADC, 0, 0);
		TxString(s, " (Exp. ");
		TxVal32(s, -1.0f / AccScale, 0, 0);
		TxChar(s, ')');

		TxString(s, "\r\nAcc. Mag.: ");
		AccMag = sqrtf(Sqr(A[Roll].Acc) + Sqr(A[Pitch].Acc) + Sqr(A[Yaw].Acc));
		TxVal32(s, AccMag * 1000.0, 3, 0);
		TxString(s, " (should be ~1.0)\r\n");

		TxString(s,
				"\r\nRates and Accs.: (should be ~0.0 except for U->D ~-1.0)");
		TxString(s, "\r\n\tPitch: \t");
		TxVal32(s, Rate[Pitch] * 1000.0f, 3, 0);
		TxString(s, "\tB->F: \t");
		TxVal32(s, Acc[BF] * 1000.0f, 3, 0);

		TxString(s, "\r\n\tRoll:  \t");
		TxVal32(s, Rate[Roll] * 1000.0f, 3, 0);
		TxString(s, "\tL->R: \t");
		TxVal32(s, Acc[LR] * 1000.0f, 3, 0);

		TxString(s, "\r\n\tYaw:   \t");
		TxVal32(s, Rate[Yaw] * 1000.0f, 3, 0);
		TxString(s, "\tU->D:    \t");
		TxVal32(s, Acc[UD] * 1000.0f, 3, 0);
		TxNextLine(s);
	} else
	TxString(s, "ACCELEROMETERS NOT CALIBRATED");

} // InertialTest

void ReceiverTest(uint8 s) {
	uint8 c;

	TxString(s, "\r\nRx: ");
	ShowRCSetup(s);

	if (F.HaveSerialRC) {
		TxString(s, "\r\nLost Frames: ");
		TxVal32(s, LostFrameCount, 0, ' ');
		TxString(s, " (Spektrum/SBus)");
	}

	TxString(s, "\r\nChannel order is: ");
	for (c = 0; c < RC_MAX_CHANNELS; c++)
	TxChar(s, RxChMnem[RMap[c]]);

	if (F.Signal)
	TxString(s, "\r\nSignal OK\r\n");
	else
	TxString(s, "\r\nSignal FAIL\r\n");

	if (F.HaveSerialRC) {
		if (currRxType == Deltang) {
			TxString(s, "\r\nRSSI: ");
			TxVal32(s, RSSI, 0, ' ');
		} else {
			TxString(s, "\r\nPacket Loss: ");
			TxVal32(s, LostFrameCount, 0, ' ');
		}
	}
	// Be wary as new RC frames are being received as this
	// is being displayed so data may be from overlapping frames
	for (c = 0; c < RC_MAX_CHANNELS; c++) {
		//zzzTxChar(s, c + '1');
		//TxString(s, ": ");
		TxChar(s, RxChMnem[RMap[c]]);
		TxString(s, ": \t");
		if ((currRxType == Spektrum1024_M7to10) || (currRxType == Spektrum2048_M7to10)
				|| (currRxType == Deltang1024_M7to10) )
		TxVal32(s, RCInp[c].SpekRaw, 3, ' ');
		TxVal32(s, RCInp[c].Raw, 3, ' ');
		TxString(s, " \t");
		TxVal32(s, RC[RMap[c]] * 100.0, 0, '%');
		TxNextLine(s);
	}

	TxString(s, "Glitches: \t");
	TxVal32(s, RCGlitches, 0, 0);
	TxNextLine(s);

	if (currRxType == rxCPPM) {
		TxString(s, "Sync Period: \t");
		TxVal32(s, RCSyncWidthuS, 3, 0);
		TxString(s, "mS\r\n");
	}

	TxString(s, "Frame Interval: \t");
	TxVal32(s, RCFrameIntervaluS, 3, 0);
	TxString(s, "mS\r\n");

} // ReceiverTest

void ShowStat(uint8 s) {
	int32 a;

	TxString(s, "\r\nFlight Stats\r\n");

	if (currStat(BadS) != 0) {
		TxString(s, "Misc(gke):     \t");
		TxVal32(s, (int32) currStat(BadS), 0, ' ');
		TxVal32(s, (int32) currStat(BadNumS), 0, ' ');
		TxNextLine(s);
	}

	TxString(s, "\r\n\r\nSensor/Rx Fails (Count)\r\n");
	TxString(s, "I2CBus:\t");
	TxVal32(s, (int32) currStat(I2CFailS), 0, 0);
	TxNextLine(s);
	TxString(s, "SPIBus:\t");
	TxVal32(s, (int32) currStat(SPIFailS), 0, 0);
	TxNextLine(s);
	TxString(s, "GPS:   \t");
	TxVal32(s, (int32) currStat(GPSInvalidS), 0, 0);
	TxNextLine(s);
	TxString(s, "Acc:   \t");
	TxVal32(s, (int32) currStat(AccFailS), 0, 0);
	TxNextLine(s);
	TxString(s, "Gyro:  \t");
	TxVal32(s, (int32) currStat(GyroFailS), 0, 0);
	TxNextLine(s);
	TxString(s, "Comp:  \t");
	TxVal32(s, (int32) currStat(CompassFailS), 0, 0);
	TxNextLine(s);
	TxString(s, "Baro:  \t");
	TxVal32(s, (int32) currStat(BaroFailS), 0, 0);
	TxNextLine(s);
	TxString(s, "Rx:       \t");
	TxVal32(s, (int32) currStat(RCGlitchesS), 0, ' ');
	TxNextLine(s);
	TxString(s, "Failsafes:\t");
	TxVal32(s, (int32) currStat(RCFailsafesS), 0, ' ');
	TxNextLine(s);

	TxString(s, "\r\nBaro\r\n");
	TxString(s, "Alt:      \t");
	TxVal32(s, (int32) currStat(AltitudeS), 0, ' ');
	TxString(s, "M \r\n");
	TxString(s, "ROC:      \t");
	TxVal32(s, (int32) currStat(MinROCS), 2, ' ');
	TxVal32(s, (int32) currStat(MaxROCS), 2, ' ');
	TxString(s, "M/S\r\n");

#if defined(INC_TEMPERATURE)
	if ( currStat(MinTempS) < INIT_MIN )
	{
		TxString(s, "Ambient:  \t");
		TxVal32(s, (int32)currStat(MinTempS), 1, ' ');
		TxVal32(s, (int32)currStat(MaxTempS), 1, ' ');
		TxString(s, "C\r\n");
	}
#else
	TxString(s, "Ambient:  \tunknown\r\n");
#endif // INC_TEMPERATURE
	TxString(s, "\r\nGPS\r\n");
	TxString(s, "Alt:      \t");
	TxVal32(s, (int32) currStat(GPSAltitudeS), 0, ' ');
	TxString(s, "M\r\n");
	TxString(s, "Vel:      \t");
	TxVal32(s, currStat(GPSVelS), 1, ' ');
	TxString(s, "M/S\r\n");

	if (currStat(GPSMinSatsS) < INIT_MIN) {
		TxString(s, "Sats:     \t");
		TxVal32(s, (int32) currStat(GPSMinSatsS), 0, ' ');
		TxVal32(s, (int32) currStat(GPSMaxSatsS), 0, 0);
		TxNextLine(s);
	}

	if (currStat(MinhAccS) < INIT_MIN) {
		TxString(s, "DOP:  \t");
		TxVal32(s, (int32) currStat(MinhAccS), 2, ' ');
		TxVal32(s, (int32) currStat(MaxhAccS), 2, 0);
		TxNextLine(s);
	}

	if (currStat(OriginValidS))
	TxString(s, "Nav ENABLED\r\n");
	else
	TxString(s, "Nav DISABLED (No fix at launch)\r\n");

} // ShowStats

#endif

