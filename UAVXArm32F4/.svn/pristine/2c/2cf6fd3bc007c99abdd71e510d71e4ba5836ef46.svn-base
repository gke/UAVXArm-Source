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

Flags F;
uint8 State;
boolean FirstIMU;
timeuS CurrPIDCycleuS = PID_CYCLE_2000US;
real32 CurrPIDCycleS;
volatile timeuS uS[uSLastArrayEntry];
volatile timemS mS[mSLastArrayEntry];

uint8 ch;
int8 i, m;

void InitMisc(void) {
	idx i;

	State = Preflight;

	for (i = 0; i < FLAG_BYTES; i++)
		F.AllFlags[i] = 0;

	for (i = 0; i < mSLastArrayEntry; i++)
		mS[i] = 0;

	for (i = 0; i < uSLastArrayEntry; i++)
		uS[i] = 0;

	InitialThrottle = ThrNeutral = ThrLow = ThrHigh = 1.0f;
	IdleThrottle = FromPercent(10);

} // InitMisc


void DoHouseKeeping(void) {

	CheckBatteries();
	CheckAlarms();
	DoCalibrationAlarm();

	CheckTelemetry(TelemetrySerial);
	UpdateWSLEDs();

} // DoHousekeeping

void CalculatedT(timeuS NowuS) {

	dT = dTUpdate(NowuS, &LastInertialUpdateuS);
	dTOn2 = 0.5f * dT;
	dTR = 1.0f / dT;
	dTROn2 = dTR * 0.5f;

} // CalculatedT

void ResetMainTimeouts(void) {
	timemS NowmS;

	NowmS = mSClock();
	mSTimer(mSClock(), CrashedTimeout, CRASHED_TIMEOUT_MS);
	mSTimer(NowmS, ArmedTimeout, ARMED_TIMEOUT_MS);
	mSTimer(NowmS, RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);

} // ResetMainTimeouts

void DoTesting(void) {

	//#define COMMISSIONING_TEST
	//#define USB_TESTING
	//#define KF_TESTING
	//#define BARO_TESTING


#if defined(USB_TESTING)

	USBConnect();

	LEDOn(ledGreenSel);

	USBTxString("starting USB Test (! to force restart)\n");

	while (true) {
		if (SerialAvailable(USBSerial)) {
			LEDToggle(ledYellowSel);
			uint8 ch = RxChar(USBSerial);
			if (ch == '!')
				NVIC_SystemReset();
			else
				TxChar(USBSerial, ch);
		}
	}
#elif defined(COMMISSIONING_TEST)

	ReadBlockNV(0, sizeof(NV), (int8 *) (&NV));

	NV.CurrPS = 0;
	for (i = 0; i < MAX_PARAMETERS; i++)
	SetP(DefaultParams[i].tag, DefaultParams[i].p[0]);

	CommissioningTest(0);

#elif defined(BARO_TESTING)

	int16 kkk, cycles;
	timeval start = uSClock();

	for (kkk = 0; kkk < 256; kkk++)
	LSBBaro[kkk] = 0;

	cycles = 10;
	for (kkk = 0; kkk < 3000; kkk++) {
		while (!DEBUGNewBaro)
		GetBaro();
		DEBUGNewBaro = false;

		LEDToggle(ledGreenSel);
		if (cycles-- <= 0) {
			cycles = 10;
			TxVal32(0, BaroTempVal, 0, ',');
			TxVal32(0, BaroPressVal, 0, ',');
			TxVal32(0, BaroTemperature * 1000, 3, ',');
			TxVal32(0, BaroPressure * 100, 2, ',');
			TxVal32(0, BaroRawAltitude * 1000, 3, ',');
			TxVal32(0, BaroAltitude * 1000, 3, ',');
			TxNextLine(0);
		}
		LSBBaro[BaroPressVal & 0xff]++;

	}

	TxVal32(0, (uSClock() - start) / 3000, 3, ',');
	TxNextLine(0);

	for (kkk = 0; kkk < 256; kkk++) {
		TxVal32(0, kkk, 0, ',');
		TxVal32(0, LSBBaro[kkk], 0, ',');
		TxNextLine(0);
	}

	LEDsOn();
	while (true) {
	};

#elif defined(KF_TESTING)

	const real32 qKF = 1000; // 400
	const real32 rKF = 100; // 88

	const real32 SampleHz = 8000.0f;
	const real32 NyquistHz = 4000.0f;
	const real32 PIDHz = 1000;
	const real32 GyroHz = 100.0f;

	real32 OverSampledT = 1.0f / SampleHz;
	real32 PIDdT = 1.0f/ PIDHz;

	filterStruct KalynF, FujinF, OSLPF, FinalLPF;
	int kkk;
	real32 Kalyn, Fujin, OSRC, RC, RN;

	NowuS = uSClock();
	timeval NextmS = mSClock();

	timeval PrevuS = NowuS;

	kkk = 0;

	const real32 GyroSignalHz = 50.0f;
	const real32 s2Hz = 7000.0f;
	const real32 s3Hz = 23000.0f;
	const real32 s4Hz = 33000.0f;

	real32 TimeS = 0.0f;
	real32 NextTimeS = 0.0f;

	initKalynFastLPKF(&KalynF, qKF, rKF, NyquistHz); // 0.011? 0.025
	initFujinFastLPKF(&FujinF, 300); //NyquistHz);
	initLPFn(&OSLPF, 2, NyquistHz);
	initLPFn(&FinalLPF, 1, GyroHz);

	TxString(0, "TimemS, Noise, Raw, Kalyn, Fujin, RC, RC2");
	TxNextLine(0);
	do {

		// mix signals with offsets - could add some noise
		RN = 1.0f * (real32) rand()/RAND_MAX;
		real32 v =
		sinf(TWO_PI * GyroSignalHz * TimeS)
		+ sinf(TWO_PI * s2Hz * TimeS)
		+ sinf(TWO_PI * s3Hz * TimeS) + sinf(TWO_PI * s4Hz * TimeS)
		+ RN;

		Kalyn = KalynFastLPKF(&KalynF, v, OverSampledT);
		Fujin = FujinFastLPKF(&FujinF, v, OverSampledT);
		OSRC = LPFn(&OSLPF, v, OverSampledT);

		if (TimeS > NextTimeS) {
			NextTimeS = TimeS + PIDdT;
			RC = LPFn(&FinalLPF, OSRC, GyroHz);
		}

		TxVal32(0, TimeS * 1000000, 3, ',');
		TxVal32(0, RN * 100, 2, ',');
		TxVal32(0, v * 100, 2, ',');
		TxVal32(0, Kalyn * 100, 2, ',');
		TxVal32(0, Fujin * 100, 2, ',');
		TxVal32(0, OSRC * 100, 2, ',');
		TxVal32(0, RC * 100, 2, ',');
		TxNextLine(0);

		TimeS += OverSampledT;

	}while (TimeS < 1.0f);

	LEDsOn();
	while (true) {
	};
#else
	// NO TESTS
#endif

} // DoTesting


int main() {
	timeuS NowuS;
	static timeuS LastUpdateuS = 0;

	InitClocks();
	Delay1mS(1000);

	InitMisc();

	CheckParametersInitialised();
	InitParameters();
	InitHarness();

	InitLEDs();

	InitExtMem();
	InitPollRxPacket();
	SPIClearSelects();

	//CheckBLHeli();

	InitIMU(imuSel);
	InitMagnetometer();
	InitMadgwick();
	InitBarometer();
	InitTemperature();

	InitEmulation();

	DoTesting(); // if any - does not exit

	InitControl();
	InitNavigation();

	LEDsOff();

	InitGPS();

	EnableRC();

	mSTimer(mSClock(), LastBattery, 0);
	uSTimer(uSClock(), NextCycleUpdate, CurrPIDCycleuS);

	FirstPass = true;
	State = Preflight;

	while (true) {

		if ((UAVXAirframe == Instrumentation) || (UAVXAirframe == IREmulation)) {

			F.Signal = true;
			StickThrottle = 0.0f;
			RCStart = 0;
			//F.ThrottleOpen = F.Navigate = F.ReturnHome = false;

		} else {

			CheckRCLoopback();
			UpdateControls(); // avoid loop sync delay

			if (State == Launching) // Placed here to defeat RC controls KLUDGE!
				OverrideSticks();

		}

		if (UseGyroOS) {
			FirstIMU = true;
			do {
				if (MPU6XXXReady(imuSel)) {
					//	Probe(1);
					if (FirstIMU)
						ReadFilteredGyroAndAcc(imuSel);
					else
						ReadGyro(imuSel);
					FirstIMU = false;
					//	Probe(0);
				}
				NowuS = uSClock();
			} while (NowuS < uS[NextCycleUpdate]);
		} else
			NowuS = uSClock();

		if (UseGyroOS || (NowuS >= uS[NextCycleUpdate])) {

			//Probe(1);

			UpdateDrives(); // from previous cycle - one cycle lag minimise jitter

			//---------------
			CalculatedT(NowuS);
			UpdateInertial();
			//---------------

			uSTimer(NowuS, NextCycleUpdate, CurrPIDCycleuS);

			uSTimer(NowuS, LastCycleTime, -LastUpdateuS);
			LastUpdateuS = NowuS;

			DoHouseKeeping();

			switch (State) {
			case Preflight:

				F.HoldingAlt = F.IsArmed = false;
				DesiredThrottle = 0.0f;
				ZeroThrottleCompensation();

				StopDrives();

				if (FailPreflight()) {
					LEDOn(ledRedSel);
					LEDOff(ledGreenSel);
				} else {
					LEDOff(ledRedSel);
					LEDOn(ledGreenSel);

					DoBeep(8, 2);

					FirstPass = F.OriginValid = F.NavigationEnabled = false;
					AlarmState = NoAlarms;
					InitialThrottle = StickThrottle;

					State = Ready;
				}

				break;
			case Ready:
				if (Armed() || (UAVXAirframe == Instrumentation)) {
					LEDOn(ledYellowSel);
					RxLoopbackEnabled = false;
					State = Starting;
				} else
					DoStickProgramming();

				break;
			case Starting:

				DoBeep(8, 2);

				InitGPS();

				DoBeep(8, 2);
				InitBlackBox();

				InitControl();
				InitNavigation();

				if (F.UsingAnalogGyros || !UsingFastStart)
					ErectGyros(imuSel, 5);

				ZeroStats();
				F.IsArmed = true;
				mSTimer(mSClock(), WarmupTimeout, WARMUP_TIMEOUT_MS);

				State = Warmup;

				break;
			case Warmup:

				BatteryCurrentADCZero = LPF1(BatteryCurrentADCZero, analogRead(
								BattCurrentAnalogSel), 0.5f);

				if (mSClock() > mS[WarmupTimeout]) {
					UbxSaveConfig(GPSTxSerial); //does this save ephemeris stuff?

					DoBeeps(3);
					DoBeep(8, 2);

					ResetMainTimeouts();

					LEDOff(ledYellowSel);

					State = (UAVXAirframe == IREmulation) ? IREmulate : Landed;
				}
				break;
			case Landed:
				ZeroThrottleCompensation();
				ZeroNavCorrections();

				if (Armed() || (UAVXAirframe == Instrumentation)) {
					F.DrivesArmed = CurrESCType == DCMotorsWithIdle;
					DesiredThrottle = F.DrivesArmed ? IdleThrottle : 0.0f;

					ZeroIntegrators();

					SavedHeading = DesiredHeading = Nav.TakeoffBearing
							= Nav.DesiredHeading = Heading;

					DoStickProgramming();
					F.HoldingAlt = false;

					if (UAVXAirframe == Instrumentation) {

						CaptureHomePosition();
						if (F.OriginValid) { // for now only works with GPS
							LEDsOff();
							UbxSaveConfig(GPSTxSerial);
							F.DrivesArmed = false;
							State = InFlight;
						}

					} else {

						if (mSClock() > mS[ArmedTimeout])
							InitiateShutdown(ArmingTimeout);
						else {

							if (F.GPSToLaunchRequired && !F.OriginValid)
								CaptureHomePosition();

							if ((StickThrottle >= IdleThrottle)
									&& ((CurrGPSType == NoGPS) || F.OriginValid
											|| !F.GPSToLaunchRequired)) {

								ResetMainTimeouts();
								setStat(RCGlitchesS, 0);
								RateEnergySum = 0.0f;
								RateEnergySamples = 0;

								UbxSaveConfig(GPSTxSerial);
								UpdateNV(); // also captures stick programming

								LEDsOff();

								F.DrivesArmed = true;
								if (F.IsFixedWing && !F.Bypass) {
									LaunchState = initLaunch;
									State = Launching;
								} else
									State = InFlight;

							} else {

								// continue in state "landed"

							}
						}
					}
				} else
					// became disarmed mid state change
					State = Preflight;
				break;
			case Landing:
				if (StickThrottle > IdleThrottle) {

					mS[StartTime] = mSClock();
					DesiredThrottle = 0.0f;
					ResetMainTimeouts();
					F.DrivesArmed = true;

					State = InFlight;

				} else {
					if (mSClock() < mS[ThrottleIdleTimeout])
						DesiredThrottle = IdleThrottle;
					else {

						F.DrivesArmed = CurrESCType == DCMotorsWithIdle;
						DesiredThrottle = F.DrivesArmed ? IdleThrottle : 0.0f;
						ZeroThrottleCompensation(); // to catch cycles between Rx updates

						ZeroNavCorrections();
						if (NavState != Perching)
							F.OriginValid = false;

						if (Tuning) {
							// TODO: save tuning?
						}

						UpdateNV(); // also captures stick programming

						ResetMainTimeouts();
						mSTimer(mSClock(), ThrottleIdleTimeout,
								THR_LOW_DELAY_MS);
						LEDOn(ledGreenSel);

						State = Landed;
					}
				}
				break;
			case Shutdown:
				if ((StickThrottle < IdleThrottle) && !(F.ReturnHome
						|| F.Navigate)) {
					TxSwitchArmed = StickArmed = false;
					FirstPass = true; // should force fail preprocess with arming switch

					State = Preflight;

				} else
					LEDsOff();
				break;
			case Launching:
				LaunchFW();
				if (LaunchState == finishedLaunch)

					State = InFlight;

				break;
			case InFlight:
				LEDChaser();
				DoNavigation();
				if (UAVXAirframe == Instrumentation) {

					if (F.NavigationEnabled)
						F.NewNavUpdate = false;

				} else { // no exit for fixed wing when using roll/yaw stick arming

					if ((StickThrottle < IdleThrottle) && (IsMulticopter
							|| (((ArmingMethod == SwitchArming)
									|| (ArmingMethod == TxSwitchArming))
									&& !Armed()))) {

						ZeroThrottleCompensation();
						mSTimer(mSClock(), ThrottleIdleTimeout,
								THR_LOW_DELAY_MS);

						State = Landing;

					} else {

						if (UpsideDownMulticopter()) {
							InitiateShutdown(UpsideDown);
						} else {

							RateEnergySum
									+= Sqr(Abs(Rate[X]) + Abs(Rate[Y]) + Abs(Rate[Z]));
							RateEnergySamples++;
							DFT8(RawAcc[X], DFT); // 145uS

							DoAltitudeControl();
						}
					}
				}
				break;
			case IREmulate:
				if (!Armed())
					State = Preflight;
				break;
			} // switch state

			setStat(UtilisationS, State == InFlight ? ((uSClock() - NowuS)
					* 100.0f) / CurrPIDCycleuS : 0);

			//Probe(0);

			if ((CurrTelType == UAVXFastRawIMUTelemetry) && (State == InFlight)) {

				BlackBoxEnabled = true;
				SendRawIMU(TelemetrySerial);
				BlackBoxEnabled = false;

			}
		}

	} // while true

	return (0);

} // loop


