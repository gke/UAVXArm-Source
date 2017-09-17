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
uint8 CurrPIDTimeSel;
uint32 CurrPIDCycleuS = PID_CYCLE_2000US;
real32 CurrPIDCycleS;
volatile uint32 uS[uSLastArrayEntry];
volatile uint32 mS[mSLastArrayEntry];

uint8 ch;
int8 i, m;

void InitMisc(void) {
	idx i;

	State = Preflight;

	for (i = 0; i < FLAG_BYTES; i++)
		F.AllFlags[i] = false;

	for (i = 0; i < mSLastArrayEntry; i++)
		mS[i] = 0;

	for (i = 0; i < uSLastArrayEntry; i++)
		uS[i] = 0;

	InitialThrottle = ThrNeutral = ThrLow = ThrHigh = 1.0f;
	IdleThrottle = FromPercent(10);

} // InitMisc

void CalculatedT(uint32 NowuS) {

	dT = dTUpdate(NowuS, &LastInertialUpdateuS);
	dTOn2 = 0.5f * dT;
	dTR = 1.0f / dT;
	dTROn2 = dTR * 0.5f;

} // CalculatedT

void ResetMainTimeouts(void) {
	uint32 NowmS;

	NowmS = mSClock();
	mSTimer(NowmS, ArmedTimeout, ARMED_TIMEOUT_MS);
	mSTimer(NowmS, RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);

} // ResetMainTimeouts


int main() {
	uint32 NowuS;
	static uint32 LastUpdateuS = 0;

	CheckParametersInitialised();

	InitClocks();

	InitHarness();

	InitMisc();
	InitAnalog();
	InitLEDs();

	if (sizeof(NV) >= NV_FLASH_SIZE)
		Catastrophe();

	InitExtMem();

#if defined(COMMISSIONING_TEST)

	ReadBlockNV(0, sizeof(NV), (int8 *) (&NV));

	NV.CurrPS = 0;
	for (i = 0; i < MAX_PARAMETERS; i++)
	SetP(DefaultParams[i].tag, DefaultParams[i].p[0]);

	CommissioningTest(0);

#else

#if defined(V4_BOARD)
	for (i = 0; i < 4; i++)
	spiSelect(i, false); // TODO do it again but why is this being changed?
#endif

	InitParameters();

	if ((P(Config2Bits) & UseBLHeliMask) != 0)
		DoBLHeliSuite(TelemetrySerial);
	else
		Delay1mS(1000); // let things settle!

	InitIMU(); // connects pass through for mag
	InitMagnetometer();
	InitMadgwick();

	InitBarometer();

	InitControl();

	InitEmulation();
	InitNavigation();
	InitTemperature();

	InitPollRxPacket();

	if (CurrComboPort1Config != ParallelPPM)
		RxEnabled[RCSerial] = true;

	FirstPass = true;
	mSTimer(mSClock(), LastBattery, 0);
	uSTimer(uSClock(), NextCycleUpdate, CurrPIDCycleuS);

	State = Preflight;

	//#define TEST_MAVLINK
#if defined(TEST_MAVLINK)
	//zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
	SetTelemetryBaudRate(0, 57600);
	while (true) {
		mavlinkUpdate(0);
		UpdateControls(); // avoid loop sync delay
	}
#endif

	while (true) {

		if ((UAVXAirframe == Instrumentation) || (UAVXAirframe == IREmulation)) {
			F.Signal = true;
			StickThrottle = 0.0f;
			RCStart = 0;
			//F.ThrottleOpen = F.Navigate = F.ReturnHome = false;
		} else {
			CheckRxLoopback();
			UpdateControls(); // avoid loop sync delay
		}

		NowuS = uSClock();
		if (NowuS >= uS[NextCycleUpdate]) {

			Probe(1);

			//---------------
			CalculatedT(NowuS);

			UpdateDrives(); // from previous cycle - one cycle lag

			UpdateInertial();
			//---------------

			uSTimer(NowuS, NextCycleUpdate, CurrPIDCycleuS);

			uSTimer(NowuS, LastCycleTime, -LastUpdateuS);
			LastUpdateuS = NowuS;

			// Housekeeping here


			switch (State) {
			case Preflight:

				F.HoldingAlt = F.IsArmed = false;
				DesiredThrottle = 0.0f;
				ZeroThrottleCompensation();

				StopDrives();

				if (FailPreflight()) {
					LEDOn(LEDRedSel);
					LEDOff(LEDGreenSel);

				} else {

					LEDOff(LEDRedSel);
					LEDOn(LEDGreenSel);

					DoBeep(8, 2);

					FirstPass = F.OriginValid = F.NavigationEnabled = false;
					AlarmState = NoAlarms;
					InitialThrottle = StickThrottle;

					State = Ready;
				}

				break;
			case Ready:
				if (Armed() || (UAVXAirframe == Instrumentation)) {
					LEDOn(LEDYellowSel);
					RxLoopbackEnabled = false;
					State = Starting;
				} else
					DoStickProgramming();

				break;
			case Starting:

				DoBeep(8, 2);

				if (GPSRxSerial == TelemetrySerial)
					InitGPS();

				DoBeep(8, 2);
				InitBlackBox();

				InitControl();
				InitNavigation();

				if (F.UsingAnalogGyros || !UsingFastStart)
					ErectGyros(5);

				ZeroStats();
				F.IsArmed = true;
				mSTimer(mSClock(), WarmupTimeout, WARMUP_TIMEOUT_MS);

				State = Warmup;

				break;
			case Warmup:

				BatteryCurrentADCZero
						= SoftFilter(BatteryCurrentADCZero, analogRead(BattCurrentAnalogSel));

				if (mSClock() > mS[WarmupTimeout]) {
					UbxSaveConfig(GPSTxSerial); //does this save ephemeris stuff?

					DoBeeps(3);
					DoBeep(8, 2);

					ResetMainTimeouts();

					LEDOff(LEDYellowSel);

					State = (UAVXAirframe == IREmulation) ? IREmulate : Landed;
				}
				break;
			case Landed:
				ZeroThrottleCompensation();
				ZeroNavCorrections();

				if (Armed() || (UAVXAirframe == Instrumentation)) {
					F.DrivesArmed = CurrESCType == DCMotorsWithIdle;
					DesiredThrottle = F.DrivesArmed ? IdleThrottle : 0.0f;

					ZeroPIDIntegrals();

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
							InitiateShutdown(PIC);
						else {
							if (F.GPSToLaunchRequired)
								CaptureHomePosition();

							if ((StickThrottle >= IdleThrottle)
									&& (F.OriginValid || !F.GPSToLaunchRequired)) {

								ResetMainTimeouts();
								setStat(RCGlitchesS, 0);
								RateEnergySum = 0.0f;
								RateEnergySamples = 0;

								UbxSaveConfig(GPSTxSerial);
								UpdateNV(); // also captures stick programming

								LEDsOff();

								F.DrivesArmed = true;
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
						LEDOn(LEDGreenSel);
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
			case InFlight:

				LEDChaser();

				DoNavigation();

				if (UAVXAirframe == Instrumentation) {
					if (F.NavigationEnabled)
						F.NewNavUpdate = false;
				} else {

					// no exit for fixed wing when using roll/yaw stick arming

					if ((StickThrottle < IdleThrottle) && (IsMulticopter
							|| (((ArmingMethod == SwitchArming)
									|| (ArmingMethod == TxSwitchArming))
									&& !Armed()))) {
						ZeroThrottleCompensation();
						mSTimer(mSClock(), ThrottleIdleTimeout,
								THR_LOW_DELAY_MS);
						State = Landing;
					} else {
						if (UpsideDownMulticopter())
							InitiateShutdown(Touchdown);
						else {
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

			Probe(0);
		} // if next cycle

		CheckBatteries();
		CheckAlarms();
		DoCalibrationAlarm();

		CheckTelemetry(TelemetrySerial);
		UpdatewsLed();

	} // while true

#endif // COMMISSIONING_TEST
	return (0);

} // loop


