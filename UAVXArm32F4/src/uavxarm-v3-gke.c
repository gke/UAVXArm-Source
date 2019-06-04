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
timeuS FlightTimemS = 0;

timeuS CurrPIDCycleuS = PID_CYCLE_2000US;
real32 CurrPIDCycleS;

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

	DesiredThrottle = StickThrottle = 0.0f;
	InitialThrottle = ThrNeutral = ThrLow = ThrHigh = 1.0f;
	IdleThrottle = FromPercent(10);

} // InitMisc


void DoHouseKeeping(void) {

	CheckLandingSwitch();
	CheckBatteries();
	CheckAlarms();
	DoCalibrationAlarm();

	if (UsingWS28XXLEDs)
		UpdateWSLEDBuffer();
#if !defined(DEBUG_BARO) && !defined(DEBUG_BARO_2)
	CheckTelemetry(TelemetrySerial);
#endif

} // DoHousekeeping

void ResetMainTimeouts(void) {

	mSTimer(CrashedTimeout, CRASHED_TIMEOUT_MS);
	mSTimer(ArmedTimeout, ARMED_TIMEOUT_MS);
	mSTimer(RxFailsafeTimeout, RC_NO_CHANGE_TIMEOUT_MS);

} // ResetMainTimeouts


int main() {
	static timeuS LastUpdateuS = 0;

	InitClocks();
	Delay1mS(1000);

	InitMisc();

	LoadParameters();
	InitHarness();

	SendDefAFNameNames(TelemetrySerial);

	InitLEDs();
	InitWSLEDs();
	LEDOn(ledYellowSel);

	InitPollRxPacket();
	SPIClearSelects();

	CheckBLHeli();

	InitIMU(imuSel); // 0.504mS
	InitMagnetometer(); // 1574mS
	InitMadgwick(); // 0.00353mS

	InitAltitudeFilters();
	InitBaro(); // 1032mS

	InitNVMem();

	InitTemperature(); // 0.0003mS

	InitEmulation();

	DoTesting(); // if any - does not exit
	InitControl();

	InitNavigation();

	LEDsOff();
	if ((GPSRxSerial != TelemetrySerial) && (CurrGPSType != NoGPS))
		InitGPS(); // 11600mS !!!!!!

	F.DrivesArmed = false;
	EnableRC();

	uSTimer(NextCycleUpdate, CurrPIDCycleuS);

	FirstPass = true;
	State = Preflight;

	while (true) {

		tickCountsEnabled = State == InFlight;
		tickCountOn(FlightTick);

		if ((UAVXAirframe == Instrumentation) || (UAVXAirframe == IREmulation)) {
			F.Signal = true;
			StickThrottle = 0.0f;
			RCStart = 0;
			//F.ThrottleOpen = F.Navigate = F.ReturnHome = false;
		} else {
			CheckRCLoopback();
			UpdateControls(); // avoid loop sync delay
		}

		if (uSTimeout(NextCycleUpdate)) {
			uSTimer(NextCycleUpdate, CurrPIDCycleuS);

			//Marker();

			dT = dTUpdate(&LastUpdateuS);
			dTOn2 = 0.5f * dT;
			dTR = 1.0f / dT;
			dTROn2 = dTR * 0.5f;

			UpdateDrives(); // from previous cycle - one cycle lag minimise jitter
			UpdateInertial();

			DoHouseKeeping();

			switch (State) {
			case Preflight:
				F.IsArmed = false;

				DisableFlightStuff();

				if (FailPreflight()) {
					LEDOn(ledRedSel);
					LEDOff(ledGreenSel);
				} else {

					LEDOff(ledRedSel);
					LEDOn(ledGreenSel);

					DoBeep(8, 2);

					FirstPass = F.OriginValid = F.NavigationEnabled
							= F.OffsetOriginValid = false;
					AlarmState = NoAlarms;
					InitialThrottle = StickThrottle;

					State = Ready;
				}

				break;
			case Ready:

				DisableFlightStuff();

				if (Armed() || (UAVXAirframe == Instrumentation)) {
					LEDOn(ledYellowSel);
					RxLoopbackEnabled = false;
					State = Starting;
				} else
					DoStickProgramming();

				break;
			case Starting:

				DisableFlightStuff();

				DoBeep(8, 2);

				if (GPSRxSerial == TelemetrySerial)
					InitGPS();

				DoBeep(8, 2);
				InitBlackBox();

				InitControl();
				InitNavigation();

				if (F.UsingAnalogGyros || !UsingFastStart)
					ErectGyros(imuSel, 2); // was 5

				ZeroStats();

				mSTimer(WarmupTimeout, WARMUP_TIMEOUT_MS);

				State = Warmup;
				break;
			case Warmup:

				DisableFlightStuff();

				CaptureBatteryCurrentADCZero();

				if (mSTimeout(WarmupTimeout)) {
					UbxSaveConfig(GPSTxSerial); //does this save ephemeris stuff?

					DoBeeps(3);
					DoBeep(8, 2);

					ResetMainTimeouts();

					LEDOff(ledYellowSel);

					State = ThrottleOpenCheck;
				}
				break;
			case ThrottleOpenCheck:

				DisableFlightStuff();

				if (F.ThrottleOpen) {
					LEDsOff();
					LEDToggle(ledRedSel);
					//LEDOff(ledGreenSel);
					AlarmState = CloseThrottle;
				} else {
					LEDOff(ledRedSel);
					LEDOn(ledGreenSel);
					AlarmState = NoAlarms;
					State = Landed;
				}
				break;
			case Landed:

				DisableFlightStuff();
				TrackOriginAltitude();
				F.DrivesArmed = false;

				if (UAVXAirframe == Instrumentation) {

					DoStickProgramming();
					CaptureHomePosition();
					CaptureBatteryCurrentADCZero();

					if (F.OriginValid) { // for now only works with GPS
						LEDsOff();
						UbxSaveConfig(GPSTxSerial);
						State = InFlight;
					}

				} else {
					if (Armed()) {

						if (CurrESCType == DCMotorsWithIdle) {
							F.DrivesArmed = true;
							DesiredThrottle = IdleThrottle;
						} else
							CaptureBatteryCurrentADCZero();

						DoStickProgramming();

						if (mSTimeout(ArmedTimeout))
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

							//	UbxSaveConfig(GPSTxSerial);
								UpdateConfig(); // also captures stick programming

								F.DrivesArmed = true;

								mS[StartTime] = mSClock();

								State = InFlight;

							}
						}
					} else
						// became disarmed mid state change
						State = Preflight;
				}
				break;
			case Landing:
				if (StickThrottle > IdleThrottle) {

					DesiredThrottle = 0.0f;
					ResetMainTimeouts();

					mS[StartTime] = mSClock();
					State = InFlight;

				} else {
					if (mSTimeout(ThrottleIdleTimeout)) {

						DesiredThrottle = 0.0f;

						if (NavState != Perching)
							F.OriginValid = F.OffsetOriginValid = false;

						if (Tuning) {
							// TODO: save tuning?
						}

						SetP(KFBaroVar, Limit(BaroVariance, 0.05f, 1.0f) * 100.0f);
						SetP(TrackKFAccZVar, Limit(TrackAccZVariance, 0.01f, 2.0f) * 100.0f);

						UpdateConfig(); // also captures stick programming

						ResetMainTimeouts();

						mSTimer(ThrottleIdleTimeout, THR_LOW_DELAY_MS);
						LEDOn(ledGreenSel);

						F.DrivesArmed = false;

						State = Landed;
					} else
						DesiredThrottle = IdleThrottle;

				}
				break;
			case Shutdown:

				if (StickThrottle < IdleThrottle) {
					TxSwitchArmed = StickArmed = false;
					FirstPass = true; // should force fail preprocess with arming switch

					State = Preflight;

				} else
					LEDsOff();
				break;
			case InFlight:

				if (F.IsFixedWing && UsingOffsetHome)
					CaptureOffsetHomePosition();

				DoNavigation();

				if (UAVXAirframe == Instrumentation) {

					if (F.NavigationEnabled)
						F.NewNavUpdate = false;

				} else { // no exit for fixed wing when using roll/yaw stick arming

					CheckNavPulse(&NavPulse);

					if ((StickThrottle < RC_THRES_START_STICK) && (IsMulticopter
							|| (((ArmingMethod == SwitchArming)
									|| (ArmingMethod == TxSwitchArming))
									&& !Armed()))) {

						mSTimer(ThrottleIdleTimeout, THR_LOW_DELAY_MS);

						State = Landing;

					} else {

						DetermineInFlightThrottle();

						if (UpsideDownMulticopter())
							InitiateShutdown(UpsideDown);
						else {

							RateEnergySum
									+= Sqr(Abs(Rate[X]) + Abs(Rate[Y]) + Abs(Rate[Z]));
							RateEnergySamples++;

							DoAltitudeControl();
						}
					}
				}
				if (State != InFlight)
					tickCountOff(FlightTick);
				break;
			case IREmulate:
				if (!Armed())
					State = Preflight;
				break;
			} // switch state

			setStat(UtilisationS, State == InFlight ? ((uSClock()
					- LastUpdateuS) * 100.0f) / CurrPIDCycleuS : 0);

			//	Marker();
		}

		tickCountOff(FlightTick);

	} // while true

	return (0);

}
// loop


