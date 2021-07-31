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
timeuS execTimeuS, execPeakTimeuS;

uint8 ch;
int8 i, m;

void InitMisc(void) {
	idx i;

	for (i = 0; i < FLAG_BYTES; i++)
		F.AllFlags[i] = 0;

	for (i = 0; i < mSLastArrayEntry; i++)
		mS[i] = 0;

	for (i = 0; i < uSLastArrayEntry; i++)
		uS[i] = 0;

	memset(&GPS, 0, sizeof(GPS));
	memset(&Nav, 0, sizeof(NavStruct));
	memset(&Alt, 0, sizeof(AltStruct));

	for (i = 0; i < 3; i++) {
		memset(&A[i], 0, sizeof(AxisStruct));
		memset(&Rate[i], 0, sizeof(Rate[0]));
		memset(&IntRate[i], 0, sizeof(IntRate[0]));
		memset(&Acc[i], 0, sizeof(Acc[0]));
		memset(&Angle[i], 0, sizeof(Angle[0]));
	}

	DesiredThrottle = StickThrottle = AHThrottle = 0.0f;
	InitialThrottle = ThrNeutral = ThrLow = ThrHigh = 1.0f;
	IdleThrottle = FromPercent(10);

	execPeakTimeuS = 0;

	GyrosErected = false;
	State = Preflight;

} // InitMisc


void DoHouseKeeping(void) {

	CheckLandingSwitch();
	CheckBatteries();
	CheckAlarms();
	DoCalibrationAlarm();

	CheckTelemetry(TelemetrySerial);

	UpdateOLED();

} // DoHousekeeping

void ResetMainTimeouts(void) {

	mSTimer(CrashedTimeoutmS, CRASHED_TIMEOUT_MS);

	mSTimer(ArmedTimeoutmS, ARMED_TIMEOUT_MS);
	mSTimer(NavStateTimeoutmS, ARMED_TIMEOUT_MS);

	mSTimer(RxFailsafeTimeoutmS, RC_NO_CHANGE_TIMEOUT_MS);

} // ResetMainTimeouts

void InitiateFlight(void) {

	ResetMainTimeouts();
	RateEnergySum = 0.0f;
	RateEnergySamples = 0;

	RefreshFence();

	mS[StartTime] = mSClock();
	F.DrivesArmed = true;

	State = InFlight;
}

int main() {

	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST))
		ResetCause = GetResetCause();

	srand(0);

	static timeuS LastUpdateuS = 0;

	InitClocks();

	Delay1mS(1000);

	InitMisc();

	LoadParameters();
	InitHarness();
	InitOLED();

	InitBattery();

	SendDefAFNames(TelemetrySerial);

	InitLEDs();
	LEDOn(ledYellowSel);

	InitPollRxPacket();
	SPIClearSelects();

	CheckBLHeli();

	InitIMU(); // 0.504mS
	InitMagnetometer(); // 1574mS
	InitMadgwick(); // 0.00353mS

	InitAltitude();

	InitNVMem();

	InitTemperature(); // 0.0003mS

	InitEmulation();

	DoTesting(); // if any - does not exit
	InitControl();

	InitNavigation();

	LEDsOff();

	SendFlightPacket(TelemetrySerial);
	DoBeep(8, 2);

	InitGPS(); // 11600mS !!!!!!

	F.DrivesArmed = false;
	EnableRC();

	uSTimer(CycleUpdateuS, CurrPIDCycleuS);

	FirstPass = true;
	State = Preflight;

	while (true) {

		if (UAVXAirframe == Instrumentation) {
			F.Signal = true;
			StickThrottle = 0.0f;
			RCStart = 0;
			//F.ThrottleOpen = F.Navigate = F.ReturnHome = false;
		} else
			UpdateControls(); // avoid loop sync delay

		if (uSTimeout(CycleUpdateuS)) {
			uSTimer(CycleUpdateuS, CurrPIDCycleuS);

			//	Marker();

			dT = dTUpdate(&LastUpdateuS);
			dTOn2 = 0.5f * dT;
			dTR = 1.0f / dT;
			dTROn2 = dTR * 0.5f;

			UpdateDrives(); // from previous cycle - one cycle lag minimise jitter
			UpdateInertial();

			DoHouseKeeping();

			switch (State) {
			case Preflight:

				DisableFlightStuff();

				if (FailPreflight())
					LEDsOn();
				else {

					DoBeep(8, 2);

					FirstPass = F.OriginValid = F.NavigationEnabled
							= F.OffsetOriginValid = false;
					AlarmState = NoAlarms;
					InitialThrottle = StickThrottle;

					//	LEDsOff();
					LEDsOffExcept(ledYellowSel);

					State = Ready;
				}

				break;
			case Ready:

				DisableFlightStuff();

				if (Armed() || (UAVXAirframe == Instrumentation)) {
					LEDsOffExcept(ledYellowSel);
					State = Starting;
				}

				break;
			case Starting:

				DisableFlightStuff();

				DoBeep(8, 2);
				InitBlackBox();

				InitControl();
				InitNavigation();

				LEDsOffExcept(ledYellowSel);

				if ((UsingFastStart || GyrosErected)) {
					mSTimer(WarmupTimeoutmS, WARMUP_TIMEOUT_MS);
					State = Warmup;
				} else {
					GyrosErected = false;
					GyroErectionSamples = 0;
					State = ErectingGyros;
				}

				break;
			case ErectingGyros:

				SendFlightPacket(TelemetrySerial);

				ErectGyros(imuSel); // blue, red if moving

				if (GyrosErected) {
					mSTimer(WarmupTimeoutmS, WARMUP_TIMEOUT_MS);
					LEDsOffExcept(ledYellowSel);
					State = Warmup;
				}

				break;
			case Warmup:

				DisableFlightStuff();

				if (mSTimeout(WarmupTimeoutmS)) {

					if (F.HaveGPS)
						UbxSaveConfig(GPSSerial); //does this save ephemeris stuff?

					DoBeeps(3);
					DoBeep(8, 2);

					ResetMainTimeouts();

					if (UAVXAirframe == Instrumentation) {
						LEDsOn();
						F.DrivesArmed = false;
						DisableFlightStuff();
						State = MonitorInstruments;
					} else
						State = ThrottleOpenCheck;
				}
				break;
			case MonitorInstruments:

				if ((CurrGPSType != NoGPS) && !F.OriginValid) {
					CaptureHomePosition();
					if (F.OriginValid)
						TrackOriginAltitude();
				}

				break;
			case ThrottleOpenCheck:

				DisableFlightStuff();

				if (F.ThrottleOpen) {
					LEDsOffExcept(ledRedSel);
					AlarmState = CloseThrottle;
				} else {
					LEDsOffExcept(ledGreenSel);
					AlarmState = NoAlarms;
					State = Landed;
				}
				break;
			case Landed:

				DisableFlightStuff();
				TrackOriginAltitude();
				F.DrivesArmed = false;

				if (Armed()) {

					DoStickProgramming(); // blue toggle if trimming acc

					if ((CurrGPSType != NoGPS) && !F.OriginValid)
						CaptureHomePosition();

					if ((StickThrottle >= IdleThrottle) && ((CurrGPSType
							== NoGPS) || F.OriginValid)) {

						InitiateFlight();

					} else if (mSTimeout(ArmedTimeoutmS))
						InitiateShutdown(ArmingTimeout);

				} else { // became disarmed mid state change

					if (Tuning) {
						// TODO: save tuning?
					}

					RefreshConfig(); // stick programming, cruise throttle

					State = Preflight;
				}

				break;
			case Landing:

				if (StickThrottle > IdleThrottle) {
					DesiredThrottle = 0.0f;

					State = InFlight;

				} else {
					if (mSTimeout(ThrottleIdleTimeoutmS)) {
						DesiredThrottle = 0.0f;

						if (NavState != Perching)
							F.OriginValid = F.OffsetOriginValid = false;

						//SetP(KFBaroVar, Limit(TrackBaroVariance * 100.0f, 10, 200));
						//SetP(KFAccUVar, Limit(TrackAccUVariance * 10.0f, 10, 200));



						ResetMainTimeouts();

						State = Landed;

					} else
						DesiredThrottle = IdleThrottle;
				}
				break;
			case Shutdown:

				if (StickThrottle < IdleThrottle) {
					TxSwitchArmed = false;
					FirstPass = true; // should force fail pre-process with arming switch

					LEDsOff();
					State = Preflight;

				} else
					LEDRandom();

				break;
			case InFlight:

				if (F.IsFixedWing && F.UsingOffsetHome)
					CaptureOffsetHomePosition();

				DoNavigation();

				// no exit for fixed wing when using roll/yaw stick arming

				CheckNavPulse(&NavPulse);

				if ((StickThrottle < RC_THRES_START_STICK) && (IsMulticopter
						|| (((ArmingMethod == SwitchArming) || (ArmingMethod
								== TxArming)) && !Armed()))) {

					ResetMainTimeouts();
					mSTimer(ThrottleIdleTimeoutmS, 500); //THR_LOW_DELAY_MS);

					LEDsOffExcept(ledGreenSel);

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
						if (DisablingLEDsInFlight)
							LEDsOff();
						else {
							if (F.HoldingAlt)
								LEDChaser();
							else
								LEDsOffExcept(ledGreenSel);
						}
					}
				}

				execTimeuS = uSClock() - LastUpdateuS;
				if (execTimeuS > execPeakTimeuS)
					execPeakTimeuS = execTimeuS;

				break;
			} // switch state

			SIOTokenFree = true;

		}
	} // while true

	return (0);

}
// loop


