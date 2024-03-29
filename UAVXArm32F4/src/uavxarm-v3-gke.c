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

	DesiredThrottle = StickThrottle = 0.0f;
	InitialThrottle = ThrNeutral = ThrLow = ThrHigh = 1.0f;
	IdleThrottle = FromPercent(10);

	execPeakTimeuS = 0;

	GyrosErected = false;
	State = Preflight;

} // InitMisc

void DoHouseKeeping(void) {

	if (F.Emulation)
		GPSEmulation();
	else
		CheckGPSTimeouts();

	CheckBatteries(); // for battery compensation

	CheckLandingSwitch();
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
	LEDsOffExcept(ledRedSel);

	while (true) {

		if (UAVXAirframe == Instrumentation) {
			F.Signal = true;
			RCFailsafe = RCSignalLost = false;
			StickThrottle = 0.0f;
			RCStart = 0;
			//F.ThrottleOpen = F.Navigate = F.ReturnHome = false;
		} else {
			if (SerialPorts[RCSerial].DMAUsed)
				while (SerialAvailable(RCSerial))
					RCUSARTISR(RxChar(RCSerial));
			UpdateControls(); // avoid loop sync delay
		}

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

					FirstPass = F.OriginValid = F.NavigationEnabled =
							F.OffsetOriginValid = false;
					AlarmState = NoAlarms;
					InitialThrottle = StickThrottle;

					LEDsOffExcept(ledGreenSel);

					State = Ready;
				}

				break;
			case Ready:

				DisableFlightStuff();

				if (Armed() || (UAVXAirframe == Instrumentation))
					State = Starting;

				break;
			case Starting:

				DisableFlightStuff();

				DoBeep(8, 2);
				InitBlackBox();

				InitControl();
				InitNavigation();

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

				ErectGyros(imuSel); // red if moving

				if (GyrosErected) {
					mSTimer(WarmupTimeoutmS, WARMUP_TIMEOUT_MS);
					State = Warmup;
				}

				break;

			case Warmup:

				DisableFlightStuff();

				if (mSTimeout(WarmupTimeoutmS)) {

					DoBeeps(3);
					DoBeep(8, 2);

					ResetMainTimeouts();

					if (UAVXAirframe == Instrumentation) {
						LEDsOn();
						F.DrivesArmed = false;
						DisableFlightStuff();
						State = MonitorInstruments;
					} else if (CurrGPSType == NoGPS)
						State = ThrottleOpenCheck;
					else
						State = InitialisingGPS;
				}

				break;

			case MonitorInstruments:

				// stay here - does not use Rx for now

				break;

			case InitialisingGPS:

				if (Armed()) {
					if (F.OriginValid)
						//UbxReset(GPSSerial, stopGNSS);
						State = ThrottleOpenCheck;
					else
						CaptureHomePosition();

				} else
				  State = Preflight;

				break;

			case ThrottleOpenCheck:

				DisableFlightStuff();

				if (F.ThrottleOpen) {
					LEDOff(ledGreenSel);
					LEDOn(ledRedSel);
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

				if (Armed()) {

					DoStickProgramming(); // blue toggle if trimming acc

					if (F.HaveGPS && !F.GPSValid) {
						// just wait for GPS to come back
					} else if (StickThrottle >= IdleThrottle) {
						InitiateFlight();
						State = InFlight;
					} else if (mSTimeout(ArmedTimeoutmS)) {
						InitiateShutdown(ArmingTimeout);
						State = Shutdown;
					}

				} else { // became disarmed mid state change

					if (Tuning) {
						// TODO: save tuning?
					}

					RefreshConfig(); // stick programming, cruise throttle

					State = Preflight;
				}

				break;
			case Landing:

				if ((StickThrottle + AltHoldThrComp) > IdleThrottle)
					State = InFlight;
				else {

					ZeroIntegrators();

					if (mSTimeout(ThrottleIdleTimeoutmS)) {
						DesiredThrottle = 0.0f;

						//	if (NavState != Perching)
						//		F.OriginValid = F.OffsetOriginValid = false;

						//SetP(KFBaroVar, Limit(TrackBaroVariance * 100.0f, 10, 200));
						//SetP(KFAccUVar, Limit(TrackAccUVariance * 10.0f, 10, 200));

						ResetMainTimeouts();

						LEDsOffExcept(ledGreenSel);
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

				if ((StickThrottle < RC_THRES_START_STICK)
						&& (IsMulticopter
								|| (((ArmingMethod == SwitchArming)
										|| (ArmingMethod == TxArming))
										&& !Armed()))) {

					ResetMainTimeouts();
					mSTimer(ThrottleIdleTimeoutmS, THR_LOW_DELAY_MS);

					//LEDsOffExcept(ledGreenSel);

					State = Landing;

				} else {

					DetermineInFlightThrottle();

					if (UpsideDownMulticopter()){
						InitiateShutdown(UpsideDown);
						State = Shutdown;
					} else {

						RateEnergySum += Sqr(
								Abs(Rate[X]) + Abs(Rate[Y]) + Abs(Rate[Z]));
						RateEnergySamples++;

						ControlAltitude();
						if (DisablingLEDsInFlight)
							LEDsOff();
						else {
							if (F.HoldingAlt)
								LEDChaser();
							//else
							//	LEDsOffExcept(ledGreenSel);
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

