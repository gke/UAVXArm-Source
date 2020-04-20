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

boolean FirstPass;
boolean PreflightFail = false;
uint8 ArmingMethod;

void Probe(uint8 p) {
	DigitalWrite(&GPIOPins[ProbeSel].P, p);
} // Probe

void Marker(void) {
	Probe(1);
	Probe(0);
} // Marker

void CheckLandingSwitch(void) {
	static uint8 Count = 5;
	static boolean SwitchP = false;
	boolean Switch;

	if (F.Emulation)
		F.AccZBump = F.LandingSwitch = Altitude < 0.1f;
	else {

		Switch = DigitalRead(&GPIOPins[LandingSel].P); // active to ground

		if (Switch != SwitchP)
			if (Count == 0) {
				SwitchP = Switch;
				Count = 5;
			} else
				Count--;
		else
			Count = 5;

		F.LandingSwitch = SwitchP;
	}

} // CheckLandingSwitch


boolean Armed(void) {
	static boolean SwitchP = false;
	boolean NewUplinkState, IsArmed;

	if ((ArmingSwitch != SwitchP) && (State != InFlight)) {
		DoBeep(3, 0);
		SwitchP = ArmingSwitch;
	}

	if (ArmingMethod == SwitchArming)
		IsArmed = ArmingSwitch;
	else if (ArmingMethod == TxSwitchArming)
		IsArmed = ArmingSwitch && TxSwitchArmed;
	else
		IsArmed = ArmingSwitch && StickArmed;

	NewUplinkState = !((GPSRxSerial == TelemetrySerial) && IsArmed);
	if (F.UsingUplink != NewUplinkState) {
		RxEnabled[TelemetrySerial] = false;
		RxQNewHead[TelemetrySerial] = RxQHead[TelemetrySerial]
				= RxQTail[TelemetrySerial] = 0;
		RxEnabled[TelemetrySerial] = true;
		F.UsingUplink = NewUplinkState;
	}

	return (IsArmed);

} // Armed

boolean FailPreflight(void) {
	boolean r;

	r = !F.Signal //
			|| (RCStart > 0) //
			|| ((Armed() && FirstPass) //
					&& !((UAVXAirframe == Instrumentation) || (UAVXAirframe
							== IREmulation))) //
			|| (RC[NavModeRC] > FromPercent(20)) //
			|| F.ThrottleOpen //
			|| !F.IMUActive //
			|| !F.IMUCalibrated
			|| ((busDev[baroSel].type != noBaro)&&!F.BaroActive) //
			|| ((busDev[magSel].type != noMag) && !(F.MagnetometerActive && F.MagnetometerCalibrated))//
			|| F.LowBatt //
			|| F.spiFatal //
			|| F.i2cFatal //
			|| (F.ReturnHome || F.Navigate);

	PreflightFail = F.ReturnHome || F.Navigate || !F.Signal;

	return (r);

} // FailPreflight


void DoCalibrationAlarm(void) {
	static timeval TimeoutmS = 0;

	if (!F.IMUCalibrated || !((F.MagnetometerActive && F.MagnetometerCalibrated) || F.IsFixedWing)) {
		if (mSClock() > TimeoutmS) {
			TimeoutmS = mSClock() + 500;
			LEDToggle(ledYellowSel);
		}
	}

} // DoAccCalibrationAlarm

void DoBeep(uint8 t, uint8 d) {
	int32 i;
	timeval Timeout;

	BeeperOn();
	for (i = 0; i < (t * 100); i++) {
		Timeout = uSClock() + 500;
		do {
			GetBaro(); // hammer it to warm it up!
		} while (uSClock() < Timeout);
	}
	BeeperOff();

	for (i = 0; i < (d * 100); i++) {
		Timeout = uSClock() + 500;
		do {
			GetBaro();
		} while (uSClock() < Timeout);
	}
} // DoBeep

void DoBeeps(uint8 b) {
	idx i;

	for (i = 0; i < b; i++)
		DoBeep(2, 8);

} // DoStartingBeeps

int16 BeeperOffTime = 100;
int16 BeeperOnTime = 100;

void CheckAlarms(void) {

	F.BeeperInUse = PreflightFail || F.LowBatt || (State == Shutdown)
			|| (NavState == Descending) || (State == Launching);

	//F.BeeperInUse = false;

	if (F.BeeperInUse) {
		if (F.LowBatt) {
			BeeperOffTime = 600;
			BeeperOnTime = 600;
		} else if (State == Shutdown) {
			BeeperOffTime = 4750;
			BeeperOnTime = 250;
		} else if (State == Launching) {
			BeeperOffTime = 875;
			BeeperOnTime = 125;
		} else { //default
			BeeperOffTime = 125;
			BeeperOnTime = 125;
		}

		if (mSClock() > mS[BeeperUpdate]) {
			if (BeeperIsOn()) {
				mSTimer(mSClock(), BeeperUpdate, BeeperOffTime);
				BeeperOff();
				LEDOff(ledRedSel);
			} else {
				mSTimer(mSClock(), BeeperUpdate, BeeperOnTime);
				BeeperOn();
				LEDOn(ledRedSel);
			}
		}
	} else {
		if (mSClock() > mS[BeeperTimeout])
			BeeperOff();
	}

} // CheckAlarms


void Catastrophe(void) {

	StopDrives();
	while (true) {
		LEDsOn();
		Delay1mS(500);
		LEDsOff();
		Delay1mS(500);
	}
} // Catastrophe


boolean UpsideDownMulticopter(void) {
	static boolean UpsideDown;

	UpsideDown = false;

	if (false) { //IsMulticopter) {
		if (Abs(A[Roll].Angle) < CRASHED_ANGLE_RAD)
			mSTimer(mSClock(), CrashedTimeout, CRASHED_TIMEOUT_MS);
		else {
			if ((mSClock() > mS[CrashedTimeout]) && (DesiredThrottle
					> IdleThrottle) && F.UsingAngleControl)
				UpsideDown = true;
		}
	}

	return (UpsideDown);

} // UpsideDownMulticopter


