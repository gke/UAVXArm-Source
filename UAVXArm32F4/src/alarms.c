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
	digitalWrite(&GPIOPins[ProbeSel], p);
} // Probe

void Marker(void) {
	Probe(1);
	Probe(0);
} // Marker

boolean Armed(void) {
	static boolean SwitchP = false;
	boolean NewUplinkState, IsArmed;

	if ((ArmingSwitch != SwitchP) && (State != InFlight)) {
		DoBeep(3, 0);
		SwitchP = ArmingSwitch;
	}

	IsArmed = (ArmingMethod != SwitchArming) ? ArmingSwitch && StickArmed
			: ArmingSwitch;

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
			|| F.ThrottleOpen //
			|| !F.IMUActive //
			|| !F.BaroActive //
			|| !F.MagnetometerActive //
			|| !F.IMUCalibrated //
			|| !F.MagnetometerCalibrated //
			|| (RC[RTHRC] > FromPercent(20)) //
			|| F.LowBatt //
			|| F.spiFatal //
			|| F.i2cFatal //
			|| (F.ReturnHome || F.Navigate);

	PreflightFail = F.ThrottleOpen || F.ReturnHome || F.Navigate || !F.Signal;

	return (r);

} // FailPreflight


void DoCalibrationAlarm(void) {
	static uint32 TimeoutmS = 0;

	if (!(F.IMUCalibrated || F.MagnetometerCalibrated)) {
		if (mSClock() > TimeoutmS) {
			TimeoutmS = mSClock() + 500;
			LEDToggle(LEDYellowSel);
		}
	}

} // DoAccCalibrationAlarm

void DoBeep(uint8 t, uint8 d) {
	int32 i;

	if (UsingFastStart)
		d /= 2;

	BeeperOn();
	for (i = 0; i < (t * 100); i++) {
		Probe(1);
		Delay1mS(1);
		GetBaro(); // hammer it to warm it up!
		Probe(0);
	}
	BeeperOff();
	for (i = 0; i < (d * 100); i++) {
		Probe(1);
		Delay1mS(1);
		GetBaro();
		Probe(0);
	}
} // DoBeep

void DoBeeps(uint8 b) {
	idx i;

	for (i = 0; i < b; i++)
		if (UsingFastStart)
			DoBeep(2, 4);
		else
			DoBeep(2, 8);

} // DoStartingBeeps

int16 BeeperOffTime = 100;
int16 BeeperOnTime = 100;

void CheckAlarms(void) {

	F.BeeperInUse = PreflightFail || F.LowBatt || F.LostModel || (State
			== Shutdown) || (NavState == Descending);

	if (F.BeeperInUse) {
		if (F.LowBatt) {
			BeeperOffTime = 600;
			BeeperOnTime = 600;
		} else if (State == Shutdown) {
			BeeperOffTime = 4750;
			BeeperOnTime = 250;
		} else { // default
			BeeperOffTime = 125;
			BeeperOnTime = 125;
		}

		if (mSClock() > mS[BeeperUpdate]) {
			if (BeeperIsOn()) {
				mSTimer(mSClock(), BeeperUpdate, BeeperOffTime);
				BeeperOff();
				LEDOff(LEDRedSel);
			} else {
				mSTimer(mSClock(), BeeperUpdate, BeeperOnTime);
				BeeperOn();
				LEDOn(LEDRedSel);
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

	if (IsMulticopter) {
		if ((Abs(A[Roll].Angle) < CRASHED_ANGLE_RAD) && (Abs(A[Pitch].Angle)
				< CRASHED_ANGLE_RAD))
			mSTimer(mSClock(), CrashedTimeout, CRASHED_TIMEOUT_MS);
		else {
			if ((mSClock() > mS[CrashedTimeout]) && (DesiredThrottle
					> IdleThrottle) && !F.UsingRateControl)
				UpsideDown = true;
		}
	}

	return (UpsideDown);

} // UpsideDownMulticopter


