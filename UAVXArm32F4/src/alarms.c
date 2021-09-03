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

void Probe(boolean p) {
#if defined(USE_AUX3_PROBE_PIN)
	if (GPIOPins[WPMissionOrProbeSel].Used)
		DigitalWrite(&GPIOPins[WPMissionOrProbeSel].P, p);
#endif
} // Probe

void Marker(void) {
	Probe(true);
	Probe(false);
} // Marker

void CheckLandingSwitch(void) { // sampled every PID cycle
	static int16 Count = 100;
	static boolean SwitchP = false;
	boolean Switch;

	if (F.Emulation)
		F.AccUBump = F.LandingSwitch = (Altitude < 0.1f);
	else {

		Switch = !DigitalRead(&GPIOPins[LandingSel].P); // active to ground

		if (Switch != SwitchP)
			if (Count <= 0) {
				SwitchP = Switch;
				Count = 100;
			} else
				Count--;
		else
			Count = 100;

		F.LandingSwitch = SwitchP;
	}

} // CheckLandingSwitch


boolean Armed(void) {
	static boolean ArmedP = false;
	boolean NewUplinkState;
	boolean IsArmed;

	if (State != InFlight)
		F.IsArmed = ((TxSwitchArmed || (ArmingMethod == SwitchArming))
				&& ArmingSwitch) && !F.RCMapFail;

	return (F.IsArmed);

} // Armed

boolean FailPreflight(void) {
	boolean r;

	r = !((UAVXAirframe == Instrumentation) || ( //
			F.Signal //
					&& (!F.RCMapFail) //
					&& (RCStart <= 0) //
					&& !(Armed() && FirstPass) //
					&& !F.ThrottleOpen //
					&& (RC[NavModeRC] <= FromPercent(20)) //
					&& F.IMUActive //
					&& F.IMUCalibrated //
					&& (F.BaroActive || (busDev[baroSel].type == noBaro)) //
					&& ((F.MagnetometerActive && F.MagnetometerCalibrated) //
							|| ((busDev[CurrMagSel].type == noMag) //
									|| F.IsFixedWing)) //
					&& !(F.LowBatt || F.sioFatal || F.ReturnHome || F.Navigate) //
			));

	PreflightFail = F.ReturnHome || F.Navigate || !F.Signal;

	return (r);

} // FailPreflight


void DoCalibrationAlarm(void) {

	if (!F.IMUCalibrated
			|| !((F.MagnetometerActive && F.MagnetometerCalibrated)
					|| F.IsFixedWing)) {
		if (mSTimeout(CalibrationTimeoutmS)) {
			mSTimer(CalibrationTimeoutmS, 500);
			LEDToggle(ledYellowSel);
		}
	}

} // DoAccCalibrationAlarm

void DoBeep(uint16 t, uint16 d) {
	int32 i;

	BeeperOn();
	for (i = 0; i < t * 100; i++)
		Delay1uS(1000);

	BeeperOff();
	for (i = 0; i < d * 100; i++)
		Delay1uS(1000);

} // DoBeep

void DoBeeps(uint16 b) {
	uint16 i;

	for (i = 0; i < b; i++)
		DoBeep(2, 8);

} // DoStartingBeeps

void ScheduleBeeper(timemS w) {

	if (!F.BeeperInUse) {
		BeeperOn();
		mSTimer(BeeperTimeoutmS, w);
	}

} // ScheduleNavBeeper


void CheckAlarms(void) {

	static timemS BeeperOffTime = 100;
	static timemS BeeperOnTime = 100;

	F.BeeperInUse = PreflightFail || F.FenceAlarm || F.LowBatt || (State == Shutdown) || (State
			== ThrottleOpenCheck) || (NavState == Descending)
			|| AltHoldAlarmActive;

	if (F.BeeperInUse) {
		if (F.FenceAlarm) {
			BeeperOffTime = 3000;
			BeeperOnTime = 250;
		} else if (F.LowBatt) {
			BeeperOffTime = 600;
			BeeperOnTime = 600;
		} else if (AltHoldAlarmActive) {
			BeeperOffTime = 1000;
			BeeperOnTime = 1000;
		} else if (State == Shutdown) {
			BeeperOffTime = 4750;
			BeeperOnTime = 250;
		} else { //default
			BeeperOffTime = 125;
			BeeperOnTime = 125;
		}

		if (mSTimeout(BeeperUpdatemS)) {
			if (BeeperIsOn) {
				mSTimer(BeeperUpdatemS, BeeperOffTime);
				BeeperOff();
			} else {
				mSTimer(BeeperUpdatemS, BeeperOnTime);
				BeeperOn();
			}
		}
	} else {
		if (mSTimeout(BeeperTimeoutmS))
			BeeperOff();
	}

} // CheckAlarms


void Catastrophe(void) {

	/*
	 F.DrivesArmed = false;

	 while (true) {
	 LEDsOn();
	 Delay1mS(500);
	 LEDsOff();
	 Delay1mS(500);
	 }
	 */
	systemReset(false);

} // Catastrophe


boolean UpsideDownMulticopter(void) {

	boolean UpsideDown;

	UpsideDown = false;
#if defined(CHECK_INVERTED)

	if (IsMulticopter) {
		if (Abs(Angle[Roll]) < CRASHED_ANGLE_RAD)
			mSTimer(CrashedTimeoutmS, CRASHED_TIMEOUT_MS);
		else {
			if (mSTimeout(CrashedTimeoutmS) && (DesiredThrottle > IdleThrottle)
					&& F.UsingAngleControl)
				UpsideDown = true;
			UpsideDown = false;
		}
	}
#endif

	return (UpsideDown);

} // UpsideDownMulticopter


