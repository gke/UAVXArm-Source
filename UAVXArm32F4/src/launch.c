// ===============================================================================================
// =                                UAVX Quadrocopter Controller                                 =
// =                           Copyright (c) 2008 by Prof. Greg Egan                             =
// =                 Original V3.15 Copyright (c) 2007 Ing. Wolfgang Mahringer                   =
// =                     http://code.google.com/p/uavp-mods/ http://uavp.ch                      =
// ===============================================================================================

//    This is part of UAVX.
//    Auto Launch concept rewritten from iNav

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/


#include "UAVX.h"

const struct {
	timemS waittimeoutmS;
	uint32 timethreshmS;
	real32 velthresh;
	real32 accthresh;
	real32 idlethrottle;
	timemS glidemS;
	timemS spinupmS;
	timemS timemS;
	real32 angle;
	real32 swingrate;
} Launch = { 3000, 40, 3.0f, GRAVITY_MPS_S * 1.5f, FromPercent(0), 500, 300,
		5000L, DegreesToRadians(45), DegreesToRadians(100) };

uint8 LaunchState = initLaunch;
timemS LaunchTimermS;

boolean LaunchDetected(timemS NowmS) { // main contribution from iNav
	real32 swingVel;
	boolean isHandLaunched, isSwingLaunched, AngleOK, Launched;

	AngleOK = Max(Abs(Angle[Roll]), Abs(Angle[Pitch])) < Launch.angle;

	swingVel = (Abs(Rate[Yaw]) > Launch.swingrate) ? Abs(Acc[LR] / Rate[Yaw])
			: 0.0f;

	isHandLaunched = AngleOK && (Acc[BF] > Launch.accthresh);
	isSwingLaunched = (swingVel > Launch.velthresh) && (Acc[BF] > 0.0f);

	Launched = false;
	if (isHandLaunched || isSwingLaunched) { // debounce
		if ((NowmS - LaunchTimermS) > Launch.timethreshmS) {
			Launched = true;
			LaunchTimermS = NowmS;
		}
	} else
		LaunchTimermS = NowmS;

	return (Launched);
} // updateFWLaunchDetector


real32 ThrottleUp(timemS TimemS) {
	real32 Thr;

	if (Launch.spinupmS > 0) { // ramp up motor(s)
		TimemS = Limit(TimemS, 0, Launch.spinupmS);
		IdleThrottle = Max(IdleThrottle, Launch.idlethrottle);
		Thr = scaleRangef(TimemS, 0, Launch.spinupmS, IdleThrottle,
				FWClimbThrottleFrac);
	} else
		Thr = FWClimbThrottleFrac;

	return (Thr);
} // ThrottleUp

void LaunchFW(void) {

	AttitudeMode = AngleMode;
	F.UsingAngleControl = true;
	A[Pitch].Stick = FWMaxClimbAngleRad / A[Pitch].P.Max;

	switch (LaunchState) {
	case initLaunch:
		mSTimer(NavStateTimeout, Launch.waittimeoutmS);
		DesiredThrottle = Max(IdleThrottle, Launch.idlethrottle);
		LaunchTimermS = mSClock();
		LaunchState = waitLaunch;
		break;
	case waitLaunch:
		ZeroIntegrators();
		ZeroThrottleCompensation();
		DesiredThrottle = Max(IdleThrottle, Launch.idlethrottle);

		if (mSTimeout(NavStateTimeout)) { // 3Sec
			F.DrivesArmed = false;
			LaunchState = finishedLaunch;
		} else {
			if (LaunchDetected(mSClock())) { // 40mS
				mSTimer(MotorStart, Launch.glidemS);
				mSTimer(NavStateTimeout, Launch.timemS);
				LaunchState = doingLaunch;
			}
		}
		break;
	case doingLaunch:
		if (mSTimeout(NavStateTimeout)) // 5Sec
			LaunchState = finishedLaunch;
		else {
			DesiredThrottle = mSTimeout(MotorStart) ?
			ThrottleUp(mSClock() - mS[MotorStart])
					: Max(IdleThrottle, Launch.idlethrottle);
		}
		break;
	case finishedLaunch:
		State = InFlight;
	default:
		break;
	} // switch

} // LaunchFW


