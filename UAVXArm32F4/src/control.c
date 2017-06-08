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

#define ALT_HOLD_MAX_ROC_MPS 0.2f // Must be changing altitude at less than this for alt. hold to be detected
#define NAV_RTH_LOCKOUT_ANGLE_RAD DegreesToRadians(10)

AxisStruct A[3];
AltStruct Alt;

real32 CameraAngle[3];
real32 OrbitCamAngle = 0.0f;
real32 AngleE, RateE;
int16 AttitudeHoldResetCount;
real32 TiltThrFF;

real32 DesiredAltitude, Altitude;
real32 AltComp, ROC, MinROCMPS, EffMinROCMPS;
real32 AltCompDecayS;
real32 TiltThrFFComp = 1.0f;
real32 BattThrFFComp = 1.0f;
real32 AltAccComp = 0.0f;
real32 HorizonTransPoint;
real32 AngleRateMix = 1.0f;
real32 YawStickScaleRadPS, RollPitchStickScaleRadPS;
real32 MaxAttitudeAngleRad;
real32 YawStickThreshold;
real32 DesiredROC = 0.0f;
real32 CruiseThrottle = 0.5f;
real32 CurrMaxTiltAngle = 0.0f;
real32 TuneKpScale;
int8 BeepTick = 0;

real32 NavHeadingTurnoutRad, FWRollPitchFFFrac, FWAileronDifferentialFrac,
		FWPitchThrottleFFFrac, MaxAltHoldCompFrac, MaxAttitudeAngleRad,
		FWMaxClimbAngleRad, FWFlapDecayS, BestROCMPS;
real32 FWGlideAngleOffsetRad = 0.0f;


void ZeroThrottleCompensation(void) {
	AltComp = 0.0f;
	BattThrFFComp = TiltThrFFComp = 1.0f;
} // ZeroThrottleCompensation


void CalcTiltThrFFComp(void) {
	const real32 TiltFFLimit = 1.0f / cosf(NAV_MAX_ANGLE_RAD);
	real32 Temp;

	if (IsMulticopter && (State == InFlight) && !F.UsingRateControl) { // forget near level check
		Temp = (1.0f / AttitudeCosine() - 1.0) * TiltThrFFFrac + 1.0f;
		Temp = Limit(Temp, 1.0f, TiltFFLimit);
		TiltThrFFComp = SlewLimit(TiltThrFFComp, Temp, TiltFFLimit, dT);
	} else
		TiltThrFFComp = 1.0f;

} // CalcTiltThrFFComp


void CalcBattThrComp(void) {

	BattThrFFComp
			= IsMulticopter && (State == InFlight) && !F.UsingRateControl ? BatterySagR
					: 1.0f;

} // CalcBattThrComp

//______________________________________________________________________________


void TrackCruiseThrottle(void) {

	if (F.Emulation)
		CruiseThrottle = IsFixedWing ? THR_DEFAULT_CRUISE_FW
				: THR_DEFAULT_CRUISE;
	else {
		//	if (IsFixedWing)
		//		CruiseThrottle = THR_DEFAULT_CRUISE_FW;
		//	else {
		if ((Abs(ROC) < ALT_HOLD_MAX_ROC_MPS) && (DesiredThrottle
				> THR_MIN_CRUISE)) {
			CruiseThrottle += (DesiredThrottle > CruiseThrottle ? 0.002f
					: -0.002f) * AltdT;
			CruiseThrottle
					= Limit(CruiseThrottle, THR_MIN_CRUISE, THR_MAX_CRUISE );
			SetP(EstCruiseThr, CruiseThrottle * 100.0f);
		}
		//	}
	}
} // TrackCruiseThrottle


void DoROCControl(real32 DesiredROC, real32 MinROCMPS, real32 MaxROCMPS) {
	real32 Pr, Dr, ROCE;

	ROCE = Limit(DesiredROC, MinROCMPS, MaxROCMPS) - ROC;

	Pr = ROCE * Alt.I.Kp;
	Dr = AltAccComp = Limit1(AccZ * Alt.I.Kd, 0.2f);

	AltComp = Limit1(Pr + Dr, IsFixedWing ? 0.5f : MaxAltHoldCompFrac);

} // DoROCControl


void AltitudeHold(real32 AltE, real32 MinROCMPS, real32 MaxROCMPS) {
	real32 Pa, Ia;

	if (UsingSpecial) {

		Pa = Limit(AltE * Alt.O.Kp, MinROCMPS, MaxROCMPS);

		Alt.O.IntE += AltE * Alt.O.Ki * AltdT;
		Ia = Alt.O.IntE;

		DesiredROC = Pa + Ia;

		if (DesiredROC > MaxROCMPS) {
			Alt.O.IntE = DesiredROC - MaxROCMPS;
			DesiredROC = MaxROCMPS;
		} else if (DesiredROC < MinROCMPS) {
			Alt.O.IntE = DesiredROC - MinROCMPS;
			DesiredROC = MinROCMPS;
		}

	} else {

		Pa = AltE * Alt.O.Kp;

		Alt.O.IntE += AltE * Alt.O.Ki * AltdT;
		Alt.O.IntE = Limit1(Alt.O.IntE, Alt.O.IL);
		Ia = Alt.O.IntE;

		DesiredROC = Pa + Ia;
	}

	DoROCControl(DesiredROC, MinROCMPS, MaxROCMPS);

} // AltitudeHold

//______________________________________________________________________________

// Fixed Wing

void DeploySpoilers(real32 a) {

	Sl = Limit(a * 0.05f, 0.0, 1.0f);
	if (Sl > 0.0f)
		AltComp = -1.0f;

} // DeploySpoilers

void AcquireAltitudeFW(void) {

	real32 AltE = DesiredAltitude - Altitude; // negative then too high => descend

	CheckRapidDescentHazard();
	if (F.RapidDescentHazard)
		DeploySpoilers((-AltE) - DESCENT_ALT_DIFF_M);
	else {
		AltitudeHold(AltE, -BestROCMPS, BestROCMPS);
		Sl = DecayX(Sl, FWFlapDecayS, AltdT);
	}

} // AcquireAltitudeFW

boolean ROCTooHigh(real32 Window) {

	return UsingDCMotors ? false : (Abs(ROCF) > Window);

} // ROCTooHigh

void AltitudeControlFW(void) {

	if (F.Glide && F.NavigationEnabled) {
		if (NavState == BoostClimb) {
			F.HoldingAlt = true;
			Sl = 0.0f;
			DoROCControl(BestROCMPS, 0.0, BestROCMPS);
		} else {
			//Sl = (NavState == AltitudeLimiting) ? Limit(Abs(Altitude - AltMaxM) * 0.05f, 0.0f, 1.0f)
			//				: 0.0f; // TODO: slew
			DeploySpoilers(Abs(Altitude - AltMaxM));
			F.HoldingAlt = false;
			AltComp = -1.0f;
		}
	} else {
		if (((NavState
				!= HoldingStation) && (NavState != PIC))) { // Navigating - using CruiseThrottle
			F.HoldingAlt = true;
			AcquireAltitudeFW();
		} else {
			CheckThrottleMoved();
			if (F.ThrottleMoving || (ROCTooHigh(1.0f) && !F.HoldingAlt)) {
				F.HoldingAlt = false;
				DesiredAltitude = Altitude;
				Alt.O.IntE = Sl = 0.0f;
				AltComp = DecayX(AltComp, AltCompDecayS, AltdT);
			} else {
				F.HoldingAlt = true;
				TrackCruiseThrottle();
				AcquireAltitudeFW(); // using Stick Throttle NOT cruise throttle
			}
		}
	}
} // AltitudeControlFW

//______________________________________________________________________________


void AcquireAltitude(void) {
	real32 AltE;
	// Synchronised to baro intervals independent of active altitude source
	real32 EffMinROCMPS;

	EffMinROCMPS = (F.RapidDescentHazard || (NavState == ReturningHome)
			|| (NavState == Transiting)) ? DESCENT_MIN_ROC_MPS : MinROCMPS;

	AltE = DesiredAltitude - Altitude;

	AltitudeHold(AltE, EffMinROCMPS, ALT_MAX_ROC_MPS);

} // AcquireAltitude


void AltitudeControl(void) {

	if (((NavState
			!= HoldingStation) && (NavState != PIC))) { // Navigating - using CruiseThrottle
		F.HoldingAlt = true;
		AcquireAltitude();
	} else {
		CheckThrottleMoved();
		if (F.ThrottleMoving || (ROCTooHigh(0.25f) && !F.HoldingAlt)) {
			F.HoldingAlt = false;
			DesiredAltitude = Altitude;
			Alt.O.IntE = 0.0f;
			AltComp = DecayX(AltComp, AltCompDecayS, AltdT);
		} else {
			F.HoldingAlt = true;
			TrackCruiseThrottle();
			AcquireAltitude(); // using Stick Throttle NOT cruise throttle
		}
	}
} // AltitudeControl


void DoAltitudeControl(void) {

	F.AltControlEnabled = !F.Bypass;

	if (F.NewAltitudeValue) { // every AltdT
		F.NewAltitudeValue = false;

		if (IsFixedWing)
			UpdateVario();

		if (F.AltControlEnabled) {
			if (IsFixedWing)
				AltitudeControlFW();
			else
				AltitudeControl();
		} else {
			//zzz check
			F.RapidDescentHazard = ROC < DESCENT_MIN_ROC_MPS;
			DesiredAltitude = Altitude; // zzz redundant
			AltComp = DecayX(AltComp, AltCompDecayS, AltdT);
			Sl = DecayX(Sl, FWFlapDecayS, AltdT);
			F.HoldingAlt = false;
		}
	}

} // DoAltitudeControl


void DetermineControl(void) {

	A[Pitch].Control = Threshold(A[Pitch].Stick, THRESHOLD_STICK)
			* MaxAttitudeAngleRad;
	A[Roll].Control = Threshold(A[Roll].Stick, THRESHOLD_STICK)
			* MaxAttitudeAngleRad;
	A[Yaw].Control = Threshold(A[Yaw].Stick, THRESHOLD_STICK)
			* YawStickScaleRadPS;

	F.YawActive = IsFixedWing ? Max(Abs(A[Roll].Control), Abs(A[Yaw].Control))
			> YawStickThreshold : Abs(A[Yaw].Control) > YawStickThreshold;

	if (F.NavigationEnabled) {
		A[Pitch].Control += A[Pitch].NavCorr;
		A[Roll].Control += A[Roll].NavCorr;
		// not used A[Yaw].Control += A[Yaw].NavCorr;
	}

} // DetermineControl


real32 ComputeRateDerivative(AxisStruct *C) {
	real32 r;

	r = LPFilter(&C->RateF, C->I.E, PROP_LP_FREQ_HZ, dT);
	C->RateD = (r - C->Ratep) * dTR;
	C->Ratep = r;
	C->RateD = Smoothr32xn(&C->RateDF, 4, C->RateD);

	return (C->RateD);

} // ComputeRateDerivative


void ZeroPIDIntegrals(void) {
	int32 a;

	for (a = Pitch; a <= Yaw; a++)
		A[a].O.IntE = A[a].I.IntE = 0.0f; // TODO: I.IntE unused

} // ZeroPIDIntegrals


void DoRateControl(int32 a) {
	real32 AngleRateMix;
	real32 Pa, Pr, Dr;
	AxisStruct *C;

	C = &A[a];

	if (P(Horizon) > 0) { // hybrid that panics back to angle when sticks centred

		AngleRateMix
				= Limit(1.0f - (CurrMaxRollPitchStick / HorizonTransPoint), 0.0f, 1.0f);

		C->O.Desired = Limit1(C->Control * AngleRateMix, C->O.Max);

		C->O.E = C->O.Desired - C->Angle;

		Pa = C->O.E * C->O.Kp;
		C->O.IntE = 0.0f; // for flip back to angle mode

		C->I.Desired = Pa + C->Control * (RollPitchStickScaleRadPS
				/ MaxAttitudeAngleRad) * (1.0f - AngleRateMix);
	} else
		// pure rate control
		C->I.Desired = C->Control * (RollPitchStickScaleRadPS
				/ MaxAttitudeAngleRad);

	C->I.E = Rate[a] - Limit1(C->I.Desired, C->I.Max);

	Pr = C->I.E * C->I.Kp;
	Dr = ComputeRateDerivative(C) * C->I.Kd;

	C->Out = Limit1(Pr + Dr, 1.0f);

} // DoRateControl


void DoAngleControl(int32 a) { // with Ming Liu
	real32 Pa, Ia, Pr, Dr, S, Windup;
	AxisStruct *C;

	C = &A[a];

	C->O.Desired = Limit1(C->Control, C->O.Max);

	//	if (UsingVTOLMode) {
	//		// TODO: needs a transition - MORE THOUGHT
	//		C->O.E = C->O.Desired - (C->Angle - DegreesToRadians(90));
	//	} else
	C->O.E = C->O.Desired - C->Angle;

	if (false) { //UsingSpecial) {

		Pa = C->O.E * C->O.Kp;

		C->O.IntE += C->O.E * C->O.Ki * dT;
		Ia = C->O.IntE;

		C->I.Desired = Pa + Ia;

		Windup = Abs(C->I.Desired) - C->I.Max;
		if (Windup > 0.0f) {
			S = Sign(C->I.Desired);
			C->O.IntE = S * (Abs(C->O.IntE) - Windup);
			C->I.Desired = S * C->I.Max;
		}

	} else {

		Pa = C->O.E * C->O.Kp;

		C->O.IntE += C->O.E * C->O.Ki * dT;
		C->O.IntE = Limit1(C->O.IntE, C->O.IL);
		Ia = C->O.IntE;

		C->I.Desired = Limit1(Pa + Ia, C->I.Max);

	}

	C->I.E = Rate[a] - C->I.Desired;

	Pr = C->I.E * C->I.Kp;
	Dr = ComputeRateDerivative(C) * C->I.Kd;

	C->Out = Limit1(Pr + Dr, 1.0f);

} // DoAngleControl


real32 MinimumTurn(real32 Desired) {
	real32 HE, absHE;
	static real32 TurnSign;
	static boolean TurnCommit = false;

	HE = MakePi(Desired - Heading);

	if (IsFixedWing) {
		if (NavState == UsingThermal) {
			TurnCommit = true;
			TurnSign = 1.0f;
			HE = Make2Pi(Desired - Heading); // turn right
		} else {
			HE = MakePi(Desired - Heading);
			absHE = fabsf(HE);
			if (absHE > DegreesToRadians(160)) {
				TurnCommit = true;
				TurnSign = Sign(HE);
			} else if (absHE < DegreesToRadians(135))
				TurnCommit = false;

			if (TurnCommit)
				HE = TurnSign * absHE;
		}
	}
	return (HE);

} // MinimumTurn

static void DoYawControlFW(void) {
	real32 Pa, Pr;
	static real32 KpScale = 1.0f;
	real32 NewRollCorr;
	AxisStruct *C;

	C = &A[Yaw];

	if (F.YawActive) {
		DesiredHeading = Heading;
		KpScale = 1.0f;
	} else if (NavState != PIC) {
		DesiredHeading = Nav.DesiredHeading;
		KpScale = 1.0f;
	}

	C->O.E = HeadingE = MinimumTurn(DesiredHeading);
	C->O.E = Limit1(C->O.E, NavHeadingTurnoutRad); // 150 30

	Pa = C->O.E * C->O.Kp * 0.1f * KpScale; //60deg * 20 * 0.1 * 0.8 -> 12

	C->I.Desired = Pa;

	C->I.E = (C->I.Desired + C->Control) - Rate[Yaw];
	Pr = C->I.E * C->I.Kp;

	C->Out = Limit1(Pr, 1.0); // needs to be driven by LR acc

	NewRollCorr = atanf(C->I.Desired * Airspeed * GRAVITY_MPS_S_R);
	A[Roll].NavCorr = Limit1(NewRollCorr, Nav.MaxAngle);

} // DoYawControlFW


static void DoYawControl(void) {
	real32 Pa, Pr;
	AxisStruct *C;

	C = &A[Yaw];

	if (F.UsingRateControl) {

		C->I.Desired = Limit1(C->Control, C->I.Max);
		C->I.E = C->I.Desired - Rate[Yaw];
		Pr = C->I.E * C->I.Kp;

		C->Out = Limit1(Pr, 1.0);

	} else {

		if (F.YawActive)
			DesiredHeading = Heading;
		else if (F.OrbitingWP || F.RapidDescentHazard || F.UsingPOI)
			DesiredHeading = Nav.DesiredHeading;

		C->O.E = HeadingE = MinimumTurn(DesiredHeading);
		C->O.E = Limit1(C->O.E, NavHeadingTurnoutRad);

		Pa = C->O.E * C->O.Kp;

		C->I.Desired = Limit1(Pa, C->I.Max);

		C->I.E = (C->I.Desired + C->Control) - Rate[Yaw];
		Pr = C->I.E * C->I.Kp;

		C->Out = Limit1(Pr, 1.0f);
	}

} // DoYawControl


void DoControl(void) {
	int32 a;

	CurrMaxTiltAngle = Max(Abs(A[Roll].Angle), Abs(A[Pitch].Angle));
	F.NearLevel = CurrMaxTiltAngle < NAV_RTH_LOCKOUT_ANGLE_RAD;

	//CalcTiltThrFFComp();
	//CalcBattThrComp();

	DetermineControl();

	if (IsFixedWing)
		DoYawControlFW(); // must do first for fixed wing turn coordination
	else
		DoYawControl();

	for (a = Pitch; a <= Roll; a++)
		if (F.UsingRateControl)
			DoRateControl(a);
		else
			DoAngleControl(a);

	UpdateDrives();

} // DoControl


void InitControl(void) {
	int32 a;
	AxisStruct *C;

	A[Roll].O.E = Sl = 0.0f;
	ZeroThrottleCompensation();

	for (a = Pitch; a <= Yaw; a++) {
		C = &A[a];
		C->O.IntE = 0.0f;
		C->O.Dp = 0.0f;
		C->Ratep = 0.0f;
		C->I.Dp = 0.0f;

		C->NavCorr = 0.0f;
		C->RateF.Primed = false;
		C->RateDF.Primed = false;

		C->Out = 0.0f;
	}

	Acc[Z] = -GRAVITY_MPS_S;

} // InitControl

//#define MATRIXPILOT

#if defined(MATRIXPILOT)
// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


#include "defines.h"
#include "states.h"
#include "config.h"
#include "navigate.h"
#include "behaviour.h"
#include "servoPrepare.h"
#include "airspeedCntrl.h"
#include "altitudeCntrl.h"
#include "helicalTurnCntrl.h"
#include "../libDCM/deadReckoning.h"
#include "../libDCM/mathlibNAV.h"
#include "../libDCM/rmat.h"
#include <math.h>
#include <stdlib.h> // for declaration of function abs() under gcc
#include "options_airspeed.h"

//#ifndef RTL_PITCH_DOWN
//#define RTL_PITCH_DOWN (0.0)
//#endif // RLT_PITCH_DOWN

//#ifndef ANGLE_OF_ATTACK_NORMAL
//#define ANGLE_OF_ATTACK_NORMAL (0.0)
//#endif // ANGLE_OF_ATTACK_NORMAL

//#ifndef ANGLE_OF_ATTACK_INVERTED
//#define ANGLE_OF_ATTACK_INVERTED (0.0)
//#endif // ANGLE_OF_ATTACK_INVERTED

//#ifndef ELEVATOR_TRIM_NORMAL
//#define ELEVATOR_TRIM_NORMAL (0.0)
//#endif // ELEVATOR_TRIM_NORMAL

//#ifndef ELEVATOR_TRIM_INVERTED
//#define ELEVATOR_TRIM_INVERTED (0.0)
//#endif // ELEVATOR_TRIM_INVERTED

//#ifndef CRUISE_SPEED
//#define CRUISE_SPEED (12.0)
//#endif // CRUISE_SPEED

#ifndef INVERTED_NEUTRAL_PITCH
#define INVERTED_NEUTRAL_PITCH (0.0)
#endif

#define RTLKICK            ((real32)(gains.RtlPitchDown*(RMAX/57.3)))
#define INVNPITCH          ((real32)(INVERTED_NEUTRAL_PITCH*(RMAX/57.3)))
#define AOA_NORMAL         ((real32)(turns.AngleOfAttackNormal*(RMAX/57.3)))
#define AOA_INVERTED       ((real32)(turns.AngleOfAttackInverted*(RMAX/57.3)))
#define ELEV_TRIM_NORMAL   ((real32)SERVORANGE*turns.ElevatorTrimNormal)
#define ELEV_TRIM_INVERTED ((real32)SERVORANGE*turns.ElevatorTrimInverted)
#define STALL_SPEED_MPS 	((real32)turns.RefSpeed*50.0) // assume stall speed approximately 1/2 of reference speed
#define AOA_OFFSET           ((real32)((AOA_NORMAL + AOA_INVERTED)*0.5f)) // offset is the average of the two values
#define AOA_SLOPE            ((real32)((AOA_NORMAL - AOA_INVERTED)*0.25f)) // multiply by 4 because base speed is 1/2 of cruise
#define ELEVATOR_TRIM_OFFSET ((real32)((ELEV_TRIM_NORMAL + ELEV_TRIM_INVERTED)*0.5f)) // offset is the average of the two values
#define ELEVATOR_TRIM_SLOPE  ((real32)((ELEV_TRIM_NORMAL - ELEV_TRIM_INVERTED)*0.25f)) // multiply by 4 because base speed is 1/2 of cruise
#define MAX_INPUT            (1.0f) // maximum input in pwm units
#define TURN_CALC_MINIMUM_AIRSPEED     (5.0f) // minimum value of airspeed in cm/sec to be used in tilt computation,
// mainly used for ground testing of turning tilt, which would go to zero at zero airspeed

real32 tiltError[3];
real32 desiredRotationRateRadians[3];
real32 rotationRateError[3];
//real32 angleOfAttack;

static real32 estimatedLift;
static real32 relativeLoading;

// Compute estimated wing lift based on orientation.
// This information can be determined directly from the accelerometers,
// but this creates an unstable feedback loop through the elevator.
// It is better to compute the lift as a feed forward term.
// It can be shown that wing lift divided by mass times gravity during a helical turn is
// equal to (Z + X * (X/Z)), where X, Y, and Z are the 3 elements
// of the bottom row of the direction cosine matrix, in real (floating point) values.
// Note: In principle, X/Z can be computed from X and Z, but since X, Z, and X/Z are already available
// in the helical turn control computations, it is more efficient to supply X/Z rather than recompute it.
// The computation of lift can use a mix of actual and desired values of X, Z and X/Z.
// The selection of the mix of actual and desired values for X, Z and X/Z is done when the routine is called.
// The following routine computes 1/16 of the ratio of wing loading divided by mass*gravity.
// The scale factor of 1/16 was selected to handle a fractional representation of a maximum lift/mg of more than 4.

static real32 wingLift(real32 X, real32 Z, real32 XoverZ) {
	// compute (1/16*(Z + X *(X/Z)))*2^16

	return X * XoverZ + Z;
}

// Compute relative wing loading, 2**15*(Load/(mass*Gravity))*(V0/V)**2
// This number ranges from -2**15 to +2**15. Either extreme represents stall conditions.
// Typical values for relative wing loading under normal conditions are around 1/8 to 1/4 of the stall value.
// V0 is stall speed in centimeters per second during level flight, V is airspeed in centimeters per second.
// Load is wing loading in acceleration units, G is gravity.
// Relative wing loading is 2**15 when airspeed is equal to stall speed during level, unaccelerated flight
//
// Implement as 8*((2**16)*(a/(16g))*(V0/V)**2
//

static real32 relativeWingLoading(real32 wingLoad, real32 airSpeed) {
	// wingLoad is(2**16)*((wing_load / mass*gravity) / 16)
	// stallSpeed is the stall speed in centimeters per second
	// airSpeed is the air speed in centimeters per second

	real32 result\;
	real32 Temp;

	if (airSpeed > stallSpeed) {
		Temp = Sqr(STALL_SPEED_MPS / airSpeed) * wingLoad;
		result = (abs(Temp) < 4095) ? Temp : (wingLoad > 0) ? 32767 : -32767;
	} else
	result = 0;

	return result;
}

// helicalTurnCntrl determines the values of the elements of the bottom row of rmat
// as well as the required rotation rates in the body frame that are required to make a coordinated turn.
// The required values for the bottom row of rmat are placed in the vector desiredTilt.
// Desired tilt is computed from the helical turn parameters from desired climb rate, turn rate, and airspeed.
// The cross product of rmat[6,7,8] with the desiredTilt produces the orientation error.
// The desired rotation rate in the body frame is computed by multiplying desired turn rate times actual tilt vector.
// The rotation rate error is the actual rotation rate vector in the body frame minus the desired rotation rate vector.
// The tilt and rate vectors are then used by roll, pitch, and yaw control to deflect control surfaces.

void helicalTurnCntrl(void) {
	union longww Temp;
	real32 pitchAdjustAngleOfAttack;
	real32 rollErrorVector[3];
	real32 rtlkick;
	real32 desiredPitch;
	real32 steeringInput;
	real32 desiredTurnRateRadians;
	real32 desiredTiltVector[3];
	real32 desiredRotationRateGyro[3];
	real32 airSpeed;
	union longww desiredTilt;
	real32 desiredPitchVector[2];
	real32 desiredPerpendicularPitchVector[2];
	real32 actualPitchVector[2];
	real32 pitchDot;
	real32 pitchCross;
	real32 pitchError;
	real32 pitchEarthBodyProjection[2];
	real32 angleOfAttack;
#ifdef TestGains
	state_flags._.GPS_steering = false; // turn off navigation
	state_flags._.pitch_feedback = true; // turn on stabilization
	airSpeed = 9.81f; // for testing purposes, an airspeed is needed
#else
	airSpeed = air_speed_3DIMU;
	if (airSpeed < TURN_CALC_MINIMUM_AIRSPEED)
	airSpeed = TURN_CALC_MINIMUM_AIRSPEED;
#endif

	// determine the desired turn rate as the sum of navigation and fly by wire.
	// this allows the pilot to override navigation if needed.
	steeringInput = 0; // just in case no airframe type is specified or radio is off
	if (udb_flags._.radio_on == 1) {
#if ( (AIRFRAME_TYPE == AIRFRAME_STANDARD) || (AIRFRAME_TYPE == AIRFRAME_GLIDER) )
		if (AILERON_INPUT_CHANNEL != CHANNEL_UNUSED) // compiler is smart about this
		{
			steeringInput = udb_pwIn[AILERON_INPUT_CHANNEL]
			- udb_pwTrim[AILERON_INPUT_CHANNEL];
			steeringInput = REVERSE_IF_NEEDED(AILERON_CHANNEL_REVERSED,
					steeringInput);
		} else if (RUDDER_INPUT_CHANNEL != CHANNEL_UNUSED) {
			steeringInput = udb_pwIn[RUDDER_INPUT_CHANNEL]
			- udb_pwTrim[RUDDER_INPUT_CHANNEL];
			steeringInput = REVERSE_IF_NEEDED(RUDDER_CHANNEL_REVERSED,
					steeringInput);
		} else
		steeringInput = 0;

#endif // AIRFRAME_STANDARD
#if (AIRFRAME_TYPE == AIRFRAME_VTAIL)
		// use aileron channel if it is available, otherwise use rudder
		if (AILERON_INPUT_CHANNEL != CHANNEL_UNUSED) // compiler is smart about this
		{
			steeringInput = udb_pwIn[AILERON_INPUT_CHANNEL]
			- udb_pwTrim[AILERON_INPUT_CHANNEL];
			steeringInput = REVERSE_IF_NEEDED(AILERON_CHANNEL_REVERSED,
					steeringInput);
		} else if (RUDDER_INPUT_CHANNEL != CHANNEL_UNUSED) {
			// unmix the Vtail
			real32 rudderInput = REVERSE_IF_NEEDED(RUDDER_CHANNEL_REVERSED,
					(udb_pwIn[RUDDER_INPUT_CHANNEL]
							- udb_pwTrim[RUDDER_INPUT_CHANNEL]));
			real32 elevatorInput = REVERSE_IF_NEEDED(ELEVATOR_CHANNEL_REVERSED,
					(udb_pwIn[ELEVATOR_INPUT_CHANNEL]
							- udb_pwTrim[ELEVATOR_INPUT_CHANNEL]));
			steeringInput = (-rudderInput + elevatorInput);
		} else
		steeringInput = 0;

#endif // AIRFRAME_VTAIL
#if (AIRFRAME_TYPE == AIRFRAME_DELTA)
		// delta wing must have an aileron input, so use that
		// unmix the elevons
		real32 aileronInput = REVERSE_IF_NEEDED(AILERON_CHANNEL_REVERSED,
				(udb_pwIn[AILERON_INPUT_CHANNEL]
						- udb_pwTrim[AILERON_INPUT_CHANNEL]));
		real32 elevatorInput = REVERSE_IF_NEEDED(ELEVATOR_CHANNEL_REVERSED,
				(udb_pwIn[ELEVATOR_INPUT_CHANNEL]
						- udb_pwTrim[ELEVATOR_INPUT_CHANNEL]));
		steeringInput = REVERSE_IF_NEEDED(ELEVON_VTAIL_SURFACES_REVERSED,
				((elevatorInput - aileronInput)));
#endif // AIRFRAME_DELTA
	}

	steeringInput = Limit1(steeringInput, MAX_INPUT);

	// note that total steering is the sum of pilot input and waypoint navigation,
	// so that the pilot always has some say in the matter

	Temp = (steeringInput * turngainfbw) / (2.0f * MAX_INPUT);

	if ((settings._.AileronNavigation || settings._.RudderNavigation)
			&& state_flags._.GPS_steering)
	Temp += (real32) navigate_determine_deflection('t');

	desiredTurnRateRadians = Limit(Temp, -2.0f * (RMAX + 1),
			2.0f * (RMAX - 1));

	// compute the desired tilt from desired turn rate and air speed
	// range for acceleration is plus minus 4 times gravity
	// range for turning rate is plus minus 4 radians per second

	// desiredTilt is the ratio(-rmat[6]/rmat[8]), times RMAX/2 required for the turn
	// desiredTilt = desiredTurnRate * airSpeed / gravity
	// desiredTilt = RMAX/2*"real desired tilt"
	// desiredTurnRate = RMAX/2*"real desired turn rate", desired turn rate in radians per second
	// airSpeed is air speed centimeters per second
	// gravity is 981 centimeters per second per second

	desiredTilt = -(desiredTurnRateRadians * airSpeed) / GRAVITY_MPS_S;

	// limit the lateral acceleration to +- 4 times gravity, total wing loading approximately 4.12 times gravity

	desiredTilt = Limit(desiredTilt, -2.0 * RMAX + 1,
			2.0 * RMAX - 1);

	desiredTurnRateRadians = (desiredTilt * GRAVITY_MPS_S) / airSpeed;

	// Compute the amount of lift needed to perform the desired turn
	// Tests show that the best estimate of lift is obtained using
	// actual values of rmat[6] and rmat[8], and the commanded value of their ratio
	estimatedLift = wingLift(rmat[6], rmat[8], desiredTilt);

	// compute angle of attack and elevator trim based on relative wing loading.
	// relative wing loading is the ratio of wing loading divided by the stall wing loading, as a function of air speed
	// both angle of attack and trim are computed by a linear approximation as a function of relative loading:
	// y = (2m)*(x/2) + b, y is either angle of attack or elevator trim.
	// x is relative wing loading. (x/2 is computed instead of x)
	// 2m and b are determined from values of angle of attack and trim at stall speed, normal and inverted.
	// b =  (y_normal + y_inverted) / 2.
	// 2m = (y_normal - y_inverted).

	// If airspeed is greater than stall speed, compute angle of attack and elevator trim,
	// otherwise set AoA and trim to zero.

	if (air_speed_3DIMU > STALL_SPEED_MPS) {
		// compute "x/2", the relative wing loading
		relativeLoading = relativeWingLoading(estimatedLift, air_speed_3DIMU);

		// add mx to b
		angleOfAttack = AOA_SLOPE * relativeLoading + AOA_OFFSET;

		// project angle of attack into the earth frame
		pitchAdjustAngleOfAttack = angleOfAttack * rmat[8];

		// similarly, compute elevator trim
		Temp = ELEVATOR_TRIM_SLOPE * relativeLoading;
		elevatorLoadingTrim = ELEVATOR_TRIM_OFFSET + Temp;
	} else {
		angleOfAttack = 0;
		pitchAdjustAngleOfAttack = 0;
		elevatorLoadingTrim = 0;
	}

	// compute desired rotation rate vector in body frame, scaling is same as gyro signal

	VectorScale(3, desiredRotationRateGyro, &rmat[6], desiredTurnRateRadians); // this operation has side effect of dividing by 2

	// compute desired rotation rate vector in body frame, scaling is in RMAX/2*radians/sec

	VectorScale(3, desiredRotationRateRadians, &rmat[6], desiredTurnRateRadians); // this produces half of what we want
	VectorAdd(3, desiredRotationRateRadians, desiredRotationRateRadians,
			desiredRotationRateRadians); // double

	// incorporate roll into desired tilt vector

	desiredTiltVector[0] = desiredTilt;
	desiredTiltVector[1] = 0;
	desiredTiltVector[2] = RMAX * 0.5f; // the divide by 2 is to account for the RMAX/2 scaling in both tilt and rotation rate
	vector3_normalize(desiredTiltVector, desiredTiltVector); // make sure tilt vector has magnitude RMAX

	// incorporate pitch into desired tilt vector
	// compute return to launch pitch down kick for unpowered RTL
	rtlkick = (!udb_flags._.radio_on && state_flags._.GPS_steering) ? RTLKICK
	: 0;

#if (GLIDE_AIRSPEED_CONTROL == 1)
	// Compute Matt's glider pitch adjustment
	fractional aspd_pitch_adj = gliding_airspeed_pitch_adjust();
	// Compute total desired pitch
	desiredPitch = - rtlkick + aspd_pitch_adj + pitchAltitudeAdjust;
#else
	desiredPitch = -rtlkick + pitchAltitudeAdjust;
#endif

	// Adjustment for inverted flight
	if (!canStabilizeInverted() || !desired_behavior._.inverted)
	desiredTiltVector[1] = -desiredPitch - pitchAdjustAngleOfAttack;
	else {
		// inverted flight
		desiredTiltVector[0] = -desiredTiltVector[0];
		desiredTiltVector[1] = -desiredPitch - pitchAdjustAngleOfAttack
		- INVNPITCH; // only one of the adjustments is not zero
		desiredTiltVector[2] = -desiredTiltVector[2];
	}

	vector3_normalize(desiredTiltVector, desiredTiltVector); // make sure tilt vector has magnitude RMAX

	// compute roll error

	VectorCross(rollErrorVector, &rmat[6], desiredTiltVector); // compute tilt orientation error
	if (VectorDotProduct(3, &rmat[6], desiredTiltVector) < 0) // more than 90 degree error
	vector3_normalize(rollErrorVector, rollErrorVector); // for more than 90 degrees, make the tilt error vector parallel to desired axis, with magnitude RMAX

	tiltError[1] = rollErrorVector[1];

	// compute pitch error

	// start by computing the projection of earth frame pitch error to body frame

	pitchEarthBodyProjection[0] = rmat[6];
	pitchEarthBodyProjection[1] = rmat[8];

	// normalize the projection vector and compute the cosine of the actual pitch as a side effect

	actualPitchVector[1] = (real32) vector2_normalize(pitchEarthBodyProjection,
			pitchEarthBodyProjection);

	// complete the actual pitch vector

	actualPitchVector[0] = rmat[7];

	// compute the desired pitch vector

	desiredPitchVector[0] = -desiredPitch;
	desiredPitchVector[1] = RMAX;
	vector2_normalize(desiredPitchVector, desiredPitchVector);

	// rotate desired pitch vector by 90 degrees to be able to compute cross product using VectorDot

	desiredPerpendicularPitchVector[0] = desiredPitchVector[1];
	desiredPerpendicularPitchVector[1] = -desiredPitchVector[0];

	// compute pitchDot, the dot product of actual and desired pitch vector
	// (the 2* that appears in several of the following expressions is a result of the Q2.14 format)

	pitchDot = 2.0f
	* VectorDotProduct(2, actualPitchVector, desiredPitchVector);

	// compute pitchCross, the cross product of the actual and desired pitch vector

	pitchCross = 2.0f * VectorDotProduct(2, actualPitchVector,
			desiredPerpendicularPitchVector);

	pitchError = (pitchDot > 0) ? pitchCross : (pitchCross > 0) ? RMAX : -RMAX;

	// multiply the normalized rmat[6], rmat[8] vector by the pitch error
	VectorScale(2, pitchEarthBodyProjection, pitchEarthBodyProjection,
			pitchError);
	tiltError[0] = 2.0f * pitchEarthBodyProjection[1];
	tiltError[2] = -2.0f * pitchEarthBodyProjection[0];

	// compute the rotation rate error vector
	VectorSubtract(3, rotationRateError, omegaAccum, desiredRotationRateGyro);
}
#endif
