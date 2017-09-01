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

real32 CameraAngle[3];
real32 OrbitCamAngle = 0.0f;
real32 AngleE, RateE;
real32 TiltThrFF;
real32 CurrDerivativeLPFreqHz;
idx DerivativeLPFOrder;

real32 DesiredAltitude, Altitude;
real32 AltComp, ROC, MinROCMPS, EffMinROCMPS;
real32 AltCompDecayS;
real32 TiltThrFFComp = 1.0f;
real32 BattThrFFComp = 1.0f;
real32 AltAccComp = 0.0f;
real32 HorizonTransPoint;
real32 AngleRateMix = 1.0f;
real32 StickDeadZone;
real32 DesiredROC = 0.0f;
real32 CruiseThrottle = 0.5f;
real32 CurrMaxTiltAngle = 0.0f;
real32 TuneKpScale;
int8 BeepTick = 0;

AltStruct Alt;

real32 FWRollPitchFFFrac, FWAileronDifferentialFrac, FWPitchThrottleFFFrac,
		MaxAltHoldCompFrac, FWMaxClimbAngleRad, FWBoardPitchAngleRad,
		FWSpoilerDecayS, FWAileronRudderFFFrac, FWAltSpoilerFFFrac, BestROCMPS;
real32 FWGlideAngleOffsetRad = 0.0f;
real32 ThrottleGain, gainScale;

void determineGainScale(void) {

	gainScale
			= ((ThrottleGain > 0.0f) && (DesiredThrottle > CruiseThrottle)) ? ThrottleGain
					* (DesiredThrottle - CruiseThrottle) / (1.0f
					- CruiseThrottle)
					: 1.0f;
} // determineGainScale

real32 conditionOut(real32 v) {

	return (Limit1(v * gainScale, 1.0f));

} // conditionOut

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
		TiltThrFFComp = SlewLimit(&TiltThrFFComp, Temp, TiltFFLimit, dT);
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
		CruiseThrottle = IsFixedWing ? THR_DEFAULT_CRUISE_FW_STICK
				: THR_DEFAULT_CRUISE_STICK;
	else {
		if ((Abs(ROC) < ALT_HOLD_MAX_ROC_MPS) && (DesiredThrottle
				> THR_MIN_ALT_HOLD_STICK)) {
			CruiseThrottle
					+= (((DesiredThrottle + AltComp) > CruiseThrottle) ? 0.002f
							: -0.002f) * AltdT;
			CruiseThrottle
					= Limit(CruiseThrottle, THR_MIN_ALT_HOLD_STICK, THR_MAX_ALT_HOLD_STICK);
			SetP(EstCruiseThr, CruiseThrottle * 100.0f);
		}
	}
} // TrackCruiseThrottle


void DoROCControl(real32 DesiredROC, real32 MinROCMPS, real32 MaxROCMPS) {
	real32 Pr, Dr;

	Alt.VelE = Limit(DesiredROC, MinROCMPS, MaxROCMPS) - ROC;

	Pr = Alt.VelE * Alt.VelKp;
	Dr = AltAccComp = Limit1(AccZ * Alt.VelKd, 0.2f);

	AltComp = Limit1(Pr + Dr, MaxAltHoldCompFrac);

} // DoROCControl


void AltitudeHold(real32 MinROCMPS, real32 MaxROCMPS) {
	real32 Pa, Ia, AltE, S, Windup, ROCLimit;

	AltE = Limit1(Alt.PosE, ALT_HOLD_BAND_M);

	Pa = Limit(AltE * Alt.PosKp, MinROCMPS, MaxROCMPS);

	Alt.PosIntE += AltE * Alt.PosKi * AltdT;
	Ia = Alt.PosIntE;

	DesiredROC = Pa + Ia;

	ROCLimit = DesiredROC > 0.0f ? MaxROCMPS : -MinROCMPS;
	Windup = Abs(DesiredROC) - ROCLimit;
	if (Windup > 0.0f) {
		S = Sign(DesiredROC);
		Alt.PosIntE = S * (Abs(Alt.PosIntE) - Windup);
		DesiredROC = S * ROCLimit;
	}

	DoROCControl(DesiredROC, MinROCMPS, MaxROCMPS);

} // AltitudeHold

//______________________________________________________________________________

// Fixed Wing

void DeploySpoilers(real32 a) {

	Sl = Limit(a * FWAltSpoilerFFFrac/ALT_HOLD_BAND_M, 0.0, FWAltSpoilerFFFrac);
	if (Sl > 0.0f)
		AltComp = -1.0f;

} // DeploySpoilers

void AcquireAltitudeFW(void) {

	Alt.PosE = DesiredAltitude - Altitude; // negative then too high => descend

	CheckRapidDescentHazard();
	if (F.RapidDescentHazard)
		DeploySpoilers(-Alt.PosE - ALT_HOLD_BAND_M);
	else {
		AltitudeHold(-BestROCMPS, BestROCMPS);
		Sl = DecayX(Sl, FWSpoilerDecayS, AltdT);
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
		if (((NavState != HoldingStation) && (NavState != PIC))) { // Navigating - using CruiseThrottle
			F.HoldingAlt = true;
			AcquireAltitudeFW();
		} else {
			CheckThrottleMoved();
			if (F.ThrottleMoving || (ROCTooHigh(1.0f) && !F.HoldingAlt)) {
				F.HoldingAlt = false;
				DesiredAltitude = Altitude;
				Alt.PosIntE = Sl = 0.0f;
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
	// Synchronised to baro intervals independent of active altitude source
	real32 EffMinROCMPS;

	Alt.PosE = DesiredAltitude - Altitude;

	EffMinROCMPS = (F.RapidDescentHazard || (NavState == ReturningHome)
			|| (NavState == Transiting)) ? DESCENT_MIN_ROC_MPS : MinROCMPS;

	AltitudeHold(EffMinROCMPS, ALT_MAX_ROC_MPS);

} // AcquireAltitude


void AltitudeControl(void) {

	if (((NavState != HoldingStation) && (NavState != PIC))) { // Navigating - using CruiseThrottle
		F.HoldingAlt = true;
		AcquireAltitude();
	} else {
		CheckThrottleMoved();
		if (F.ThrottleMoving || (ROCTooHigh(0.25f) && !F.HoldingAlt)) {
			F.HoldingAlt = false;
			DesiredAltitude = Altitude;
			Alt.PosIntE = 0.0f;
			AltComp = DecayX(AltComp, AltCompDecayS, AltdT);
		} else {
			F.HoldingAlt = true;
			TrackCruiseThrottle();
			AcquireAltitude(); // using Stick Throttle NOT cruise throttle
		}
	}
} // AltitudeControl


void DoAltitudeControl(void) {

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
			DesiredAltitude = Altitude;
			AltComp = DecayX(AltComp, AltCompDecayS, AltdT);
			Sl = DecayX(Sl, FWSpoilerDecayS, AltdT);
			F.HoldingAlt = false;
		}

	}

} // DoAltitudeControl


real32 ComputeRateDerivative(AxisStruct *C) {

	C->RateD = PavelDifferentiator(&C->RateDF, C->RateE) * dTR;
	C->RateD = LPFilter(&C->RateF, DerivativeLPFOrder, C->RateD,
			DerivativeLPFreqHz, dT);

	return (C->RateD);

} // ComputeRateDerivative


void ZeroPIDIntegrals(void) {
	idx a;

	for (a = Pitch; a <= Yaw; a++)
		A[a].AngleIntE = 0.0f;

} // ZeroPIDIntegrals


void DoRateControl(idx a) {
	AxisStruct *C;

	C = &A[a];

	real32 AngleRateMix;
	real32 Pa, Pr, Dr;

	C->Control = Threshold(C->Stick, StickDeadZone);

	if (P(Horizon) > 0) { // hybrid that panics back to angle when sticks centred

		C->AngleDesired = C->Control * C->AngleMax;
		if (IsFixedWing && (a == Pitch))
			C->AngleDesired += FWBoardPitchAngleRad;

		C->AngleDesired = Limit1(C->AngleDesired, C->AngleMax);

		C->AngleE = C->AngleDesired - C->Angle;
		Pa = C->AngleE * C->AngleKp;

		C->AngleIntE = 0.0f; // for flip back to angle mode

		AngleRateMix
				= Limit(1.0f - (CurrMaxRollPitchStick / HorizonTransPoint), 0.0f, 1.0f);

		C->RateDesired = Pa * AngleRateMix + C->Control * C->RateMax * (1.0f
				- AngleRateMix);

	} else
		// pure rate control
		C->RateDesired = C->Control * C->RateMax;

	C->RateE = Rate[a] - C->RateDesired;

	Pr = C->RateE * C->RateKp;
	Dr = ComputeRateDerivative(C) * C->RateKd;

	C->Out = conditionOut(Pr + Dr);

} // DoRateControl


void DoAngleControl(idx a) { // with Ming Liu
	real32 Pa, Ia, Pr, Dr, S, Windup;
	AxisStruct *C;

	C = &A[a];

	C->AngleDesired = C->Control = Threshold(C->Stick, StickDeadZone)
			* C->AngleMax;

	C->AngleDesired += C->NavCorr;

	if (IsFixedWing && (a == Pitch))
		C->AngleDesired += FWBoardPitchAngleRad;

	C->AngleDesired = Limit1(C->AngleDesired, C->AngleMax);

	C->AngleE = C->AngleDesired - C->Angle;
	Pa = C->AngleE * C->AngleKp;

	C->AngleIntE += C->AngleE * C->AngleKi * dT;

	Ia = C->AngleIntE;
	C->RateDesired = Pa + Ia;

	Windup = Abs(C->RateDesired) - C->RateMax;
	if (Windup > 0.0f) {
		S = Sign(C->RateDesired);
		C->AngleIntE = S * (Abs(C->AngleIntE) - Windup);
		C->RateDesired = S * C->RateMax;
	}

	C->RateE = Rate[a] - C->RateDesired;

	Pr = C->RateE * C->RateKp;
	Dr = ComputeRateDerivative(C) * C->RateKd;

	C->Out = conditionOut(Pr + Dr);

} // DoAngleControl


real32 DesiredYawRate(void) {

	F.YawActive = ((IsFixedWing && ((NavState == PIC) || (Abs(A[Roll].Stick)
			> StickDeadZone))) || (Abs(A[Yaw].Stick) > StickDeadZone));

	if (F.YawActive) {
		DesiredHeading = Heading;
		A[Yaw].AngleE = 0.0f;// for log
		A[Yaw].Control = Threshold(A[Yaw].Stick, StickDeadZone)
				* A[Yaw].RateMax;
		return (A[Yaw].Control);
	} else {
		if ((F.Navigate || F.ReturnHome) && !IsFixedWing)
			DesiredHeading = Nav.DesiredHeading;

		A[Yaw].AngleE
				= Limit1(MinimumTurn(DesiredHeading), Nav.HeadingTurnoutRad);
		return (Limit1(A[Yaw].AngleE * A[Yaw].AngleKp, A[Yaw].CompassRateMax));
	}
} // DesiredYawRate


static void DoTurnControl(void) {
	real32 Pr, Dr;

	A[Yaw].RateDesired = DesiredYawRate();

	A[Yaw].RateE = A[Yaw].RateDesired - Rate[Yaw];
	Pr = A[Yaw].RateE * A[Yaw].RateKp;
	Dr = ComputeRateDerivative(&A[Yaw]) * A[Yaw].RateKd;

	A[Yaw].Out = Pr + Dr;

	if (IsFixedWing) {
		A[Yaw].Out *= FWAileronRudderFFFrac;
		A[Roll].NavCorr
				= Limit1(A[Yaw].RateE * Airspeed * GRAVITY_MPS_S_R, Nav.MaxAngle)
						* Nav.Sensitivity;
	}
} // DoTurnControl


void DoControl(void) {
	idx a;

	CurrMaxTiltAngle = Max(Abs(A[Roll].Angle), Abs(A[Pitch].Angle));
	F.NearLevel = CurrMaxTiltAngle < NAV_RTH_LOCKOUT_ANGLE_RAD;

	//CalcTiltThrFFComp();
	//CalcBattThrComp();

#if defined(BARE_METAL)
	for (a = Pitch; a <= Yaw; a++) {
		A[a].RateDesired = A[a].StickD * A[a].RateMax * 200.0f;
		A[a].Out = conditionOut(A[a].RateDesired * A[a].RateKp);
	}
#else

	determineGainScale(); // primitive gain scheduling

	DoTurnControl(); // MUST BE BEFORE ROLL CONTROL

	for (a = Pitch; a <= Roll; a++)
		if (F.UsingRateControl)
			DoRateControl(a);
		else
			DoAngleControl(a);

#endif

	// move to start of cycle to reduce jitter UpdateDrives();

} // DoControl


void InitControl(void) {
	idx a;
	AxisStruct *C;

	A[Roll].AngleE = Sl = 0.0f;
	ZeroThrottleCompensation();

	for (a = Pitch; a <= Yaw; a++) {
		C = &A[a];
		C->AngleIntE = 0.0f;
		C->Ratep = 0.0f;
		C->RateDp = 0.0f;

		C->NavCorr = 0.0f;
		C->RateF.Primed = false;
		C->RateDF.Primed = false;

		C->StickP = C->StickD = 0.0f;

		C->Out = 0.0f;
	}

	Acc[Z] = -GRAVITY_MPS_S;

} // InitControl


