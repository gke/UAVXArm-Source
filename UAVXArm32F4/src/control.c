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
real32 TiltThrFF;

real32 CurrAccLPFHz = 20;
real32 CurrGyroLPFHz = 100;
real32 CurrDerivativeLPFHz = 75;
real32 CurrYawLPFHz = 20;
boolean UsingPavelFilter;

idx AttitudeMode = AngleMode;
idx DerivativeLPFOrder;
real32 DesiredHeading, SavedHeading;
real32 Altitude;
real32 AltComp, ROC, MinROCMPS, EffMinROCMPS;
real32 AltCompDecayS;
real32 TiltThrFFComp = 1.0f;
real32 BattThrFFComp = 1.0f;
real32 AltAccComp = 0.0f;
real32 HorizonTransScale;
real32 AngleRateMix = 1.0f;
real32 StickDeadZone;
real32 CruiseThrottle = 0.5f;
real32 CurrMaxTiltAngle = 0.0f;
real32 TuneKpScale;
int8 BeepTick = 0;

AltStruct Alt;

real32 FWRollPitchFFFrac, FWAileronDifferentialFrac, FWPitchThrottleFFFrac,
		MaxAltHoldCompFrac, FWMaxClimbAngleRad, FWBoardPitchAngleRad,
		FWClimbThrottleFrac, FWSpoilerDecayS, FWAileronRudderFFFrac,
		FWAltSpoilerFFFrac, BestROCMPS;
real32 FWGlideAngleOffsetRad = 0.0f;
real32 ThrottleGain, gainScale;

real32 ComputeAttitudeRateDerivative(PIDStruct *C) {
	// Using "rate on measurement" to avoid "derivative kick"
	real32 r;

	if (UsingPavelFilter) {
		C->RateD = PavelDifferentiator(&C->RateDF, C->Error) * dTR;
		if (P(DerivativeLPFHz) > 0)
			C->RateD
					= LPFilter(&C->RateF, 1, C->RateD, CurrDerivativeLPFHz, dT);
	} else {
		r = (P(DerivativeLPFHz)) > 0 ? LPFilter(&C->RateF, 1, C->Error,
				CurrDerivativeLPFHz, dT) : C->Error;
		C->RateD = (r - C->RateP) * dTR;
		C->RateP = r;
		C->RateD = Smoothr32xn(&C->RateDF, 4, C->RateD);
	}

	return (C->RateD);

} // ComputeRateDerivative

real32 DoPID(PIDStruct * C, real32 Current, real32 dT) {
	// do most general case - slightly more expensive

	C->Error = Limit1(C->Desired - Current, C->Max);

	C->PTerm = C->Error * C->Kp;

	if (C->Ki > 0.0f) {
		C->IntE += C->Error * C->Ki * dT;
		C->IntE = Limit1(C->IntE, C->IntLim);
	} else
		C->IntE = 0.0f; // redundant

	C->ITerm = C->IntE;

	C->DTerm = (C->Kd > 0.0f) ? ComputeAttitudeRateDerivative(C) * C->Kd : 0.0f;

	return (C->PTerm + C->ITerm + C->DTerm);

} // DoPID


void ZeroIntegrators(void) {
	idx a;

	for (a = Pitch; a <= Yaw; a++)
		A[a].P.IntE = A[a].R.IntE = 0.0f;

} // ZeroIntegrators

//______________________________________________________________________________


void determineGainScale(void) {

	gainScale = ((P(ThrottleGainRate) > 0)
			&& (DesiredThrottle > CruiseThrottle)) ? 1.0f - ThrottleGain
			* (DesiredThrottle - CruiseThrottle) / (1.0f - CruiseThrottle)
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

	if (IsMulticopter && (State == InFlight) && F.UsingAngleControl) { // forget near level check
		Temp = (1.0f / AttitudeCosine() - 1.0) * TiltThrFFFrac + 1.0f;
		Temp = Limit(Temp, 1.0f, TiltFFLimit);
		TiltThrFFComp = SlewLimit(&TiltThrFFComp, Temp, TiltFFLimit, dT);
	} else
		TiltThrFFComp = 1.0f;

} // CalcTiltThrFFComp


void CalcBattThrComp(void) {

	BattThrFFComp
			= IsMulticopter && (State == InFlight) && F.UsingAngleControl ? BatterySagR
					: 1.0f;

} // CalcBattThrComp

//______________________________________________________________________________


void TrackCruiseThrottle(void) {

	if (F.Emulation)
		CruiseThrottle = F.IsFixedWing ? THR_DEFAULT_CRUISE_FW_STICK
				: THR_DEFAULT_CRUISE_STICK;
	else {
		if ((Abs(ROCF) < ALT_HOLD_MAX_ROC_MPS) && (DesiredThrottle
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

	Alt.R.Error = Limit(DesiredROC - ROC, MinROCMPS, MaxROCMPS);

	Alt.R.PTerm = Alt.R.Error * Alt.R.Kp;
	Alt.R.DTerm = AltAccComp = Limit1(AccZ * Alt.R.Kd, 0.2f);

	AltComp = Limit1(Alt.R.PTerm + Alt.R.DTerm, MaxAltHoldCompFrac);

} // DoROCControl


void AltitudeHold(real32 MinROCMPS, real32 MaxROCMPS) {

	Alt.R.Desired = DoPID(&Alt.R, Altitude, AltdT);

	DoROCControl(Alt.R.Desired, MinROCMPS, MaxROCMPS);

} // AltitudeHold

//______________________________________________________________________________

// Fixed Wing

void DeploySpoilers(real32 a) {

	Sl = Limit(a * FWAltSpoilerFFFrac/ALT_HOLD_BAND_M, 0.0, FWAltSpoilerFFFrac);
	if (Sl > 0.0f)
		AltComp = -1.0f;

} // DeploySpoilers

void AcquireAltitudeFW(void) {

	CheckRapidDescentHazard();
	if (F.RapidDescentHazard)
		DeploySpoilers((Altitude - Alt.P.Desired) - ALT_HOLD_BAND_M);
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
		if ((State == Launching) || ((NavState != HoldingStation) && (NavState
				!= PIC))) { // Navigating - using CruiseThrottle
			F.HoldingAlt = true;
			AcquireAltitudeFW();
		} else {
			CheckThrottleMoved();
			if (F.ThrottleMoving || (ROCTooHigh(1.0f) && !F.HoldingAlt)) {
				F.HoldingAlt = false;
				SetDesiredAltitude(Altitude);
				Alt.P.IntE = Sl = 0.0f;
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

	EffMinROCMPS = (F.RapidDescentHazard || (NavState == ReturningHome)
			|| (NavState == Transiting)) ? DESCENT_MIN_ROC_MPS : MinROCMPS;

	AltitudeHold(EffMinROCMPS, ALT_MAX_ROC_MPS);

} // AcquireAltitude


void AltitudeControl(void) {

	if ((State == Launching) || ((NavState != HoldingStation) && (NavState
			!= PIC))) { // Navigating - using CruiseThrottle
		F.HoldingAlt = true;
		AcquireAltitude();
	} else {
		CheckThrottleMoved();
		if (F.ThrottleMoving || (ROCTooHigh(0.25f) && !F.HoldingAlt)) {
			F.HoldingAlt = false;
			SetDesiredAltitude(Altitude);
			Alt.P.IntE = 0.0f;
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

		if (F.IsFixedWing)
			UpdateVario();

		if (F.AltControlEnabled) {
			if (F.IsFixedWing)
				AltitudeControlFW();
			else
				AltitudeControl();
		} else {
			//zzz check
			F.RapidDescentHazard = ROC < DESCENT_MIN_ROC_MPS;
			SetDesiredAltitude(Altitude);
			AltComp = DecayX(AltComp, AltCompDecayS, AltdT);
			Sl = DecayX(Sl, FWSpoilerDecayS, AltdT);
			F.HoldingAlt = false;
		}
	}

} // DoAltitudeControl


void DoRateDampingControl(idx a) {
	AxisStruct *C = &A[a];
	real32 Stick;

	Stick = Threshold(C->Stick, StickDeadZone);

	C->R.Error = -Limit1(Rate[a], C->R.Max);

	C->R.PTerm = C->R.Error * C->R.Kp * Limit(1.0f - Abs(Stick) * HorizonTransScale, 0.0f, 1.0f);

	C->Out = -conditionOut(C->R.PTerm + Stick * C->R.Max * C->R.Kp);

} // DoRateDampingControl


void DoRateControl(idx a) {
	AxisStruct *C = &A[a];

	C->R.Desired = Threshold(C->Stick, StickDeadZone) * C->P.Max;

	C->Out = -conditionOut(DoPID(&C->R, Rate[a], dT));

} // DoRateControl


void DoAngleControl(idx a) { // with Ming Liu
	AxisStruct *C = &A[a];

	C->P.Desired = Threshold(C->Stick, StickDeadZone) * C->P.Max + C->NavCorr;

	if (F.IsFixedWing && (a == Pitch))
		C->P.Desired += FWBoardPitchAngleRad;

	C->R.Desired = DoPID(&C->P, C->Angle, dT);
	C->Out = -conditionOut(DoPID(&C->R, Rate[a], dT));

} // DoAngleControl


void DoHorizonControl(idx a) {
	real32 AngleRateMix;
	AxisStruct *C = &A[a];

	C->P.Desired = Threshold(C->Stick, StickDeadZone) * C->P.Max;
	if (F.IsFixedWing && (a == Pitch))
		C->P.Desired += FWBoardPitchAngleRad;

	C->P.Desired = Limit1(C->P.Desired, C->P.Max);
	C->P.Error = C->P.Desired - C->Angle;
	C->P.PTerm = C->P.Error * C->P.Kp;

	C->P.IntE = 0.0f; // for flip back to angle mode

	AngleRateMix
			= Limit(1.0f - (CurrMaxRollPitchStick * HorizonTransScale), 0.0f, 1.0f);

	C->R.Desired = C->P.PTerm * AngleRateMix + C->P.Desired * C->P.Max * (1.0f
			- AngleRateMix);

	C->Out = -conditionOut(DoPID(&C->R, Rate[a], dT));

} // DoHorizonControl


real32 DesiredYawRate(void) {
	AxisStruct *C = &A[Yaw];

	F.YawActive = F.Bypass || (!F.ValidHeading) || (Abs(C->Stick)
			> StickDeadZone) || (F.IsFixedWing && (Abs(A[Roll].Stick)
			> StickDeadZone));

	if (F.YawActive) {
		DesiredHeading = SavedHeading = Heading;
		C->P.Error = C->P.IntE = C->R.IntE = 0.0f;
		C->R.Desired = Threshold(C->Stick, StickDeadZone) * C->R.Max;

	} else {
		if ((F.Navigate || F.ReturnHome) && F.IsFixedWing)
			DesiredHeading = Nav.DesiredHeading;

		C->P.Error = Limit1(MinimumTurn(DesiredHeading), C->P.Max);

		C->R.Desired = Limit1(C->P.Error * C->P.Kp, Nav.MaxCompassRate); // redundant limit
	}

	F.ValidHeading = false;

	return (C->R.Desired);

} // DesiredYawRate


static void DoTurnControl(void) {
	AxisStruct *C = &A[Yaw];

	C->R.Desired = DesiredYawRate();

	C->Out = DoPID(&C->R, Rate[Yaw], dT);

	if (F.IsFixedWing) {
		A[Roll].NavCorr
				= F.Bypass ? 0.0f
						: Limit1(C->R.Error * Airspeed * GRAVITY_MPS_S_R, Nav.MaxBankAngle)
								* Nav.Sensitivity;
		C->Out *= FWAileronRudderFFFrac;
	}

	C->Out = conditionOut(C->Out);

} // DoTurnControl


void DoControl(void) {
	idx a;

	CurrMaxTiltAngle = Max(Abs(A[Roll].Angle), Abs(A[Pitch].Angle));
	F.NearLevel = CurrMaxTiltAngle < NAV_RTH_LOCKOUT_ANGLE_RAD;

	//CalcTiltThrFFComp();
	//CalcBattThrComp();

	determineGainScale(); // primitive gain scheduling

	DoTurnControl(); // MUST BE BEFORE ROLL CONTROL


	for (a = Pitch; a <= Roll; a++)
		if (F.IsFixedWing)
			switch (AttitudeMode) {
			case AngleMode:
				DoAngleControl(a);
				break;
			case HorizonMode:
			case RateMode:
				DoRateDampingControl(a);
				break;
			} // switch
		else
			switch (AttitudeMode) {
			case AngleMode:
				DoAngleControl(a);
				break;
			case HorizonMode:
				DoHorizonControl(a);
				break;
			case RateMode:
				DoRateControl(a);
				break;
			} // switch

	// moved UpdateDrives() to start of cycle to reduce jitter

} // DoControl


void InitControl(void) {
	PIDStruct * C;
	idx a;

	Sl = 0.0f;
	ZeroThrottleCompensation();

	for (a = Pitch; a <= Yaw; a++) {
		C = &A[a].P;
		C->IntE = 0.0f;
		C->RateF.Primed = C->RateDF.Primed = false;
		C = &A[a].R;
		C->IntE = 0.0f;
		C->RateF.Primed = C->RateDF.Primed = false;

		A[a].NavCorr = 0.0f;
		A[a].Out = 0.0f;
	}

	Acc[Z] = -GRAVITY_MPS_S;

} // InitControl


