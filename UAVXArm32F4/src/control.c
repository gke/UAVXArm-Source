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

idx AttitudeMode = AngleMode;
real32 DesiredHeading, SavedHeading;
real32 Altitude;
real32 AltHoldThrComp, ROC, MinROCMPS, MaxROCMPS, EffMinROCMPS;

real32 BattThrFFComp = 1.0f;
real32 TiltThrFFFrac = 0.0f;
real32 TiltThrFFComp = 1.0f;
real32 HorizonTransScale;
real32 AngleRateMix = 1.0f;
real32 StickDeadZone;
real32 CruiseThrottle = 0.5f;
real32 AltitudeHoldROCWindow = ALT_HOLD_MAX_ROC_MPS;
real32 CurrMaxTiltAngle = 0.0f;
real32 MaxAltHoldThrComp = 0.05f;
real32 TuneKpScale;
int8 BeepTick = 0;
real32 DesiredThrottle, IdleThrottle, InitialThrottle;
real32 FWRollPitchFFFrac, FWAileronDifferentialFrac, FWPitchThrottleFFFrac,
		FWMaxClimbAngleRad, FWBoardPitchAngleRad, FWClimbThrottleFrac,
		FWSpoilerDecayPS, FWAileronRudderFFFrac, FWAltSpoilerFFFrac,
		VRSDescentRateMPS, FWRollControlPitchLimitRad,
		AltHoldThrCompDecayPS;
real32 FWGlideAngleOffsetRad = 0.0f;
real32 ThrottleGain, AttitudeGainScale;
real32 MaxAttitudeGainReduction = 0.0f;

boolean AltHoldAlarmActive = false;

void DoSetPointSlews(void) {
	idx a;

    //Alt.P.Desired = SlewLimit(Alt.P.Desired, DesiredAlt, Alt.P.Max, dT);
	//Alt.P.Desired = DesiredAlt;

	for (a = Pitch; a <= Roll; a++)
		A[a].NavCorr = SlewLimit(A[a].NavCorr, A[a].DesiredNavCorr, A[a].R.Max,
				dT);

} // DoSetPointSlews

void ResetHeading(void) {
	SavedHeading = DesiredHeading = Nav.TakeoffBearing = Nav.DesiredHeading =
			Heading;
} // SetDesiredHeading;

void CalcTiltThrFF(void) {
	const real32 TiltScaleLimit = 1.0f / cosf(NAV_MAX_ANGLE_RAD);
	real32 Temp;

	if (IsMulticopter && (State == InFlight)) { // forget near level check
		Temp = (1.0f / AttitudeCosine() - 1.0) * TiltThrFFFrac + 1.0f;
		Temp = Limit(Temp, 1.0f, TiltScaleLimit);
		TiltThrFFComp = SlewLimit(TiltThrFFComp, Temp, TiltScaleLimit, dT);
	} else
		TiltThrFFComp = 1.0f;

} // CalcTiltThrFF

void DisableFlightStuff(void) { // paranoid ;)

	F.HoldingAlt = false;
	DesiredThrottle = 0.0f;
	AltHoldThrComp = 0.0f;
	ZeroIntegrators();
	ZeroNavCorrections();
	ResetHeading();

} // DisableFlightStuff

void CheckAltHoldAlarm(void) {

	AltHoldAlarmActive = F.UsingAltHoldAlarm && NotDescending();

} // CheckAltHoldAlarm

void DetermineInFlightThrottle(void) {

	if (F.PassThru)
		DesiredThrottle = StickThrottle;
	else {
		if (F.ForcedLanding) { // override everything!
			F.AltControlEnabled = F.HoldingAlt = true;
			DesiredThrottle = CruiseThrottle;
		} else {
			if (F.NavigationEnabled) {
				if (NavState == HoldingStation)
					DesiredThrottle = StickThrottle;
				else {
					if ((NavState == Perching) || (NavState == Touchdown))
						DisableFlightStuff(); // suppresses alt. hold etc.
					else
						DesiredThrottle = CruiseThrottle;
				}
			} else
				DesiredThrottle = StickThrottle;
		}
	}
} // DetermineDesiredThrottle

//______________________________________________________________________________

// Altitude

void TrackCruiseThrottle(void) {
#define CRUISE_TRACKING_RATE FromPercent(0.5f) // percent per second

	F.Hovering = (Abs(ROCTrack) < AltitudeHoldROCWindow)
			&& (DesiredThrottle > THR_MIN_ALT_HOLD_STICK);

	if (!F.Emulation) {
		if (F.Hovering) {
			CruiseThrottle = SlewLimit(CruiseThrottle,
					DesiredThrottle + AltHoldThrComp,
					CRUISE_TRACKING_RATE, AltdT);

			CruiseThrottle = Limit(CruiseThrottle, THR_MIN_ALT_HOLD_STICK,
					THR_MAX_ALT_HOLD_STICK);
			SetP(EstCruiseThr, CruiseThrottle * 100.0f);
		}
	}

} // TrackCruiseThrottle

void DoROCControl(void) {

	Alt.R.Error = Alt.R.Desired - ROC;

	Alt.R.PTerm = Alt.R.Error * Alt.R.Kp;

	Alt.R.IntE += (Alt.R.Error * Alt.R.Ki * AltdT);
	Alt.R.IntE = Limit1(Alt.R.IntE, Alt.R.IntLim);
	Alt.R.ITerm = Alt.R.IntE;

	AltHoldThrComp = Limit1(Alt.R.PTerm + Alt.R.ITerm, MaxAltHoldThrComp);

	CheckAltHoldAlarm();

} // DoROCControl

void DoAltitudeControl(real32 CurrMinROCMPS, real32 CurrMaxROCMPS) {

	Alt.P.Error = Alt.P.Desired - Altitude;

	Alt.P.PTerm = Alt.P.Error * Alt.P.Kp;
	Alt.P.IntE += (Alt.P.Error * Alt.P.Ki * AltdT);
	Alt.P.IntE = Limit1(Alt.P.IntE, Alt.P.IntLim);
	Alt.P.ITerm = Alt.P.IntE;

	Alt.R.Desired = Limit(Alt.P.PTerm + Alt.P.ITerm, CurrMinROCMPS,
			CurrMaxROCMPS);

	DoROCControl();

} // DoAltitudeControl

//______________________________________________________________________________

// ChatGBT 3.5 20231216


// Constants
//#define HOVER_THROTTLE 0.5  // Hover throttle value
#define MAX_THRUST 2.0      // Maximum thrust in Kg
#define MAX_INTEGRAL 0.1    // Maximum value for the integral term


// Function to compute throttle values for climbing
real32 DoAltitudeControlGBT(real32 CurrMinROCMPS, real32 CurrMaxROCMPS) {
    // Altitude control parameters

	real32 Mass = 2.0f;

    // Proportional control: directly use the altitude error
    Alt.P.Error = Alt.P.Desired - Altitude;
    Alt.P.PTerm = Alt.P.Error * Alt.P.Kp;

    // Integral control with anti-windup
    if (Alt.P.Error == 0.0) {
        // Reset integral term when there is no altitude error
        Alt.P.IntE = 0.0;
    } else {
        Alt.P.IntE += (Alt.P.Error * Alt.P.Ki * AltdT);
        Alt.P.IntE = Limit(Alt.P.IntE, -Alt.P.IntLim, Alt.P.IntLim);
    }
    Alt.P.ITerm = Alt.P.IntE;

    // Desired vertical velocity directly limited by MinROCMPS and MaxROCMPS
    real32 desiredVerticalVelocity = Limit(Alt.P.PTerm + Alt.P.ITerm, CurrMinROCMPS, CurrMaxROCMPS);

    // Convert desired vertical velocity to acceleration (considering mass)
    real32 desiredAcc = (desiredVerticalVelocity - ROC) / AltdT;

    // Convert acceleration to thrust
    real32 thrust = Mass * (GRAVITY_MPS_S + desiredAcc);

    // Convert thrust to throttle (assuming linear relationship between thrust and throttle)
    real32 AltHoldThrComp = thrust / (Mass * GRAVITY_MPS_S);

    // Ensure throttle is within bounds
    AltHoldThrComp = Limit(AltHoldThrComp, 0.0, 1.0);

}


//______________________________________________________________________________

// Fixed Wing

void DeploySpoilers(real32 a) {
	real32 NewSl;

	NewSl = Limit(FWAltSpoilerFFFrac * (a / Alt.P.Max), 0.0,
			FWAltSpoilerFFFrac);
	Sl = SlewLimit(Sl, NewSl, FromPercent(20.0f), AltdT);
	if (Sl > 0.0f)
		AltHoldThrComp = DecayX(AltHoldThrComp, AltHoldThrCompDecayPS, AltdT);

} // DeploySpoilers

void AcquireAltitudeFW(void) {

	CheckRapidDescentHazard();
	if (F.RapidDescentHazard)
		DeploySpoilers((Altitude - Alt.P.Desired) - Alt.P.Max);
	else {
		DoAltitudeControl(-Alt.R.Max, Alt.R.Max);
		Sl = DecayX(Sl, FWSpoilerDecayPS, AltdT);
	}

} // AcquireAltitudeFW

boolean ROCTooHigh(real32 Window) {

	return UsingDCMotors ? false : (Abs(ROCTrack) > Window);

} // ROCTooHigh

void ControllingAltitudeFW(void) {
	real32 t;

	if (F.ForcedLanding) {
		F.HoldingAlt = true;
		AcquireAltitudeFW();
	} else {
		if (F.Glide && F.NavigationEnabled) {
			if (NavState == BoostClimb) {
				F.HoldingAlt = true;
				Sl = 0.0f;
				Alt.R.Desired = Alt.R.Max;
				DoROCControl();
			} else {
				t = Altitude - AltMaxM;
				//Sl = (NavState == AltitudeLimiting) ? Limit(Abs(t) * 0.05f, 0.0f, 1.0f)
				//				: 0.0f; // TODO: slew
				DeploySpoilers(Abs(t));
				F.HoldingAlt = AltHoldAlarmActive = false;
				AltHoldThrComp = DecayX(AltHoldThrComp, AltHoldThrCompDecayPS, AltdT);
			}
		} else {
			if (F.NavigationEnabled) { // Navigating - using CruiseThrottle
				F.HoldingAlt = true;
				AcquireAltitudeFW();
			} else {
				if ((F.HoldingAlt) && !F.ThrottleMoving) {
					TrackCruiseThrottle();
					AcquireAltitudeFW();
				} else {
					SetDesiredAltitude(Altitude); // just track altitude
					F.HoldingAlt = !ROCTooHigh(1.0f);
				}
			}
		}
	}
} // ControllingAltitudeFW

//______________________________________________________________________________

void AcquireAltitude(void) {
	// Synchronised to baro intervals independent of active altitude source
	real32 EffMinROCMPS;

	EffMinROCMPS =
			(F.RapidDescentHazard || (NavState == Transiting)) ?
					VRSDescentRateMPS : MinROCMPS;

	DoAltitudeControl(EffMinROCMPS, Alt.R.Max);

} // AcquireAltitude

void ControllingAltitude(void) {
	real32 t;

	if (F.ForcedLanding || F.NavigationEnabled) { // autonomous
		F.HoldingAlt = true;
		AcquireAltitude();
	} else {
		if ((F.HoldingAlt) && !F.ThrottleMoving) {
			TrackCruiseThrottle();
			AcquireAltitudeFW();
		} else {
			SetDesiredAltitude(Altitude); // just track altitude
			F.HoldingAlt = !(F.ThrottleMoving || ROCTooHigh(0.25f));

			if (!F.HoldingAlt)
				AltHoldThrComp = DecayX(AltHoldThrComp, AltHoldThrCompDecayPS, AltdT);
		}
	}

} // ControllingAltitude

void ControlAltitude(void) {

	if (F.NewAltitudeValue) { // every 
		F.NewAltitudeValue = false;

		if (F.IsFixedWing)
			UpdateVario();

		if (F.AltControlEnabled) {
			if (F.IsFixedWing)
				ControllingAltitudeFW();
			else
				ControllingAltitude();
		} else {
			F.RapidDescentHazard = ROC < VRSDescentRateMPS;
			SetDesiredAltitude(Altitude);
			AltHoldThrComp = DecayX(AltHoldThrComp, AltHoldThrCompDecayPS, AltdT);
			Sl = DecayX(Sl, FWSpoilerDecayPS, AltdT);
			F.HoldingAlt = false;
		}
	}

} // ControlAltitude

real32 ComputeAttitudeRateDerivative(PIDStruct *R) {
	// Using "rate on measurement" to avoid "derivative kick"
	real32 r;

	if (UsingPavelFilter) {
		R->RateD = FIRF(&R->RateDF, R->Error) * dTR;
		R->RateD = LPFn(&R->RateF, R->RateD, dT);
	} else {
		r = LPFn(&R->RateF, R->Error, dT);
		R->RateD = (r - R->RateP) * dTR;
		R->RateP = r;
		R->RateD = MAF(&R->RateDF, R->RateD);
	}

	/*
	 BROKEN
	 real32 wCut = TWO_PI * CurrGyroLPFHz * 0.6f;
	 R->RateD = R->Error * wCut;
	 R->RateI += R->RateD * dT / wCut;

	 */

	return (R->RateD);

} // ComputeRateDerivative

//______________________________________________________________________________

// Attitude

void ZeroIntegrators(void) {
	idx a;

	for (a = Pitch; a <= Yaw; a++)
		A[a].P.IntE = A[a].R.IntE = 0.0f;

} // ZeroIntegrators

void ConditionIntE(PIStruct *P, int8 * S, real32 dT) {

	// tentative scheme - need to check around 180
	if (Sign(P->Error) == *S) {
		P->IntE += (P->Error * P->Ki * dT);
		P->IntE = Limit1(P->IntE, P->IntLim);
	} else {
		P->IntE = 0.0f;
		*S = Sign(P->Error);
	}
} // ConditionIntE

void DoAttitudeGainScale(void) {

	AttitudeGainScale =
			((P(ThrottleGainRate) > 0) && (DesiredThrottle > CruiseThrottle)) ?
					1.0f
							- MaxAttitudeGainReduction
									* (DesiredThrottle - CruiseThrottle)
									/ (1.0f - CruiseThrottle) :
					1.0f;

} // DoAttitudeGainScale

real32 conditionOut(real32 v) {

	return (Limit1(v * AttitudeGainScale, 1.0f));

} // conditionOut

void ControlRate(idx a) {
	PIDStruct *R = &A[a].R;

	R->Error = Limit1(R->Desired, R->Max) - Rate[a];

	R->PTerm = R->Error * R->Kp;
	R->DTerm = ComputeAttitudeRateDerivative(R) * R->Kd;

	A[a].Out = -conditionOut(R->PTerm + R->DTerm);

} // ControlRate

void DoRateControl(idx a) {

	A[a].R.Desired = Threshold(A[a].Stick, StickDeadZone) * A[a].R.Max;

	ControlRate(a);

} // DoRateControl

void DoRateDampingControl(idx a) {
	PIDStruct *R = &A[a].R;
	real32 Stick;

	Stick = Threshold(A[a].Stick, StickDeadZone);

	R->Error = -Rate[a];
	R->PTerm = R->Error * R->Kp
			* Limit(1.0f - Abs(Stick) * HorizonTransScale, 0.0f, 1.0f);

	A[a].Out = -conditionOut(R->PTerm + Stick * R->Max * R->Kp);

} // DoRateDampingControl

void DoAngleControl(idx a) {
	real32 AngleRateMix;
	PIStruct *P = &A[a].P;

	P->Desired = Threshold(A[a].Stick, StickDeadZone) * P->Max + A[a].NavCorr;
	if (F.IsFixedWing && (a == Pitch))
		P->Desired += FWBoardPitchAngleRad;

	P->Error = Limit1(P->Desired, P->Max) - Angle[a];
	P->PTerm = P->Error * P->Kp;

	if (AttitudeMode == HorizonMode) {
		P->IntE = 0.0f; // for flip back to angle mode
		AngleRateMix = Limit(1.0f - (CurrMaxRollPitchStick * HorizonTransScale),
				0.0f, 1.0f);
		A[a].R.Desired = P->PTerm * AngleRateMix
				+ P->Desired * P->Max * (1.0f - AngleRateMix);
	} else {
		P->IntE += (P->Error * P->Ki * dT);
		P->IntE = Limit1(P->IntE, P->IntLim);
		P->ITerm = P->IntE;

		A[a].R.Desired = P->PTerm + P->ITerm;
	}

	ControlRate(a);

} // DoAngleControl

static void DoYawControlFW(void) {
	static real32 KpScale = 1.0f;
	real32 NewRollCorr;
	PIStruct *P = &A[Yaw].P;
	PIDStruct *R = &A[Yaw].R;

	/*
	 if (F.YawActive) {
	 DesiredHeading = Heading;
	 KpScale = 1.0f;
	 } else if (NavState != PIC) {
	 DesiredHeading = Nav.DesiredHeading;
	 KpScale = Nav.Sensitivity;
	 }
	 */

	P->Error = MinimumTurn(DesiredHeading);
	P->Error = Limit1(P->Error, Nav.HeadingTurnout); // 150 30

	P->PTerm = P->Error * P->Kp * 0.1f * KpScale; //60deg * 20 * 0.1 * 0.8 -> 12

	R->Desired = P->PTerm;

	R->Error = (R->Desired + A[Yaw].Control) - Rate[Yaw];
	R->PTerm = R->Error * R->Kp;

	A[Yaw].Out = Limit1(R->PTerm, 1.0); // needs to be driven by LR acc

	NewRollCorr = atanf(R->Desired * Airspeed * GRAVITY_MPS_S_R);
	A[Roll].NavCorr = Limit1(NewRollCorr, Nav.MaxBankAngle);

} // DoYawControlFW

static void DoTurnControl(void) {
	PIDStruct *R = &A[Yaw].R;
	PIStruct *P = &A[Yaw].P;
	int8 ErrorSignP = 1;

	F.YawActive = Abs(A[Yaw].Stick) > StickDeadZone;

	if (F.YawActive) {
		DesiredHeading = SavedHeading = Heading;
		P->Error = 0.0f;
		A[Yaw].R.Desired = Threshold(A[Yaw].Stick, StickDeadZone)
				* A[Yaw].R.Max;
	} else {
		if ((NavState != HoldingStation)
				&& (F.OrbitingWP || F.RapidDescentHazard || F.UsingPOI
						|| F.UsingTurnToWP))
			DesiredHeading = Nav.DesiredHeading;

		P->Error = Limit1(MinimumTurn(DesiredHeading), P->Max);
		P->PTerm = P->Error * P->Kp;

		ConditionIntE(P, &ErrorSignP, dT);

		P->ITerm = P->IntE;

		A[Yaw].R.Desired = Limit1(P->PTerm + P->ITerm, Nav.MaxCompassRate);
	}

	R->Error = Limit1(R->Desired, R->Max) - Rate[Yaw];

	R->PTerm = R->Error * R->Kp;
	R->DTerm = ComputeAttitudeRateDerivative(R) * R->Kd;

	A[Yaw].Out = conditionOut(R->PTerm + R->DTerm);

} // DoTurnControl

static void DoTurnControlFW(void) {
	PIDStruct *R = &A[Yaw].R;
	PIStruct *P = &A[Yaw].P;
	int8 ErrorSignP = 1.0f;
	real32 NewRollCorr;

	F.YawActive = (Abs(A[Yaw].Stick) > StickDeadZone)
			|| (Abs(A[Roll].Stick) > StickDeadZone);

	if (F.YawActive) {
		DesiredHeading = SavedHeading = Heading;
		P->Error = 0.0f;
		R->Desired = Threshold(A[Yaw].Stick, StickDeadZone) * A[Yaw].R.Max;

		R->Error = Limit1(R->Desired, R->Max) - Rate[Yaw];
		R->PTerm = R->Error * R->Kp;
		R->DTerm = 0.0f; //ComputeAttitudeRateDerivative(R) * R->Kd;

		A[Yaw].Out = conditionOut(R->PTerm + R->DTerm);

	} else {
		if (F.Navigate || F.ReturnHome)
			DesiredHeading = Nav.DesiredHeading;

		P->Error = MinimumTurn(DesiredHeading);
		P->Error = Limit1(P->Error, Nav.HeadingTurnout); // 150 30

		P->PTerm = P->Error * P->Kp * 0.1f; //60deg * 20 * 0.1 * 0.8 -> 12
		ConditionIntE(P, &ErrorSignP, dT);

		P->ITerm = P->IntE;

		R->Desired = Limit1(P->PTerm + P->ITerm, Nav.MaxCompassRate);

		R->Error = Limit1(R->Desired, R->Max) - Rate[Yaw];
		R->PTerm = R->Error * R->Kp;
		R->DTerm = 0.0f; //ComputeAttitudeRateDerivative(R) * R->Kd;

		A[Yaw].Out = conditionOut(R->PTerm + R->DTerm); // needs to be driven by LR acc

		NewRollCorr = atanf(R->Desired * Airspeed * GRAVITY_MPS_S_R);
		A[Roll].NavCorr = Limit1(NewRollCorr, Nav.MaxBankAngle);
	}

} // DoTurnControlFW

void DoControl(void) {
	idx a;

	CurrMaxTiltAngle = Max(Abs(Angle[Roll]), Abs(Angle[Pitch]));
	F.NearLevel = CurrMaxTiltAngle < NAV_RTH_LOCKOUT_ANGLE_RAD;

	DoSetPointSlews();
	DoAttitudeGainScale(); // primitive gain scheduling

	if (F.IsFixedWing) {

		DoTurnControlFW(); // MUST BE BEFORE ROLL CONTROL

		for (a = Pitch; a <= Roll; a++)

			if ((a == Roll) && (Abs(Angle[Pitch]) > FWRollControlPitchLimitRad))
				DoRateDampingControl(Roll);
			else
				switch (AttitudeMode) {
				case AngleMode:
					DoAngleControl(a);
					break;
				case HorizonMode: // no Horizon Control for FW
				case RateMode:
					DoRateDampingControl(a);
					break;
				} // switch
	} else if (IsGroundVehicle) {

	} else {

		CalcTiltThrFF();

		DoTurnControl();

		for (a = Pitch; a <= Roll; a++)
			switch (AttitudeMode) {
			case AngleMode:
			case HorizonMode:
				DoAngleControl(a);
				break;
			case RateMode:
				DoRateControl(a);
				break;
			} // switch
	}

	switch (CurrBBLogType) {
	case logPitch:
		SendAttitudeControlPacket(TelemetrySerial, Pitch);
		break;
	case logRoll:
		SendAttitudeControlPacket(TelemetrySerial, Roll);
		break;
	case logYaw:
		SendAttitudeControlPacket(TelemetrySerial, Yaw);
		break;
	default:
		break;
	}

} // DoControl

void InitControl(void) {

	Sl = 0.0f;
	DesiredAlt = 0.0f;
	AltHoldThrComp = 0.0f;

} // InitControl

