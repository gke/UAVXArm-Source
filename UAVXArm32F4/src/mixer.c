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

const real32 AFOrientation[AFUnknown + 1] = { // K1 arm relative to board North
		-180, -180, -180, // TriAF, TriCoaxAF, VTailAF
				0, -45, 0, -45, // QuadAF, QuadXAF, QuadCoaxAF, QuadCoaxXAF
				0, -30, // HexAF, HexXAF
				0, -22.5, // OctAF, OctXAF
				0, 0, // Heli90AF, BiAF
				0, 0, 0, 0, 0, 0, // ElevonAF, DeltaAF, AileronAF, AileronSpoilerFlapsAF, AileronVTailAF, RudderElevatorAF,
				0, // DifferentialTwinAF
				0, 0, // VTOLAF, VTOL2AF
				0, 0, 0, // TrackedAF, FourWheelAF,TwoWheelAF,
				0, // GimbalAF,
				0, // Instrumentation
				0 }; // AFUnknown

const uint8 SM[] = { RightAileronC, LeftAileronC, ElevatorC, RudderC, SpoilerC,
		CamRollC, Aux1CamPitchC };

real32 PWSense[MAX_PWM_OUTPUTS];

real32 OrientationRad = 0.0f;
real32 OrientS = 0.0f;
real32 OrientC = 1.0f;

real32 IdleThrottlePW;
real32 NetThrottle;
real32 CGOffset;
boolean VTOLMode = false;

void RotateOrientation(real32 * nx, real32 * ny, real32 x, real32 y) {
	real32 Temp;

	Temp = x * OrientC + y * OrientS;
	*ny = -x * OrientS + y * OrientC;
	*nx = Temp;
}
// RotateOrientation

void DoDifferential(uint8 R, uint8 L) {

    real32 d = 1.0f - FWAileronDifferentialFrac;
	if (PW[R] > 0.0f)
		PW[R] *= d;

	if (PW[L] > 0.0f)
		PW[L] *= d;

} // DoDifferential

void MixAndLimitCam(void) {
	real32 NewCamPitch, NewCamRoll;

	if (UAVXAirframe == GimbalAF) { // Gimbal not commissioned
		NewCamRoll = Angle[Roll];
		NewCamPitch = Angle[Pitch];

		PW[CamRollC] = PW[Aux1CamPitchC] = OUT_NEUTRAL;
	} else {
		NewCamRoll = Angle[Roll] * Cam.RollKp + (real32) P(RollCamTrim) * 0.01f;
		NewCamRoll = (real32) PWSense[CamRollC] * NewCamRoll * OUT_MAXIMUM
				+ OUT_NEUTRAL;

		NewCamPitch = Angle[Pitch] * Cam.PitchKp + OrbitCamAngle + CamPitchTrim;
		NewCamPitch = PWSense[Aux1CamPitchC] * NewCamPitch * OUT_MAXIMUM
				+ OUT_NEUTRAL;

		PW[CamRollC] = NewCamRoll;
		PW[Aux1CamPitchC] = NewCamPitch;
	}

} // MixAndLimitCam

void DoVTOLMix(void) {
	real32 TempThrottle, TempElevator, TempRudder;

	TempThrottle =
			(F.PassThru) ? DesiredThrottle : DesiredThrottle + AltHoldThrComp;
	NetThrottle = TempThrottle = Limit(TempThrottle * OUT_MAXIMUM, 0.0f, 1.0f);

	if (VTOLMode) {

		PW[LeftThrottleC] = TempThrottle + Rl;
		PW[RightThrottleC] = TempThrottle - Rl;

		TempRudder = -PWSense[RudderC] * Yl;
		TempElevator = PWSense[ElevatorC] * Pl;

		// assume servos are opposite hand
		PW[RightPitchYawC] = PWSense[RightPitchYawC]
				* (TempElevator + TempRudder) + OUT_NEUTRAL;
		PW[LeftPitchYawC] = PWSense[LeftPitchYawC]
				* (-TempElevator + TempRudder) + OUT_NEUTRAL;

	} else {

		TempRudder = -PWSense[RudderC] * Yl; // use Roll not Yaw
		PW[LeftThrottleC] = TempThrottle + Yl;
		PW[RightThrottleC] = TempThrottle - Yl;

		TempElevator = PWSense[ElevatorC]
				* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)));
		// assume servos are opposite hand
		PW[RightPitchYawC] = PWSense[RightPitchYawC]
				* (TempElevator + TempRudder + Rl) + OUT_NEUTRAL;
		PW[LeftPitchYawC] = PWSense[LeftPitchYawC]
				* (-TempElevator + TempRudder + Rl) + OUT_NEUTRAL;
	}
} // DoVTOLMix


void DoGroundVehicleMix(void) {


} // DoGroundVehicleMix

void DoMix(void) {
#define OUT_MAX_SPOILER 0.3f // so we still have some aileron control left
	real32 TempThrottle, TempRudder, TempElevator, TempAileron,
			TempSpoilerFlaps;

	if (F.PassThru) // do here at lowest level rather than complicating higher level logic
		TempThrottle = DesiredThrottle;
	else {
		TempThrottle =
				ThrottleSuppressed ? 0.0f : DesiredThrottle + AltHoldThrComp;
		TempThrottle = Limit(TempThrottle, 0.0f, OUT_MAXIMUM * FWClimbThrottleFrac);

	}

	NetThrottle = PW[RightThrottleC] = PW[LeftThrottleC] = Limit(
			TempThrottle * OUT_MAXIMUM, 0.0f, OUT_MAXIMUM);

	switch (UAVXAirframe) {
	case Heli90AF:
		PW[RudderC] = PWSense[RudderC] * Yl + OUT_NEUTRAL;
		PW[RightAileronC] = PWSense[RightAileronC] * Rl + OUT_NEUTRAL;
		// left aileron not used
		PW[ElevatorC] = PWSense[ElevatorC] * Pl + OUT_NEUTRAL;

		break;
	case BiAF:
		PW[LeftThrottleC] -= Rl;
		PW[RightThrottleC] += Rl;

		PW[LeftTiltServoC] = PWSense[LeftTiltServoC] * (Pl + Yl) + OUT_NEUTRAL;
		PW[RightTiltServoC] = PWSense[RightTiltServoC]
				* (-Pl - Yl)+ OUT_NEUTRAL;

		break;
	case ElevonAF:
	case DeltaAF:
		if (VTOLMode) {
			PW[RudderC] = -PWSense[RudderC] * Rl + OUT_NEUTRAL;

			TempElevator = PWSense[ElevatorC] * Pl;
			// assume servos are opposite hand
			PW[RightElevonC] = PWSense[RightElevonC]
					* (TempElevator + Yl)+ OUT_NEUTRAL;
			PW[LeftElevonC] = PWSense[LeftElevonC]
					* (-TempElevator + Yl)+ OUT_NEUTRAL;
		} else {
			PW[RudderC] = -PWSense[RudderC] * Yl + OUT_NEUTRAL;

			TempElevator = PWSense[ElevatorC]
					* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)));
			// assume servos are opposite hand
			PW[RightElevonC] = PWSense[RightElevonC]
					* (TempElevator + Rl)+ OUT_NEUTRAL;
			PW[LeftElevonC] = PWSense[LeftElevonC]
					* (-TempElevator + Rl)+ OUT_NEUTRAL;
		}
		// assume servos are opposite hand
		PW[SpoilerC] = PWSense[SpoilerC] * Sl;

		break;
	case AileronAF:
		PW[RudderC] = PWSense[RudderC] * Yl + OUT_NEUTRAL;

		PW[RightAileronC] = Rl;
		PW[LeftAileronC] = -Rl;
		DoDifferential(RightAileronC, LeftAileronC);
		PW[RightAileronC] *= PWSense[RightAileronC];
		PW[LeftAileronC] *= -PWSense[LeftAileronC];
		PW[RightAileronC] += OUT_NEUTRAL;
		PW[LeftAileronC] += OUT_NEUTRAL;

		PW[ElevatorC] =
				PWSense[ElevatorC]
						* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)))+ OUT_NEUTRAL;

		// assume servos are opposite hand
		PW[SpoilerC] = PWSense[SpoilerC] * Sl + OUT_NEUTRAL;

		break;
	case AileronSpoilerFlapsAF:

		PW[RudderC] = PWSense[RudderC] * Yl + OUT_NEUTRAL;

		PW[RightAileronC] = Rl;
		PW[LeftAileronC] = -Rl;
		DoDifferential(RightAileronC, LeftAileronC);
		PW[RightAileronC] *= PWSense[RightAileronC];
		PW[LeftAileronC] *= -PWSense[LeftAileronC];

		TempSpoilerFlaps = -PWSense[SpoilerC] * Sl * OUT_MAX_SPOILER;
		PW[RightAileronC] =
				(TempSpoilerFlaps + PW[RightAileronC]) + OUT_NEUTRAL;
		PW[LeftAileronC] +=
				(-TempSpoilerFlaps + PW[LeftAileronC]) + OUT_NEUTRAL;

		PW[ElevatorC] =
				PWSense[ElevatorC]
						* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)))+ OUT_NEUTRAL;
		break;
	case AileronVTailAF:
		PW[RightAileronC] = Rl;
		PW[LeftAileronC] = -Rl;
		DoDifferential(RightAileronC, LeftAileronC);
		PW[RightAileronC] *= PWSense[RightAileronC];
		PW[LeftAileronC] *= -PWSense[LeftAileronC];
		PW[RightAileronC] += OUT_NEUTRAL;
		PW[LeftAileronC] += OUT_NEUTRAL;

		TempElevator = (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)));
		PW[RightRudderElevatorC] = PWSense[RightRudderElevatorC]
				* TempElevator+ OUT_NEUTRAL;
		PW[LeftRudderElevatorC] = -PWSense[LeftRudderElevatorC]
				* TempElevator+ OUT_NEUTRAL;

		PW[RightRudderElevatorC] -= Yl;
		PW[LeftRudderElevatorC] -= Yl;

		PW[SpoilerC] = PWSense[SpoilerC] * Sl + OUT_NEUTRAL;

		break;
	case RudderElevatorAF:
		TempAileron = PWSense[RightAileronC] * Rl;

		PW[RudderC] = PWSense[RudderC] * (TempAileron + Yl) + OUT_NEUTRAL;

		PW[ElevatorC] =
				PWSense[ElevatorC]
						* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)))+ OUT_NEUTRAL;

		PW[SpoilerC] = PWSense[SpoilerC] * Sl + OUT_NEUTRAL;
		break;
	case DifferentialTwinAF:
		TempThrottle = PW[RightThrottleC];
		TempAileron = PWSense[RightAileronC] * Rl;
		PW[RightThrottleC] = TempThrottle + TempAileron;
		PW[LeftThrottleC] = TempThrottle - TempAileron;
		break;
	case VTOL2AF:
	case VTOLAF:
		DoVTOLMix();
		break;

	default:
		break;
	} // switch

} // DoMix

void MixMulti(void) {
	real32 R, P, Y;
	idx m;

	switch (UAVXAirframe) {
	case TriAF: // usually flown K1 motor to the rear - use orientation of 24
		R = Rl * 1.1547f;
		P = Pl * (1.0f + CGOffset);
		PW[LeftC] = -R + P;
		PW[RightC] = R + P;

		PW[FrontC] = -(Pl * (1.0f - CGOffset));

		PW[YawC] = PWSense[YawC] * Yl + OUT_NEUTRAL; // * 1.3333 yaw servo
		break;
	case TriCoaxAF: // Y6
		R = Rl * 1.1547f;
		PW[FrontBC] = PW[FrontTC] = -Pl;
		PW[LeftBC] = PW[LeftTC] = -R + Pl;
		PW[RightBC] = PW[RightTC] = R + Pl;

		Y = Yl * 0.6667f;
		PW[FrontTC] += Y;
		PW[LeftTC] += Y;
		PW[RightTC] += Y;

		PW[FrontBC] -= Y;
		PW[LeftBC] -= Y;
		PW[RightBC] -= Y;
		break;
	case VTailAF: // usually flown VTail (K1+K4) to the rear
		P = Pl * (1.0f + CGOffset);
		PW[LeftC] = P - Rl; // right rear
		PW[RightC] = P + Rl; // left rear

		P = Pl * (1.0f - CGOffset);
		PW[FrontLeftC] = -(P + PWSense[RudderC] * Yl);
		PW[FrontRightC] = -(P - PWSense[RudderC] * Yl);
		break;
	case QuadAF:
	case QuadXAF:
	case QuadCoaxAF:
	case QuadCoaxXAF: // not commissioned
		Yl *= MultiPropSense;
		PW[LeftC] = -Rl - Yl;
		PW[RightC] = Rl - Yl;
		PW[FrontC] = -Pl + Yl;
		PW[BackC] = Pl + Yl;
		break;
	case HexAF:
	case HexXAF:
		Yl *= MultiPropSense;
		P = Pl * 0.5f;
		R = Rl * 0.5773503f;
		Y = Yl; //* 0.6667f;
		PW[HFrontC] = -P + Y;
		PW[HLeftFrontC] = -R - P - Y;
		PW[HRightFrontC] = R - P - Y;

		PW[HLeftBackC] = -R + P + Y;
		PW[HRightBackC] = R + P + Y;
		PW[HBackC] = P - Y;
		break;
	case OctAF:
	case OctXAF: // use Y leads
		Yl *= MultiPropSense;
		PW[LeftC] = (-Rl - Yl) * 0.5f;
		PW[RightC] = (Rl - Yl) * 0.5f;
		PW[FrontC] = (-Pl + Yl) * 0.5f;
		PW[BackC] = (Pl + Yl) * 0.5f;
		break;
	default:
		break;
	} // switch
} // MixMulti

real32 MaxMotorSwing(void) {
	real32 DemandSwing, pw;
	idx m;

	DemandSwing = Abs(PW[0]);
	for (m = 1; m < NoOfDrives; m++) {
		pw = Abs(PW[m]);
		if (pw > DemandSwing)
			DemandSwing = pw;
	}

	return (DemandSwing);
} // MaxMotorSwing

void RescaledMultiMix_OLD(real32 CurrThrottlePW) {
#define	MIN_PRESERVED_YAW_PW FromPercent(10)
	idx m;
	real32 Scale, DemandSwing, AvailableSwing, TempYl;

	TempYl = Yl;
	Yl = 0.0f;

	MixMulti(); // without yaw

	AvailableSwing = Min(OUT_MAXIMUM - CurrThrottlePW,
			CurrThrottlePW - THR_START_PW);
	DemandSwing = MaxMotorSwing();

	F.Saturation = DemandSwing > AvailableSwing;

	if (F.Saturation) { // nothing left for yaw!

		Scale = AvailableSwing / (DemandSwing - MIN_PRESERVED_YAW_PW);
		Rl *= Scale;
		Pl *= Scale;

		Yl = Limit1(TempYl, MIN_PRESERVED_YAW_PW);
		MixMulti();
	} else {
		Yl = Limit1(TempYl, AvailableSwing - DemandSwing);
		MixMulti();
	}

} // RescaledMultiMix


void DoMulticopterMix(void) {
	real32 CurrThrottlePW, AvailableYawSwing, MinThrottle;
	idx m;

	RotateOrientation(&Rl, &Pl, Rl, Pl);

	CurrThrottlePW = (
			(State == InFlight) ?
					(DesiredThrottle + AltHoldThrComp) * TiltThrFFComp
							* BattThrFFComp :
					DesiredThrottle) * OUT_MAXIMUM;

	CurrThrottlePW = Limit(CurrThrottlePW, IdleThrottlePW, OUT_MAXIMUM);

	if ((CurrThrottlePW < IdleThrottlePW) || !F.DrivesArmed) {
		CurrThrottlePW = 0.0f;
		for (m = 0; m < NoOfDrives; m++)
			PW[m] = PWp[m] = 0;
	} else {
		F.EnforceDriveSymmetry = true; //TODO: BROKEN at low throttle - maybe try yaw preserve?;
		if (F.EnforceDriveSymmetry) {
			AvailableYawSwing = Min(OUT_MAXIMUM - CurrThrottlePW,
					CurrThrottlePW - THR_START_PW) * 0.6f;
			Yl = Limit1(Yl, AvailableYawSwing);
		}

		MixMulti();

		MinThrottle = (State == InFlight) ? IdleThrottlePW : 0.0f;
		for (m = 0; m < NoOfDrives; m++)
			PW[m] = Limit(PW[m] + CurrThrottlePW, MinThrottle, OUT_MAXIMUM);
	}
	NetThrottle = CurrThrottlePW; // for logging only

} // DoMulticopterMix

void InitServoSense(void) {
	idx b, m;

	for (m = 0; m < MAX_PWM_OUTPUTS; m++)
		PWSense[m] = 1.0f;

	b = P(ServoSense);
	for (m = 0; m <= 6; m++) { // RightAileron .. CamPitch
		PWSense[SM[m]] = ((b & 1) ? -1.0f : 1.0f);
		b >>= 1;
	}

} // InitServoSense

