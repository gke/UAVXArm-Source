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


#ifndef _control_h
#define _control_h

enum Attitudes {
	// strictly roll is a rotation around x but because
	// of the link between Acc and Angle we associate it with y or
	// the rotation of y around x
	Pitch,
	Roll,
	Yaw
};
enum Sensors {
	X, Y, Z
};

enum MagSensors {
	MX, MZ, MY
};
enum Directions {
	BF, LR, UD
};
// Roll, Pitch & Yaw

enum AttitudeModes {
	AngleMode, HorizonMode, RateMode, UnknownAttitudeMode
};

enum DerivativeFilterTypes {maDFilt, pavelDFilt, mlDFilt, unknownDFilt};

typedef struct {
	real32 Desired, Error, Kp, Ki, IntE, IntLim, Max;
	real32 PTerm, ITerm;
} PIStruct;

typedef struct {
	real32 Desired, Error, Kp, Ki, IntE, IntLim, Kd, Max;
	real32 PTerm, ITerm, DTerm;

	real32 RateP, RateD, RateDp;
	filterStruct RateF, RateDF;
} PIDStruct;

typedef struct {
	real32 Stick, Control;
	PIStruct P;
	PIDStruct R;
	//real32 Angle;
	real32 NavCorr, NavCorrP;
	real32 Out;
} AxisStruct;

typedef struct {
	PIStruct P;
	PIDStruct R;
} AltStruct;

AltStruct Alt;

void ZeroThrottleCompensation(void);
void ResetHeading(void);
void DoAltitudeControl(void);
void ZeroIntegrators(void);
real32 ComputeRateDerivative(PIDStruct *C);

void DoControl(void);
void InitControl(void);

extern AxisStruct A[];

extern idx AttitudeMode;
extern real32 TiltThrFFFrac;
extern real32 YawSense;

extern uint8 CurrOSLPFType;

extern real32 CameraAngle[3], OrbitCamAngle;
extern real32 DesiredHeading, SavedHeading;
extern real32 CurrMaxTiltAngle;
extern real32 Altitude;
extern real32 AltComp, ROC, MinROCMPS;
extern real32 DesiredThrottle, CruiseThrottle, IdleThrottle, InitialThrottle;
extern real32 BattThrFFComp, TiltThrFFComp;
extern real32 AltAccComp;
extern real32 HorizonTransScale;
extern real32 StickDeadZone;
extern real32 OrientationRad, OrientS, OrientC;

extern real32 FWRollPitchFFFrac, FWAileronDifferentialFrac,
		FWPitchThrottleFFFrac, FWMaxClimbAngleRad, FWClimbThrottleFrac,
		MaxRollAngleRad, FWGlideAngleOffsetRad, FWBoardPitchAngleRad,
		FWSpoilerDecayS, FWAileronRudderFFFrac,
		FWAltSpoilerFFFrac, MaxROCMPS, VRSDescentRateMPS;
extern real32 MaxControlGainReduction;
extern real32 GS;

#endif

