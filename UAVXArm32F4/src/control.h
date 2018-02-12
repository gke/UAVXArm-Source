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
	real32 Desired, Error, Kp, Kd, Max;
	real32 PTerm, DTerm;

	real32 RateP, RateD, RateDp;
	HistStruct RateF, RateDF;
} PDStruct;

typedef struct {
	real32 Stick;
	real32 Control; // zzz used by last good Ken version
	PIStruct P;
	PDStruct R;
	real32 Angle;
	real32 NavCorr, NavCorrP;
	real32 Out;
} AxisStruct;

typedef struct {
	PIStruct P;
	PDStruct R;
} AltStruct;

AltStruct Alt;

void ZeroThrottleCompensation(void);
void DoAltitudeControl(void);
void ZeroIntegrators(void);
real32 ComputeRateDerivative(PDStruct *C);

void DoControl(void);
void InitControl(void);

AxisStruct A[3];

idx AttitudeMode;
real32 TiltThrFFFrac;
real32 CurrAccLPFHz, CurrGyroLPFHz, CurrYawLPFHz, CurrServoLPFHz;
boolean UsingPavelFilter;

real32 CameraAngle[3], OrbitCamAngle;
real32 DesiredHeading, SavedHeading;
real32 CurrMaxTiltAngle;
real32 Altitude;
real32 AltComp, ROC, MinROCMPS;
real32 CruiseThrottle;
real32 BattThrFFComp, TiltThrFFComp;
real32 AltAccComp;
real32 HorizonTransScale;
real32 StickDeadZone;
real32 OrientationRad, OrientS, OrientC;

real32 FWRollPitchFFFrac, FWAileronDifferentialFrac,
		FWPitchThrottleFFFrac, MaxAltHoldCompFrac, FWMaxClimbAngleRad, FWClimbThrottleFrac,
		MaxRollAngleRad, FWGlideAngleOffsetRad, FWBoardPitchAngleRad,
		FWSpoilerDecayS, FWAileronRudderFFFrac,
		FWAltSpoilerFFFrac, BestROCMPS;
real32 MaxControlGainReduction;
real32 GS;

#endif

