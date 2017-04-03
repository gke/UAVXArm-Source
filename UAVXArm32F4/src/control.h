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

enum ControlModes {
	AngleMode, RateMode, RelayAngleMode, RelayRateMode
};

typedef struct {
	real32 Kp, Ki, IntE, IL, Kd, Dp, Max;
} PIDStruct;

typedef struct {
	PIDStruct O, I;
	// controls
	real32 Desired;
	// body frame sensors
	real32 Ratep, DriftCorr, Angle;
	// stats
	real32 RateD, RateDp;
	HistStruct RateF, RateDF;
	real32 Control, NavCorr;
	real32 Out;
} AxisStruct;

typedef struct {
	PIDStruct O, I;
	real32 IL;
} AltStruct;

void ZeroCompensation(void);
void DoAltitudeControl(void);
void ZeroPIDIntegrals(void);

void DoControl(void);
void InitControl(void);

AxisStruct A[3];
AltStruct Alt;
real32 TiltThrFFFrac;

real32 CameraAngle[3], OrbitCamAngle;

real32 CurrMaxTiltAngle;
int16 AttitudeHoldResetCount;
real32 DesiredAltitude, Altitude, DesiredROC;
real32 AltComp, AltCompDecayS, ROC, MinROCMPS;
real32 CruiseThrottle;
real32 BattThrFFComp, TiltThrFFComp;
real32 AltAccComp;
real32 HorizonTransPoint;
real32 YawStickScaleRadPS, RollPitchStickScaleRadPS;
real32 MaxAttitudeAngleRad;
real32 YawStickThreshold;
real32 OrientationRad, OrientS, OrientC;

real32 FWRollPitchFFFrac, FWAileronDifferentialFrac,
		FWPitchThrottleFFFrac, MaxAltHoldCompFrac, FWMaxClimbAngleRad, MaxRollAngleRad,
		FWGlideAngleOffsetRad, FWFlapDecayS, BestROCMPS;

real32 GS;

#endif

