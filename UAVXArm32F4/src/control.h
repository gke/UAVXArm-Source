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
	real32 Stick, StickP, StickD;
	real32 AngleDesired, AngleE, AngleKp, AngleKi, AngleIntE, AngleIL, AngleMax,
			RateDesired, RateE, RateKp, RateKd, RateMax, CompassRateMax;
	// controls
	// body frame sensors
	real32 Ratep, DriftCorr, Angle;
	// stats
	real32 RateD, RateDp;
	HistStruct RateF, RateDF;
	real32 Control, NavCorr, NavCorrP;
	real32 Out;
} AxisStruct;

typedef struct {
	real32 PosKp, PosKi, PosE, PosIntE, PosIL, VelKp, VelKd, VelE; // does not include altitude and ROC
} AltStruct;

AltStruct Alt;

void ZeroThrottleCompensation(void);
void DoAltitudeControl(void);
void ZeroPIDIntegrals(void);
real32 ComputeRateDerivative(AxisStruct *C);

void DoControl(void);
void InitControl(void);

AxisStruct A[3];

real32 TiltThrFFFrac;
real32 DerivativeLPFreqHz;
idx DerivativeLPFOrder;

real32 CameraAngle[3], OrbitCamAngle;

real32 CurrMaxTiltAngle;
real32 DesiredAltitude, Altitude, DesiredROC;
real32 AltComp, AltCompDecayS, ROC, MinROCMPS;
real32 CruiseThrottle;
real32 BattThrFFComp, TiltThrFFComp;
real32 AltAccComp;
real32 HorizonTransPoint;
real32 StickDeadZone;
real32 OrientationRad, OrientS, OrientC;

real32 FWRollPitchFFFrac, FWAileronDifferentialFrac,
		FWPitchThrottleFFFrac, MaxAltHoldCompFrac, FWMaxClimbAngleRad,
		MaxRollAngleRad, FWGlideAngleOffsetRad, FWBoardPitchAngleRad,
		FWSpoilerDecayS, FWAileronRudderFFFrac,
		FWAltSpoilerFFFrac, BestROCMPS;
real32 ThrottleGain;
real32 GS;

#endif

