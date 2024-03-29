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


#ifndef _autonomous_h
#define _autonomous_h

enum Coords {
	NorthC, EastC, DownC
};

typedef struct {
	PIStruct P;
	PIDStruct R;
	real32 Acc;
	real32 DesPos, Pos, PosE, PosIntE;
	real32 DesVel, Vel, VelE;
	real32 Corr;
} NavCoordStruct;

typedef struct {

	NavCoordStruct C[3];

	real32 PosKp, PosKi, VelKp;
	real32 UNUSED_Sensitivity;
	real32 TakeoffBearing;
	real32 Distance, Bearing, Elevation;
	real32 Hint;

	real32 DesiredVel, DesiredHeading, SavedHeading;

	real32 WPDistance, OriginalWPBearing, WPBearing;
	real32 CrossTrackKp, CrossTrackE;

	real32 ProximityAlt, ProximityRadius;

	real32 FenceRadius;

	real32 MaxVelocity;
	real32 MaxBankAngle;

	real32 HeadingTurnout;
	real32 MaxCompassRate;
} NavStruct;

extern NavStruct Nav;

enum NavStates {
	HoldingStation,
	ReturningHome,
	AtHome,
	Descending,
	Touchdown,
	Transiting,
	Loitering,
	OrbitingPOI,
	Perching,
	Takeoff,
	PIC,
	AcquiringAltitude,
	UsingThermal,
	UsingRidge,
	UsingWave,
	BoostClimb,
	AltitudeLimiting,
	JustGliding,
	WPAltFail,
	WPProximityFail,
	UnknownNavState
};

enum NavComs {
	navVia, navOrbit, navPerch, navPOI, navPulse, navGlide, navLand, navGeoVertex, navUnknown
};

extern const char * NavComNames[];

typedef struct {
	real32 Pos[3];
	real32 Velocity;
	int16 Loiter;
	real32 OrbitRadius;
	real32 OrbitAltitude;
	real32 OrbitVelocity;
	int32 PulseWidthmS;
	int32 PulsePeriodmS;
	uint8 Action;
} WPStruct;

enum AlarmStates {
	NoAlarms, Monitoring, BatteryLow, LostSignal, HitFenceRTH, UpsideDown, ForcedLanding, GPSSerialPassThru,
	ArmingTimeout, CloseThrottle
};

enum LandingStates {
	InitDescent, CommenceDescent, Descent, DescentStopped
};

enum MotorStopActions {landNoStop, landContactSw, landDescentRate, landAccUBump, landDescentRateAndAccU};

void DisableFlightStuff(void );
void DoNavigation(void );
void InitNavigation(void );

void UpdateRTHSwState(void );
void CheckProximity(real32 V, real32 H);
void InitiateRTH(void );
void InitiatePH(void );
void CapturePosition(void );
void CheckRapidDescentHazard(void );
boolean NotDescending(void);

void DecayPosCorr(void );
void InitiateShutdown(uint8 s);

extern uint8 NavState, NavSwState, NavSwStateP, AlarmState;
extern real32 DesiredNavHeading;
extern boolean WPNavEnabled;

#endif


