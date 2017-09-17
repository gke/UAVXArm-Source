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

// Autonomous flight routines

#include "UAVX.h"

WPStruct WP, HP, POI;
uint8 CurrWPNo = 0;

const char * NavComNames[] = { "Via", "Orbit", "Perch", "POI" };

//  North, East, Altitude, Velocity, Loiter, OrbitRadius OrbitAltitude OrbitVelocity Action

//#define NAV_LEG_LENGTH 75
#define NAV_LEG_LENGTH 150

//#define DEFAULT_HOME_LAT  (-352902889L)
//#define DEFAULT_HOME_LON  (1491109972L)

const uint8 NoOfTestWayPoints = 4; // start at 1
const WPStruct TestWP[] = { { { 0, 0, 15 }, 3, 30, 0, 0, 0, 0 }, {
		{ 0, 0, 15 }, 3, 30, 0, 0, 0, navPOI }, { { 0, 50, 15 }, 3, 10, 0, 0,
		0, navPerch }, { { NAV_LEG_LENGTH, NAV_LEG_LENGTH, 100 }, 4, 1, 0, 0,
		0, navVia }, { { NAV_LEG_LENGTH, 0, 15 }, 3, 60, 8, 0, 2, navOrbit } };

void CaptureHomePosition(void) {
	idx a;

	if (F.GPSValid && (GPS.hAcc <= GPSMinhAcc) && !F.OriginValid) {

		mS[LastGPS] = mSClock();

		GPS.startTime = GPS.missionTime;

		NV.Mission.OriginLatitude = GPS.C[NorthC].OriginRaw = GPS.C[NorthC].Raw;
		NV.Mission.OriginLongitude = GPS.C[EastC].OriginRaw = GPS.C[EastC].Raw;

		// do it once - operations area small
		GPS.longitudeCorrection
				= Abs(cos(DegreesToRadians((real64)GPS.C[NorthC].Raw * 1e-7)));
		for (a = NorthC; a <= EastC; a++)
			GPS.C[a].Pos = GPS.C[a].Vel = GPS.C[a].PosP = 0.0f;

		GPS.originAltitude = GPS.altitude;
		F.HoldingAlt = false;

		NV.Mission.OriginAltitude = GPS.altitude;
		NV.Mission.OriginLatitude = GPS.C[NorthC].OriginRaw;
		NV.Mission.OriginLongitude = GPS.C[EastC].OriginRaw;

		setStat(OriginValidS, true);

		F.OriginValid = true;

		CapturePosition();

		if (F.GPSToLaunchRequired) {
			Delay1mS(500);
			DoBeep(2, 2);
			DoBeep(6, 2);
		} else {
			mS[BeeperTimeout] = mSClock() + 1500;
			BeeperOn();
		}
	}
} // CaptureHomePosition

void GenerateNavTestMission(void) {
	MissionStruct * M;
	real32 Scale;
	uint8 wp;

	Scale = IsMulticopter ? 1.0f : 4.0f;
	M = &NV.Mission;

	M->NoOfWayPoints = Limit(NoOfTestWayPoints, 0, NAV_MAX_WAYPOINTS);
	M->ProximityAltitude = NAV_PROXIMITY_ALTITUDE_M;
	M->ProximityRadius = IsMulticopter ? NAV_PROXIMITY_RADIUS_M
			: WING_PROXIMITY_RADIUS_M;
	M->OriginAltitude = OriginAltitude;
	M->OriginLatitude = GPS.C[NorthC].OriginRaw;
	M->OriginLongitude = GPS.C[EastC].OriginRaw;
	M->RTHAltHold = (int16) (P(NavRTHAlt)); // ??? not used

	for (wp = 1; wp <= M->NoOfWayPoints; wp++) {
		M->WP[wp].LatitudeRaw = MToGPS(TestWP[wp].Pos[NorthC]) * Scale
				+ M->OriginLatitude;
		M->WP[wp].LongitudeRaw = MToGPS(TestWP[wp].Pos[EastC]) * Scale
				/ GPS.longitudeCorrection + M->OriginLongitude;
		M->WP[wp].Altitude = TestWP[wp].Pos[DownC];
		M->WP[wp].VelocitydMpS = TestWP[wp].Velocity * 10;
		M->WP[wp].Loiter = TestWP[wp].Loiter;
		M->WP[wp].OrbitRadius = (TestWP[wp].OrbitRadius) * Scale;
		M->WP[wp].OrbitAltitude = TestWP[wp].OrbitAltitude;
		M->WP[wp].OrbitVelocitydMpS = TestWP[wp].OrbitVelocity * 10;
		M->WP[wp].Action = TestWP[wp].Action;
	}

	UpdateNV();

} // GenerateNavTestMission


boolean NavMissionSanityCheck(MissionStruct * M) {

	// rely on UAVXNav for now
	//CHECK FOR ANY ZERO LAT/LON VALUES ZZZ

	return (true);
} // NavMissionSanityCheck

uint8 NextWPState(void) {

//zzz
	CurrWPNo++;
	RefreshNavWayPoint();

	return (CurrWPNo == 0 ? ReturningHome : Transiting);
} // NexWPState

void RefreshNavWayPoint(void) {

	GetNavWayPoint();
	while (WP.Action == navPOI) {
		POI.Pos[NorthC] = WP.Pos[NorthC];
		POI.Pos[EastC] = WP.Pos[EastC];
		CurrWPNo++;
		GetNavWayPoint();
	}

	DesiredAltitude = WP.Pos[DownC];

} // RefreshNavWayPoint


void GetNavWayPoint(void) {
	WPStructNV * W;

	if (CurrWPNo > NV.Mission.NoOfWayPoints)
		CurrWPNo = 0;

	if (CurrWPNo == 0) { // override mission wp 0 and force to Origin
		WP.Pos[NorthC] = HP.Pos[NorthC] = 0.0f;
		WP.Pos[EastC] = HP.Pos[EastC] = 0.0f;
		WP.Pos[DownC] = HP.Pos[DownC] = P(NavRTHAlt);
		WP.Velocity = HP.Velocity = Nav.MaxVelocity;
		WP.Loiter = (int16) P(DescentDelayS); // mS
		WP.Action = navLand;

		WP.OrbitRadius = HP.OrbitRadius = DESCENT_RADIUS_M;
		WP.OrbitAltitude = P(NavRTHAlt);
		WP.OrbitVelocity = HP.OrbitVelocity = DESCENT_VELOCITY_MPS;

		HP.Loiter = 0;
		HP.Action = navUnknown;

		F.UsingPOI = false;
	} else {
		W = &NV.Mission.WP[CurrWPNo];
		WP.Pos[NorthC] = GPSToM(W->LatitudeRaw - GPS.C[NorthC].OriginRaw);
		WP.Pos[EastC] = GPSToM(W->LongitudeRaw - GPS.C[EastC].OriginRaw)
				* GPS.longitudeCorrection;
		WP.Pos[DownC] = (real32) W->Altitude;
		WP.Velocity = (real32) W->VelocitydMpS * 0.1f;
		WP.Loiter = (int16) W->Loiter; // S
		WP.Action = W->Action;
		if (WP.Action == navPOI)
			F.UsingPOI = true;

		WP.OrbitRadius = (real32) W->OrbitRadius; // M
		WP.OrbitAltitude = (real32) W->OrbitAltitude;
		WP.OrbitVelocity = (real32) W->OrbitVelocitydMpS * 0.1f; // dM/S
	}

#ifdef NAV_ENFORCE_ALTITUDE_CEILING
	WP.Pos[DownC] = Limit(WP.Pos[DownC], 0, NAV_CEILING_M);
	WP.OrbitAltitude = Limit(WP.OrbitAltitude, 0, NAV_CEILING_M);
#endif // NAV_ENFORCE_ALTITUDE_CEILING
} // GetNavWaypoint

void DoMissionUpdate(void) {

	if (NavMissionSanityCheck(&NV.NewMission))
		memcpy(&NV.Mission, &NV.NewMission, sizeof(MissionStruct));

	memset(&NV.NewMission, 0, sizeof(MissionStruct));

	if (State != InFlight)
		UpdateNV();

} // DoMissionUpdate


