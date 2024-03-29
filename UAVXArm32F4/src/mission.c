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

WPStruct WP, HP, HomeWP, OffsetHomeWP, POI;
uint8 CurrWPNo = 0;

const char * NavComNames[] = { "Via", "Orbit", "Perch", "POI" };

//  North, East, Altitude, Velocity, Loiter, OrbitRadius OrbitAltitude OrbitVelocity Action

MissionStruct NewNavMission;
boolean NavMissionUpdated = true;
NavPulseStruct NavPulse, SavedNavPulse;
boolean UsingSurveyPulse;

void EnableWPNav(void) {
#if !defined(USE_AUX3_PROBE_PIN)
	WPNavEnabled = !DigitalRead(&GPIOPins[WPMissionOrProbeSel].P);
#endif
}// EnableWPNav

//______________________________________________________________________________

// CheckFence

FenceSegmentStruct Fence[NAV_MAX_FENCE_SEGMENTS];
uint16 noofFenceSegments = 0;

void RefreshFence(void) {
	WPStructNV * W;
	idx wp;

	noofFenceSegments = 0;
	for (wp = 0; wp < Config.Mission.NoOfWayPoints; wp++) {
		W = &Config.Mission.WP[wp];
		if ((W->Action == navGeoVertex) && (noofFenceSegments
				< NAV_MAX_FENCE_SEGMENTS)) {
			Fence[noofFenceSegments].lat = W->LatitudeRaw;
			Fence[noofFenceSegments].lon = W->LongitudeRaw;
			noofFenceSegments++;
		}
	}
} // RefreshFence


void CheckFence(void) {
	//  Randolph Franklin https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
	idx i, j;
	boolean c;

	if (Altitude > NAV_CEILING_M)
		F.FenceAlarm = true;
	else if (noofFenceSegments <= 0)
		F.FenceAlarm = Nav.Distance > Nav.FenceRadius;
	else {
		c = false;
		for (i = 0, j = noofFenceSegments - 1; i < noofFenceSegments; j = i++) {
			if (((Fence[i].lat > GPS.lat) != (Fence[j].lat > GPS.lat))
					&& (GPS.lon < (Fence[j].lon - Fence[i].lon) * (GPS.lat
							- Fence[i].lat) / (Fence[j].lat - Fence[i].lat)
							+ Fence[i].lon))
				c = !c;
		}
		F.FenceAlarm = !c;
	}

} // CheckFence


void ClearNavMission(void) {

	memset(&Config.Mission, 0, sizeof(MissionStruct));

	Config.Mission.FenceRadius = NAV_DEFAULT_FENCE_M;

	noofFenceSegments = 0;

} // ClearMission

void GenerateHomeWP(void) {

	memset(&HomeWP, 0, sizeof(HomeWP));

	HomeWP.Pos[DownC] = P(NavRTHAlt);
	HomeWP.Velocity = Nav.MaxVelocity;
	HomeWP.Loiter = (int16) P(DescentDelayS); // mS

	HomeWP.OrbitAltitude = P(NavRTHAlt); // TODO: for camera height above ground
	HomeWP.Action = navLand;

	memcpy(&OffsetHomeWP, &HomeWP, sizeof(WPStruct));
	memcpy(&HP, &HomeWP, sizeof(WPStruct));

	F.OffsetOriginValid = false;

} // GenerateHomeWP


void CaptureHomePosition(void) {
	idx a;

	if (F.GPSValid && (GPS.hAcc <= GPSMinhAcc) && (GPS.vAcc <= GPSMinvAcc)) {

		Delay1mS(5000); // GPS packets are still arriving under interrupts

		mS[LastGPSmS] = mSClock();

		GPS.startTime = GPS.missionTime;

		if (F.Emulation) {
			GPS.C[NorthC].Raw = DEFAULT_HOME_LAT;
			GPS.C[EastC].Raw = DEFAULT_HOME_LON;
			GPS.longitudeCorrection = DEFAULT_LON_CORR;
		}

		GPS.C[NorthC].OriginRaw = GPS.C[NorthC].Raw;
		GPS.C[EastC].OriginRaw = GPS.C[EastC].Raw;

		// do it once - operations area small
		GPS.longitudeCorrection
				= Abs(cos(DegreesToRadians((real64)GPS.C[NorthC].Raw * 1e-7)));
		for (a = NorthC; a <= EastC; a++) {
			GPS.C[a].Pos = GPS.C[a].Vel = 0.0f;
			GPS.C[a].RawP = 0;
		}

		GPS.originAltitude = GPS.altitude;
		F.HoldingAlt = false;

		F.OriginValid = true;

		CapturePosition();

		DoBeep(2, 2);
		DoBeep(6, 2);
	}
} // CaptureHomePosition

void CaptureOffsetHomePosition(void) {

	if (!F.OffsetOriginValid) {
		if (sqrtf(Sqr(GPS.C[NorthC].Pos) + Sqr(GPS.C[EastC].Pos))
				> Nav.ProximityRadius) {// TODO: RTH or first PH
			OffsetHomeWP.Pos[NorthC] = GPS.C[NorthC].Pos;
			OffsetHomeWP.Pos[EastC] = GPS.C[EastC].Pos;
			CapturePosition();
			F.OffsetOriginValid = true;
		}
	}

} // CaptureOffsetHomePosition


boolean NavMissionSanityCheck(MissionStruct * M) {

	// rely on UAVXNav for now
	//CHECK FOR ANY ZERO LAT/LON VALUES ZZZ


	return (true);
} // NavMissionSanityCheck


void ScheduleNavPulse(NavPulseStruct * n, timemS w, timemS p) {
	n->State = false;
	n->WidthmS = w;
	n->PeriodmS = p;
	n->Active = w > 0;
	mSTimer(NavPulseUpdatemS, 0);
} // ScheduleNavPulse


void UpdateNavPulse(boolean v) {
#if defined(UAVXF4V3)|| defined(UAVXF4V3BBFLASH)
	if (GPIOPins[Aux1Sel].Used && F.NavigationEnabled)
	DigitalWrite(&GPIOPins[Aux1Sel].P, v);
#endif
} // DoNavPulse

void CheckNavPulse(NavPulseStruct * n) {

	if (mSTimeout(NavPulseUpdatemS)) {
		switch (n->State) {
		case false:
			if (F.NavigationEnabled && n->Active || UsingSurveyPulse) {
				UpdateNavPulse(true);
				mSTimer(NavPulseUpdatemS, n->WidthmS);
			}
			break;
		case true:
			UpdateNavPulse(false);
			if ((n->PeriodmS <= n->WidthmS)) { // oneshot
				n->Active = UsingSurveyPulse = false;
			} else
				mSTimer(NavPulseUpdatemS, n->PeriodmS - n->WidthmS);
			break;
		} // switch
		n->State = !n->State;
	}

} // CheckNavPulse


void NextWP(void) {

	CurrWPNo++;
	if (CurrWPNo > Config.Mission.NoOfWayPoints)
		CurrWPNo = 0;

	NavState = Transiting;

} // NextWP

void SetWPHome(void) {
	if (F.IsFixedWing && F.UsingOffsetHome && F.OffsetOriginValid)
		memcpy(&WP, &OffsetHomeWP, sizeof(WPStruct));
	else
		memcpy(&WP, &HomeWP, sizeof(WPStruct));
} // SetWPHome

void RefreshNavWayPoint(void) {

	GetNavWayPoint();
	while ((WP.Action == navPOI) || (WP.Action == navPulse) || (WP.Action
			== navGeoVertex)) {
		switch (WP.Action) {
		case navPOI:
			F.UsingPOI = true;
			POI.Pos[NorthC] = WP.Pos[NorthC];
			POI.Pos[EastC] = WP.Pos[EastC];
			break;
		case navPulse:
			ScheduleNavPulse(&NavPulse, WP.PulseWidthmS, WP.PulsePeriodmS);
			UsingSurveyPulse = NavPulse.Active;
			break;
		default:
			break;
		} // switch
		NextWP();
		GetNavWayPoint();
	}

	SetDesiredAltitude(WP.Pos[DownC]);

} // RefreshNavWayPoint


void GetNavWayPoint(void) {
	WPStructNV * W;
	static uint8 LastWPUpdated = 255;

	if (NavMissionUpdated || (CurrWPNo != LastWPUpdated)) {
		NavMissionUpdated = false;
		LastWPUpdated = CurrWPNo;

		if (CurrWPNo > Config.Mission.NoOfWayPoints)
			CurrWPNo = 0;

		if (CurrWPNo == 0) { // WP #0 is Origin

			SetWPHome();
			NavState = Transiting;
			F.UsingPOI = false;

		} else { // TODO: run time expansion - a little expensive!

			W = &Config.Mission.WP[CurrWPNo];
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

			WP.PulseWidthmS = W->PulseWidthmS;
			WP.PulsePeriodmS = W->PulsePeriodmS;

		}
#if defined(NAV_ENFORCE_ALTITUDE_CEILING)
		WP.Pos[DownC] = Limit(WP.Pos[DownC], 0, NAV_CEILING_M);
		WP.OrbitAltitude = Limit(WP.OrbitAltitude, 0, NAV_CEILING_M);
#endif // NAV_ENFORCE_ALTITUDE_CEILING
	}
} // GetNavWaypoint


void UpdateNavMission(void) {

	if (NavMissionSanityCheck(&NewNavMission)) {
		if (NewNavMission.NoOfWayPoints > 0)
			memcpy(&Config.Mission, &NewNavMission, sizeof(MissionStruct));
		else
			ClearNavMission();

		RefreshFence();

		memset(&NewNavMission, 0, sizeof(MissionStruct));

		NavMissionUpdated = ConfigChanged = true;

		RefreshConfig();
	}

} // DoNavMissionUpdate


