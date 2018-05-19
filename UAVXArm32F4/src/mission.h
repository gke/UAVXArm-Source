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


#ifndef _mission_h
#define _mission_h

#define NAV_MAX_WAYPOINTS 11

typedef struct {
	int32 LatitudeRaw; // 1e7/degree
	int32 LongitudeRaw;
	int16 Altitude;
	int16 VelocitydMpS;
	int16 Loiter;
	int16 OrbitRadius;
	int16 OrbitAltitude; // relative to Origin
	int16 OrbitVelocitydMpS;
	uint8 Action;
}__attribute__((packed)) WPStructNV;

typedef struct {
	int8 NoOfWayPoints;
	int8 ProximityAltitude;
	int8 ProximityRadius;
	int16 FenceRadius;
	int16 OriginAltitude;
	int32 OriginLatitude;
    int32 OriginLongitude;
	WPStructNV WP[NAV_MAX_WAYPOINTS];
}__attribute__((packed)) MissionStruct;

extern MissionStruct NewNavMission;
void ClearNavMission(void);
void CaptureHomePosition(void);
void DisplayNavMissions(uint8 s);
boolean NavMissionSanityCheck(MissionStruct * M);
uint8 NextWPState(void);
void RefreshNavWayPoint(void);
void GetNavWayPoint(void);
void DisplayNavMission(uint8 s, MissionStruct * M);
void UpdateNavMission(void);
void GenerateHomeWP(void);

extern WPStruct WP, HP, POI;
extern uint8 CurrWPNo, PrevWPNo;
extern boolean NavMissionUpdated;

#endif


