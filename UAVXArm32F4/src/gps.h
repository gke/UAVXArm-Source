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


#ifndef _gps_h
#define _gps_h

enum ubxResets {stopGNSS = 8, startGNSS = 9};

enum WaitStates {
	WaitSentinel,
	WaitSentinel2,
	WaitID,
	WaitClass,
	WaitLength,
	WaitLength2,
	WaitBody,
	WaitCheckSum,
	WaitCheckSum2,

	WaitNMEAID,
	WaitNMEABody,
	WaitNMEACheckSum,
	WaitNMEACheckSum2

};

enum GPSProtcols {
UbxM8GPSInit, UbxLegacyGPSInit, UbxGPS, NMEAGPS, NoGPS
};

typedef struct {
	int32 Raw, RawP, OriginRaw;
	real32 Pos, Vel;
} GPSCoord;

typedef struct {
	char swVersion[30];
	uint32 hwVersion;
	real32 lag;
	boolean datevalid;
	uint16 year;
	uint8 month;
	uint8 day;
	uint8 hour;
	uint8 minute;
	uint8 second;
	uint8 noofsats;
	uint8 fix;
	timemS missionTime, startTime;
	timemS lastPosUpdatemS;
	real32 altitude, relAltitude, originAltitude, geoidheight;
	GPSCoord C[3];
	real32 longitudeCorrection;
	real32 Distance, Direction;
	int8 Hint;
	real32 magHeading, magVariation;
	uint32 iTOW;
	int32 lat;
	int32 lon;
	real32 height; // above mean sea level (m)
	real32 hAcc; // horizontal accuracy est (m)
	real32 vAcc; // vertical accuracy est (m)
	real32 velN; // north velocity (m/s)
	real32 velE; // east velocity (m/s)
	real32 velD; // down velocity (m/s)
	real32 gspeed; // ground speed (m/s)
	real32 heading; // deg
	real32 sAcc; // speed accuracy est (m/s)
	real32 cAcc; // course accuracy est (deg)
	real32 pDOP; // position Dilution of Precision
	real32 hDOP;
	real32 vDOP; // zzz
	real32 tDOP;
	real32 nDOP; //zzz
	real32 eDOP; // zzz
	//real32 DOP[3];
	real32 gDOP;

	timemS TPtowMS; // timepulse time of week (ms)
	timemS lastReceivedTPtowMS;

	timemS lastTimepulse;
	timemS lastMessage;
	boolean gpsVelFlag; // gke
	//int32 microsPerSecond;
} GPSRec;

GPSRec GPS;

void UbxSaveConfig(uint8 s);

void UpdateField(void);
real32 GPSToM(int32 g);
int32 MToGPS(real32 m);
void SetGPSOrigin(void);
void ParseGPSSentence(void);
void CheckGPSUpdate(void);
void CheckGPSTimeouts(void);
void ShowGPSStatus(uint8 s);
void UbxReset(uint8 s, uint16 resetType);
void GPSISR(char ch);
void InitGPS(void);

#define MAXTAGINDEX 4
#define GPSRXBUFFLENGTH 80

extern uint8 GPSPacketTag;
extern timemS GPSdTmS;
extern timemS LastGPSUpdatemS;
extern timemS NavGPSTimeoutmS;
extern uint8 nll, cc, lo, hi;
extern boolean EmptyField;
extern real32 GPSLag;
extern real32 GPSMinhAcc, GPSMinvAcc;

extern uint8 CurrGPSType;

extern uint8 ll, ss, tt, RxCh;
extern uint8 RxCheckSum, GPSCheckSumChar, GPSTxCheckSum;

extern int16 UbxVersion;

#endif

