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

// 	GPS routines

#include "UAVX.h"

// Moving average of coordinates needed - or Kalman Estimator probably

#define GPSVelocityFilter NoFilter		// done after position filter
const uint8 NMEATags[MAX_NMEA_SENTENCES][5] = {
// NMEA
		{ 'G', 'P', 'G', 'G', 'A' }, // position fix
		{ 'G', 'P', 'R', 'M', 'C' }, // ground speed and heading
		{ 'G', 'N', 'G', 'G', 'A' }, // position fix
		{ 'G', 'N', 'R', 'M', 'C' }, // ground speed and heading
		};

uint8 CurrGPSType;

uint8 RxState = WaitSentinel;

NMEAStruct NMEA;

uint8 GPSPacketTag;

GPSRec GPS;
const uint32 GPSBaud = 115200;
const uint32 UbxGPSBaud = 115200; //230400;

real32 GPSdT;
real32 GPSLag = 1.0f; // MTK 0.5 for UBlox
real32 GPSMinhAcc = GPS_MIN_HACC;
real32 GPSMinvAcc = GPS_MIN_VACC;

timemS GPSPosUpdatemSP = 0;
timemS NavGPSTimeoutmS;

uint8 nll, cc, lo, hi, ll, ss, tt, GPSCheckSumChar;
uint8 GPSTxCheckSum, RxCheckSum;
boolean EmptyField;
int16 ValidGPSSentences;

#define DEFAULT_BAUD_RATES 7
const uint32 DefaultBaud[] =
		{ 4800, 9600, 19200, 38400, 57600, 115200, 230400 };

void TxGPSString(uint8 s, const char *pch) {
	while (*pch != (char) 0) {
		TxChar(s, *pch++);
		Delay1mS(5);
	}
} // TxGPSString

//______________________________________________________________________________

// UBlox Code rewritten from AQ (C) Bill Nesbitt

int16 UbxVersion;

#define UBX_PREAMBLE1	    0xB5	// u
#define UBX_PREAMBLE2	    0x62	// b
#define UBX_NAV_CLASS	    0x01
#define UBX_RXM_CLASS	    0x02
#define UBX_INF_CLASS		0x04
#define UBX_ACK_CLASS		0x05
#define UBX_CFG_CLASS		0x06
#define UBX_MON_CLASS		0x0a
#define UBX_AID_CLASS	    0x0b
#define UBX_TIM_CLASS	    0x0d
#define UBX_ESF_CLASS		0x10
#define UBX_LOG_CLASS		0x21

#define UBX_AID_REQ	    	0x00
#define UBX_TIM_TP	    	0x01

#define UBX_NAV_POSLLH	    0x02
#define UBX_NAV_STATUS	    0x03
#define UBX_NAV_DOP	    	0x04
#define UBX_NAV_SOL	    	0x06
#define UBX_NAV_PVT			0x07
#define UBX_NAV_VELNED	    0x12
#define UBX_NAV_TIMEUTC		0x21
#define UBX_NAV_SVINFO	    0x30
#define UBX_NAV_SBAS		0x32

#define UBX_CFG_PRT       	0x00
#define UBX_CFG_MSG			0x01
#define UBX_CFG_DAT			0x09
#define UBX_CFG_TP			0x07
#define UBX_CFG_RATE		0X08
#define UBX_CFG_CFG			0X09
#define UBX_CFG_EKF			0x12
#define UBX_CFG_SBAS		0x16
#define UBX_CFG_NAV5		0x24

#define UBX_MON_VER	    	0x04
#define UBX_MON_HW	    	0x09

#define UBX_RXM_RAW	    	0x10

#define UBX_SFRB_RAW	    0x11

const uint8 DISABLE_UBX[][2] = { // fill out Ids later ;)
		{ UBX_AID_CLASS, 0x30 }, //
				{ UBX_AID_CLASS, 0x50 }, //
				{ UBX_AID_CLASS, 0x33 }, //
				{ UBX_AID_CLASS, 0x31 }, //
				{ UBX_AID_CLASS, 0x00 }, //
				{ UBX_ESF_CLASS, 0x02 }, //
				{ UBX_ESF_CLASS, 0x02 }, //
				{ UBX_ESF_CLASS, 0x02 }, //
				{ UBX_ESF_CLASS, 0x10 }, //
				{ UBX_LOG_CLASS, 0x0E }, //
				{ UBX_LOG_CLASS, 0x08 }, //
				{ UBX_LOG_CLASS, 0x09 }, //
				{ UBX_LOG_CLASS, 0x0B }, //
				{ UBX_LOG_CLASS, 0x0F }, //
				{ UBX_LOG_CLASS, 0x0D }, //
				{ UBX_MON_CLASS, 0x05 }, //
				{ UBX_MON_CLASS, 0x09 }, //
				{ UBX_MON_CLASS, 0x0B }, //
				{ UBX_MON_CLASS, 0x02 }, //
				{ UBX_MON_CLASS, 0x06 }, //
				{ UBX_MON_CLASS, 0x07 }, //
				{ UBX_MON_CLASS, 0x21 }, //
				{ UBX_MON_CLASS, 0x2E }, //
				{ UBX_MON_CLASS, 0x08 }, //
				{ UBX_NAV_CLASS, 0x60 }, //
				{ UBX_NAV_CLASS, 0x22 }, //
				{ UBX_NAV_CLASS, 0x31 }, //
				{ UBX_NAV_CLASS, 0x04 }, //
				{ UBX_NAV_CLASS, 0x40 }, //
				{ UBX_NAV_CLASS, 0x09 }, //
				{ UBX_NAV_CLASS, 0x34 }, //
				{ UBX_NAV_CLASS, 0x01 }, //
				{ UBX_NAV_CLASS, 0x02 }, //
				{ UBX_NAV_CLASS, 0x07 }, //
				{ UBX_NAV_CLASS, 0x35 }, //
				{ UBX_NAV_CLASS, 0x32 }, //
				{ UBX_NAV_CLASS, 0x06 }, //
				{ UBX_NAV_CLASS, 0x03 }, //
				{ UBX_NAV_CLASS, 0x30 }, //
				{ UBX_NAV_CLASS, 0x24 }, //
				{ UBX_NAV_CLASS, 0x23 }, //
				{ UBX_NAV_CLASS, 0x20 }, //
				{ UBX_NAV_CLASS, 0x21 }, //
				{ UBX_NAV_CLASS, 0x11 }, //
				{ UBX_NAV_CLASS, 0x12 }, //
				{ UBX_RXM_CLASS, 0x30 }, //
				{ UBX_RXM_CLASS, 0x31 }, //
				{ UBX_RXM_CLASS, 0x10 }, //
				{ UBX_RXM_CLASS, 0x15 }, //
				{ UBX_RXM_CLASS, 0x11 }, //
				{ UBX_RXM_CLASS, 0x13 }, //
				{ UBX_RXM_CLASS, 0x20 }, //
				{ UBX_TIM_CLASS, 0x11 }, //
				{ UBX_TIM_CLASS, 0x16 }, //
				{ UBX_TIM_CLASS, 0x13 }, //
				{ UBX_TIM_CLASS, 0x04 }, //
				{ UBX_TIM_CLASS, 0x03 }, //
				{ UBX_TIM_CLASS, 0x12 }, //
				{ UBX_TIM_CLASS, 0x01 }, //
				{ UBX_TIM_CLASS, 0x06 } };

#define UBX_MAX_PAYLOAD   384
#define GPS_LATENCY	    75000	// us (comment out to use Ubx timepulse)
typedef struct {
	char swVersion[30];
	char hwVersion[10];
	char extension[30][7];
}__attribute__((packed)) UbxStructVER;

typedef struct {
	uint32 iTOW; // GPS Millisecond Time of Week (ms)
	int32 lon; // Longitude (deg * 1e-7)
	int32 lat; // Latitude (deg * 1e-7)
	int32 height; // Height above Ellipsoid (mm)
	int32 hMSL; // Height above mean sea level (mm)
	uint32 hAcc; // Horizontal Accuracy Estimate (mm)
	uint32 vAcc; // Vertical Accuracy Estimate (mm)
}__attribute__((packed)) UbxStructPOSLLH;

typedef struct {
	uint32 iTOW; // ms GPS Millisecond Time of Week
	uint16 gDOP; // Geometric DOP
	uint16 pDOP; // Position DOP
	uint16 tDOP; // Time DOP
	uint16 vDOP; // Vertical DOP
	uint16 hDOP; // Horizontal DOP
	uint16 nDOP; // Northing DOP
	uint16 eDOP; // Easting DOP
}__attribute__((packed)) UbxStructDOP;

typedef struct {
	uint32 time;
	int32 time_nsec;
	int16 week;
	uint8 fixtype;
	uint8 fix_status;
	int32 ecef_x; // cm
	int32 ecef_y; // cm
	int32 ecef_z; // cm
	uint32 position_accuracy_3d; // cm
	int32 ecef_x_velocity; // cm/S
	int32 ecef_y_velocity; // cm/S
	int32 ecef_z_velocity; // cm/S
	uint32 speed_accuracy; // cm/S
	uint16 position_DOP;
	uint8 res;
	uint8 satellites;
	uint32 res2;
}__attribute__((packed)) UbxStructSOL;

typedef struct {
	uint8 svid;
	uint8 flags;
	uint8 udre;
	uint8 svSys;
	uint8 svService;
	uint8 res0;
	int16 prc; // cm
	int16 res1;
	int16 ic; // cm
}__attribute__((packed)) SVStruct;

typedef struct {
	uint32 iTOW;
	uint8 geo;
	uint8 mode;
	int8 sys;
	uint8 service;
	uint8 cnt;
	uint8 res[3];
	SVStruct SV[16];
}__attribute__((packed)) UbxStructSBAS;

typedef struct {
	uint32 iTOW;
	uint8 fixtype;
	uint8 fix_status;
	uint8 flags2;
	uint32 ttff;
	uint32 msss;
}__attribute__((packed)) UbxStructSTATUS;

typedef struct {
	uint32 iTOW; // GPS Millisecond Time of Week (ms)
	int32 velN; // NED north velocity (cm/s)
	int32 velE; // NED east velocity (cm/s)
	int32 velD; // NED down velocity (cm/s)
	uint32 speed; // Speed (3-D) (cm/s)
	uint32 gSpeed; // Ground Speed (2-D) (cm/s)
	int32 heading; // Heading 2-D (deg * 1e-5)
	uint32 sAcc; // Speed Accuracy Estimate (cm/s)
	uint32 cAcc; // Course / Heading Accuracy Estimate (deg * 1e-5)
}__attribute__((packed)) UbxStructVALNED;

typedef struct {
	uint32 towMS;
	uint32 towSubMS;
	int32 qErr;
	uint16 week;
	uint8 flags;
	uint8 res;
}__attribute__((packed)) UbxStructTP;

typedef struct {
	uint32 iTOW; // GPS Millisecond Time of Week (ms)
	uint32 tAcc; // Time Accuracy Estimate
	int32 nano; // Nanosecond of second (UTC)
	uint16 year; // Year, range 1999..2099 (UTC)
	uint8 month; // Month, range 1..12 (UTC)
	uint8 day; // Day of Month, range 1..31 (UTC)
	uint8 hour; // Hour of Day, range 0..23 (UTC)
	uint8 min; // Minute of Hour, range 0..59 (UTC)
	uint8 sec; // Second of Minute, range 0..59 (UTC)
	uint8 valid; // Validity Flags
}__attribute__((packed)) UbxStructTIMEUTC;

typedef struct {
	uint32 iTOW; // GPS Time of Week [ms]
	uint16 year; // Year (UTC)
	uint8 month; // Month, range 1..12 (UTC)
	uint8 day; // Day of month, range 1..31 (UTC)
	uint8 hour; // Hour of day, range 0..23 (UTC)
	uint8 min; // Minute of hour, range 0..59 (UTC)
	uint8 sec;// Seconds of minute, range 0..60 (UTC)
	uint8 valid; // Validity flags (see UBX_RX_NAV_PVT_VALID_...)
	uint32 tAcc; // Time accuracy estimate (UTC) [ns]
	int32 nano; // Fraction of second (UTC) [-1e9...1e9 ns]
	uint8 fixtype; // GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix
	uint8 flags1; // Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...)
	uint8 flags2;
	uint8 numSV; // Number of SVs used in Nav Solution
	int32 lon; // Longitude [1e-7 deg]
	int32 lat; // Latitude [1e-7 deg]
	int32 height;// Height above ellipsoid [mm]
	int32 hMSL;// Height above mean sea level [mm]
	uint32 hAcc; // Horizontal accuracy estimate [mm]
	uint32 vAcc; // Vertical accuracy estimate [mm]
	int32 velN;// NED north velocity [mm/s]
	int32 velE;// NED east velocity [mm/s]
	int32 velD;// NED down velocity [mm/s]
	int32 gSpeed;// Ground Speed (2-D) [mm/s]
	int32 headMot; // Heading of motion (2-D) [1e-5 deg]
	uint32 sAcc; // Speed accuracy estimate [mm/s]
	uint32 cAcc; // Heading accuracy estimate (motion and vehicle) [1e-5 deg]
	uint16 pDOP; // Position DOP [0.01]
	uint16 reserved2;
	uint32 reserved3;
	int32 headVeh; //(ubx8+ only) Heading of vehicle (2-D) [1e-5 deg]
	uint32 reserved4; //(ubx8+ only)
}__attribute__((packed)) UbxStructPVT;

struct {
	uint8 state;
	uint8 count;
	uint8 class;
	uint8 id;
	uint8 length;
	union {
		UbxStructVER ver;
		UbxStructPOSLLH posllh;
		UbxStructVALNED valned;
		UbxStructSOL sol;
		UbxStructPVT pvt;
		UbxStructSTATUS status;
		UbxStructDOP dop;
		UbxStructTP tp;
		UbxStructTIMEUTC timeutc;
		char other[UBX_MAX_PAYLOAD];
	} payload;

	uint8 RxCK_A;
	uint8 RxCK_B;

	uint8 TxUbxCK_A;
	uint8 TxUbxCK_B;
}__attribute__((packed)) ubx;

void rtcSetDataTime(int32 year, int32 month, int32 day, int32 hour,
		int32 minute, int32 second) {
	// No RTC for UAVXArm
} // rtcSetDataTime

void TxUbxu8(uint8 s, uint8 c) {
	TxChar(s, c);
	ubx.TxUbxCK_A += c;
	ubx.TxUbxCK_B += ubx.TxUbxCK_A;
} // TxUbxu8

void UbxWriteI1(uint8 s, int8 c) {
	TxChar(s, (uint8) c);
	ubx.TxUbxCK_A += c;
	ubx.TxUbxCK_B += ubx.TxUbxCK_A;
} // UbxWriteI1

void TxUbxu16(uint8 s, uint16 x) {
	TxUbxu8(s, x);
	TxUbxu8(s, x >> 8);
} // TxUbxu16

void UbxWriteI2(uint8 s, int16 x) {
	TxUbxu8(s, x);
	TxUbxu8(s, x >> 8);
} // UbxWriteI2

void TxUbxu32(uint8 s, uint32 x) {
	TxUbxu8(s, x);
	TxUbxu8(s, x >> 8);
	TxUbxu8(s, x >> 16);
	TxUbxu8(s, x >> 24);
} // TxUbxu32

void UbxWriteI4(uint8 s, int32 x) {
	TxUbxu8(s, x);
	TxUbxu8(s, x >> 8);
	TxUbxu8(s, x >> 16);
	TxUbxu8(s, x >> 24);
} // UbxWriteI4

void UbxSendPreamble(uint8 s) {

	TxUbxu8(s, UBX_PREAMBLE1); // u
	TxUbxu8(s, UBX_PREAMBLE2); // b

	ubx.TxUbxCK_A = 0;
	ubx.TxUbxCK_B = 0;
} // UbxSendPreamble

void TxUbxCheckSum(uint8 s) {

	TxChar(s, ubx.TxUbxCK_A);
	TxChar(s, ubx.TxUbxCK_B);

	TxNextLine(s); // for debug
} // TxUbxCheckSum

void UbxEnableMessage(uint8 s, uint8 Class, uint8 ID, uint8 Rate) {

	UbxSendPreamble(s);
	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_MSG);
	TxUbxu16(s, 3);
	TxUbxu8(s, Class);
	TxUbxu8(s, ID);
	TxUbxu8(s, Rate);
	TxUbxCheckSum(s);

	Delay1mS(50);
} // UbxEnableMessage


void UbxDisableNavMessages(uint8 s) {
	uint16 i;

	for (i = 0; i < sizeof(DISABLE_UBX) / 2; i++)
		UbxEnableMessage(s, DISABLE_UBX[i][0], DISABLE_UBX[i][1], 0);

} // UbxDisableNavMessages

void UbxSetInterval(uint8 s, uint16 Interval) {

	UbxSendPreamble(s);
	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_RATE);
	TxUbxu16(s, 6);
	TxUbxu16(s, Interval);
	TxUbxu16(s, 0x01); // cycles
	TxUbxu16(s, 0x01); // use GPS time
	TxUbxCheckSum(s);

	Delay1mS(50);
} // UbxSetInterval

void UbxSaveConfig(uint8 s) {
	enum clearMask { // beware 8M flags extended
		ioport = _b0,
		msgConf = _b1,
		infMsg = _b2,
		navConf = _b3,
		rxmConf = _b4,
		senConf = _b8,// M8
		rinvConf = _b9,// M8
		antConf = _b10,
		logConf = _b11,// M8
		ftsConf = _b12
	// M8
	};
	enum devMask {
		devBBR = _b0,
		devFlash = _b1,
		devEEPROM = _b2,
		devReserved = _b3,
		devSPIFlash = _b4
	};

	UbxSendPreamble(s);
	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_CFG);
	TxUbxu16(s, 13);
	TxUbxu32(s, 0); // clear mask
	TxUbxu32(s, ioport | msgConf | infMsg | navConf | rxmConf | antConf); // save mask
	TxUbxu32(s, 0); // load mask
	TxUbxu8(s, devEEPROM | devFlash | devBBR);
	TxUbxCheckSum(s);

	Delay1mS(1000);

} // UbxSaveConfig


void UbxSetSBAS(uint8 s, uint8 enable) {

	enum Usage {
		range = _b0, diffcor = _b1, integrity = _b2
	};
	UbxSendPreamble(s);
	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_SBAS);
	TxUbxu16(s, 8);
	TxUbxu8(s, enable); // disable in USA?
	TxUbxu8(s, diffcor | integrity);
	TxUbxu8(s, 3); // maxSBAS 3 was 0
	TxUbxu8(s, 0); // scan mode 2
	TxUbxu32(s, 0); // scan mode 1
	TxUbxCheckSum(s);

	Delay1mS(50);
} // UbxSetSBAS

void UbxSetMode(uint8 s) {

	enum Masks {
		dyn = _b0,
		minEl = _b1,
		fixMode = _b2,
		drLim = _b3,
		posMask = _b4,
		timeMask = _b5,
		staticHold = _b6,
		dgpsMask = _b7
	};

	enum DynamicModel {
		Portable = 0,
		Stationary = 2,
		Pedestrian = 3,
		Automotive = 4,
		Sea = 5,
		Airborne1G = 6,
		Airborne2g = 7,
		Airborne4G = 8
	};

	enum FixTypes {
		Fix2D = 1, ThreeD = 2, Automatic = 3
	};

	UbxSendPreamble(s);
	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_NAV5);
	TxUbxu16(s, 36);
	TxUbxu8(s, fixMode | dyn); // mask LSB (fixMode, dyn)
	TxUbxu8(s, 0); // mask MSB (reserved)
	if (F.IsFixedWing)
		TxUbxu8(s, Airborne1G);
	else
		TxUbxu8(s, Pedestrian);
	TxUbxu8(s, Automatic); // ThreeD
	TxUbxu32(s, 0); // altitude
	TxUbxu32(s, 10000); // 0.0001 altitude variance for 2D

	TxUbxu8(s, 5); // min elev deg (i8)
	TxUbxu8(s, 0); // dead reckoning limit s
	TxUbxu16(s, 250); // pdop x 0.1
	TxUbxu16(s, 250); // tdop x 0.1
	TxUbxu16(s, 100); // pacc m
	TxUbxu16(s, 300); // tacc m
	TxUbxu8(s, 0); // hold threshold cm/S
	TxUbxu8(s, 60); // dgps timeout was 0
	TxUbxu32(s, 0); // reserved
	TxUbxu32(s, 0); // reserved
	TxUbxu32(s, 0); // reserved

	TxUbxCheckSum(s);

	Delay1mS(50);
} // UbxSetMode

void UbxInitPort(uint8 s) {

	enum portMask {
		inUBX = _b0, inNMEA = _b1, inRCTM = _b2, inRCTM3 = _b5
	};
	UbxSendPreamble(s);

	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_PRT);

	TxUbxu8(s, 0x14); // length lsb
	TxUbxu8(s, 0x00); // length msb

	TxUbxu8(s, 0x01); // portId
	TxUbxu8(s, 0x00); // reserved
	TxUbxu16(s, 0x00); // txRead
	TxUbxu32(s, 0x000008D0); // mode
	TxUbxu32(s, UbxGPSBaud); // baudRate
	TxUbxu16(s, inRCTM | inNMEA | inUBX); // inProtoMask
	TxUbxu16(s, inUBX); // outProtoMask
	TxUbxu16(s, 0x00); // flags
	TxUbxu16(s, 0x00); // reserved
	TxUbxCheckSum(s);

	Delay1mS(400);
}

void UbxSetTimepulse(uint8 s) {

	UbxSendPreamble(s);
	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_TP);
	TxUbxu16(s, 20);
	TxUbxu32(s, 1000000); // interval (us)
	TxUbxu32(s, 100000); // length (us)
#if defined(GPS_LATENCY)
	UbxWriteI1(s, 0x00); // config setting (0 == off)
#else
	UbxWriteI1(s, 0x01); // config setting (1 == +polarity)
#endif
	TxUbxu8(s, 0x01); // alignment reference time (GPS)
	TxUbxu8(s, 0x00); // bit mask (sync mode 0)
	TxUbxu8(s, 0x00); // reserved
	UbxWriteI2(s, 0x00); // antenna delay
	UbxWriteI2(s, 0x00); // rf group delay
	UbxWriteI4(s, 0x00); // user delay
	TxUbxCheckSum(s);

	Delay1mS(50);
} // UbxSetTimepulse

void UbxPollVersion(uint8 s) {
	UbxSendPreamble(s);

	TxUbxu8(s, UBX_MON_CLASS);
	TxUbxu8(s, UBX_MON_VER);

	TxUbxu8(s, 0x00); // length lsb
	TxUbxu8(s, 0x00); // length msb

	TxUbxCheckSum(s);

} // UbxPollVersion


void ParseUbxPacket(void) {

	enum UbxFixTypes {
		FixNone = 0,
		FixDeadReckoning = 1,
		Fix2D = 2,
		Fix3D = 3,
		FixGPSDeadReckoning = 4,
		FixTime = 5
	};

	switch (ubx.class) {
	case UBX_NAV_CLASS:
		switch (ubx.id) {
		case UBX_NAV_PVT:
			GPS.missionTime = GPS.lastPosUpdatemS = GPS.lastVelUpdatemS
					= mSClock(); //ubx.payload.pvt.iTOW;
			GPS.year = ubx.payload.pvt.year;
			GPS.month = ubx.payload.pvt.month;
			GPS.day = ubx.payload.pvt.day;
			GPS.hour = ubx.payload.pvt.hour;
			GPS.minute = ubx.payload.pvt.min;
			GPS.second = ubx.payload.pvt.sec;
			GPS.datevalid = ubx.payload.pvt.valid != 0;
			// = ubx.payload.pvt.tAcc;
			// = ubx.payload.pvt.nano;
			GPS.fix = ubx.payload.pvt.fixtype;
			// = ubx.payload.pvt.flags1;
			// = ubx.payload.pvt.flags2;
			GPS.noofsats = ubx.payload.pvt.numSV;
			GPS.lon = GPS.C[EastC].Raw = ubx.payload.pvt.lon;
			GPS.lat = GPS.C[NorthC].Raw = ubx.payload.pvt.lat;
			// = ubx.payload.pvt.height; // Height above ellipsoid [mm]
			GPS.height = GPS.altitude = ubx.payload.pvt.hMSL * 0.001f; // mm => m
			GPS.hAcc = ubx.payload.pvt.hAcc * 0.001f; // mm/s => m/s
			GPS.vAcc = ubx.payload.pvt.vAcc * 0.001f; // mm/s => m/s
			GPS.velN = GPS.C[NorthC].Vel = ubx.payload.pvt.velN * 0.001f; // mm => m
			GPS.velE = GPS.C[EastC].Vel = ubx.payload.pvt.velE * 0.001f; // mm => m
			GPS.velD = GPS.C[DownC].Vel = ubx.payload.pvt.velD * 0.001f; // mm => m
			GPS.gspeed = ubx.payload.pvt.gSpeed * 0.001f; // mm/s => m/s
			GPS.heading = DegreesToRadians(ubx.payload.pvt.headMot * 1e-5f);
			GPS.sAcc = ubx.payload.pvt.sAcc * 0.001f; // mm/s => m/s
			GPS.cAcc = ubx.payload.pvt.cAcc * 1e-5f;
			// = ubx.payload.pvt.pDOP;// Position DOP [0.01]
			// = ubx.payload.pvt.reserved2;
			// = ubx.payload.pvt.reserved3;
			// = ubx.payload.pvt.headVeh; // (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg]
			// = ubx.payload.pvt.reserved4; // (ubx8+ only)
			break;
		case UBX_NAV_STATUS:
			// time
			GPS.fix = ubx.payload.status.fixtype;
			// flags2
			// tttf
			// msss
			break;

		case UBX_NAV_SOL:
			// time
			// time_nsec
			// week
			GPS.fix = ubx.payload.sol.fixtype;
			// ecef_x
			// ecef_y
			// ecef_z
			// position_accuracy_3d
			// ecef_x_velocity
			// ecef_y_velocity
			// ecef_z_velocity
			GPS.sAcc = ubx.payload.sol.speed_accuracy * 0.01f; // cm/S

			// position_DOP
			// res
			GPS.noofsats = ubx.payload.sol.satellites;
			// res2
			break;
		case UBX_NAV_POSLLH:
			GPS.missionTime = GPS.lastPosUpdatemS = mSClock(); //ubx.payload.posllh.iTOW;
			GPS.lat = GPS.C[NorthC].Raw = ubx.payload.posllh.lat;
			GPS.lon = GPS.C[EastC].Raw = ubx.payload.posllh.lon;
			GPS.height = GPS.altitude = ubx.payload.posllh.hMSL * 0.001f; // mm => m
			GPS.hAcc = ubx.payload.posllh.hAcc * 0.001f; // mm => m
			GPS.vAcc = ubx.payload.posllh.vAcc * 0.001f; // mm => m
			break;
		case UBX_NAV_VELNED:
			GPS.lastVelUpdatemS = mSClock(); //ubx.payload.valned.iTOW;
			GPS.velN = GPS.C[NorthC].Vel = ubx.payload.valned.velN * 0.01f; // cm => m
			GPS.velE = GPS.C[EastC].Vel = ubx.payload.valned.velE * 0.01f; // cm => m
			GPS.velD = ubx.payload.valned.velD * 0.01f; // cm => m
			GPS.gspeed = ubx.payload.valned.gSpeed * 0.01f; // cm/s => m/s
			GPS.heading = DegreesToRadians(ubx.payload.valned.heading * 1e-5f);
			GPS.sAcc = ubx.payload.valned.sAcc * 0.01f; // cm/s => m/s
			GPS.cAcc = ubx.payload.valned.cAcc * 1e-5f;
			break;
		case UBX_NAV_DOP:
			GPS.pDOP = ubx.payload.dop.pDOP * 0.01f;
			GPS.hDOP = ubx.payload.dop.hDOP * 0.01f;
			GPS.vDOP = ubx.payload.dop.vDOP * 0.01f;
			GPS.tDOP = ubx.payload.dop.tDOP * 0.01f;
			GPS.nDOP = ubx.payload.dop.nDOP * 0.01f;
			GPS.eDOP = ubx.payload.dop.eDOP * 0.01f;
			GPS.gDOP = ubx.payload.dop.gDOP * 0.01f;
			break;
		case UBX_NAV_TIMEUTC:
			if (ubx.payload.timeutc.valid & 0b100) {
				rtcSetDataTime(ubx.payload.timeutc.year,
						ubx.payload.timeutc.month, ubx.payload.timeutc.day,
						ubx.payload.timeutc.hour, ubx.payload.timeutc.min,
						ubx.payload.timeutc.sec);

				GPS.year = ubx.payload.timeutc.year;
				GPS.month = ubx.payload.timeutc.month;
				GPS.day = ubx.payload.timeutc.day;

				UbxEnableMessage(GPSSerial, UBX_NAV_CLASS, UBX_NAV_TIMEUTC, 0); // disable message
			}
			break;
		case UBX_NAV_SVINFO:

			break;
		case UBX_NAV_SBAS:
			// zzz use field names direct??
			// UbxEnableMessage(GPSSerial, UBX_NAV_CLASS, UBX_NAV_SBAS, 0); // disable message
			break;
		default:
			break;
		}
		break;
	case UBX_MON_CLASS:
		switch (ubx.id) {
		case UBX_MON_HW:

			break;
		case UBX_MON_VER:
			//unused UbxVersion = atoi(ubx.payload.ver.hwVersion) / 10000;
			break;
		default:
			break;
		} // switch
		break;
	case UBX_TIM_CLASS:
		switch (ubx.id) {
		case UBX_TIM_TP:
			GPS.TPtowMS = ubx.payload.tp.towMS;
			break;
		default:
			break;
		}
	default:
		break;
	}

	F.ValidGPSVel = GPS.sAcc <= GPS_MIN_SACC;
	F.ValidGPSHeading = GPS.cAcc <= GPS_MIN_CACC;

	F.GPSValid = F.ValidGPSVel && ( (ubx.payload.pvt.fixtype == Fix3D)
			|| (ubx.payload.pvt.fixtype == Fix2D)
			|| (ubx.payload.pvt.fixtype == FixGPSDeadReckoning));

	GPS.lastMessage = uSClock() * 0.000001f;

} // ParseUbxPacket

void RxUbxCheckSum(uint8 c) {
	ubx.RxCK_A += c;
	ubx.RxCK_B += ubx.RxCK_A;
} // RxUbxCheckSum


void InitUbxGPS(uint8 s, int16 UbxVersion) {
	uint16 i, retries;

	//Black GPS BD
	//SW=<2.01 (75331)>
	//HW=<00080000>
	//Ver=8

	//White uBloxNEO-6M
	//SW=<7.03 (45969)>
	//HW=<00040007>
	//Ver=4

	//Ancient 1.575R-A Z NEO-6M-0-001
	//SW=<7.03 (45969)>
	//HW=<00040007>
	//Ver=4

	//Ancient 1.575R-A Z NEO-6M-0-001 (small antenna)
	//SW=<7.03 (45969)>
	//HW=<00040007>
	//Ver=4

	LEDOn(ledYellowSel);

	RxEnabled[GPSSerial] = false;

	for (i = 0; i < DEFAULT_BAUD_RATES; i++) {
		SetBaudRate(s, DefaultBaud[i]);
		UbxInitPort(s); // yell at it twice!
		UbxInitPort(s);
		LEDToggle(ledBlueSel);
	}
	LEDOff(ledBlueSel);

	SetBaudRate(s, UbxGPSBaud);

	Delay1mS(1000);

	UbxDisableNavMessages(s);

	RxEnabled[GPSSerial] = true;

	UbxSetTimepulse(s);

	UbxSetMode(s); // dynamic filter etc

	if (UbxVersion == 8) {
		UbxSetInterval(s, 100);
		UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_PVT, 1);
	} else {
		UbxSetInterval(s, 200);
		UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_SOL, 1); // for fix and # of sats
		UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_VELNED, 1);
		UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_POSLLH, 1);
		// Ken reports problems UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_TIMEUTC, 255);
	}

	//UbxSetSBAS(s, UbxVersion != 7); // v1 broken

	//	UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_DOP, 1);
	//	UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_SVINFO, 5);
	//	UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_SAT, 5);
	//	UbxEnableMessage(s, UBX_MON_CLASS, UBX_MON_HW, 1);

	UbxSaveConfig(s);

	F.GPSPacketReceived = false;

	GPS.lag = 0.5f;

	LEDOff(ledYellowSel);

} // InitUbxGPS

//___________________________________________________________________________________

// NMEA Decoder

real32 GPSToM(int32 c) {
	//return ((real64) c * 0.011131948079f);
	return ((real64) c * ((real64) EARTH_RADIUS_M * PI) / (180.0 * 1e7));
} // GPSToM

int32 MToGPS(real32 c) {
	return ((real64) c / (((real64) EARTH_RADIUS_M * PI) / (180.0 * 1e7)));
} // MToGPS

int32 I32(uint8 lo, uint8 hi) {
	idx i;
	int32 r;

	r = 0;
	if (!EmptyField)
		for (i = lo; i <= hi; i++)
			r = r * 10 + NMEA.s[i] - '0';

	return (r);
} // I32

int32 ConvertLatLon(uint8 lo, uint8 hi) {
	int32 dd, mm, dm, r;
	idx dp;

	r = 0;
	if (!EmptyField) {
		dp = lo + 4;
		while (NMEA.s[dp] != '.')
			dp++;

		dd = I32(lo, dp - 3);
		mm = I32(dp - 2, dp - 1);
		if ((hi - dp) > (uint8) 4)
			dm = I32(dp + 1, dp + 5);
		else
			dm = I32(dp + 1, dp + 4) * 10L;

		r = dd * 10000000;
		r += (mm * 10000000 + dm * 100 + 30) / 60;
	}

	return (r);
} // ConvertLatLon


int32 ConvertUTime(uint8 lo, uint8 hi) {
	int32 ival;

	ival = 0;
	if (!EmptyField)
		ival = (int32) (I32(lo, lo + 1)) * 3600 + (int32) (I32(lo + 2, lo + 3)
				* 60) + (int32) (I32(lo + 4, hi));

	return (ival);
} // ConvertUTime

void UpdateField(void) {
	uint8 ch;

	lo = cc;

	ch = NMEA.s[cc];
	while ((ch != ',') && (ch != '*') && (cc < nll))
		ch = NMEA.s[++cc];

	hi = cc - 1;
	cc++;
	EmptyField = hi < lo;
} // UpdateField

void ParseGXGGASentence(void) { // full position $GXGGA fix

	cc = 0;
	nll = NMEA.length;

	UpdateField();

	UpdateField(); //UTime
	GPS.missionTime = GPS.lastPosUpdatemS =  mSClock(); // ConvertUTime(lo, hi);

	UpdateField(); //Lat
	GPS.C[NorthC].Raw = ConvertLatLon(lo, hi);
	UpdateField(); //LatH
	if (NMEA.s[lo] == 'S')
		GPS.C[NorthC].Raw = -GPS.C[NorthC].Raw;

	UpdateField(); //Lon
	GPS.C[EastC].Raw = ConvertLatLon(lo, hi);
	UpdateField(); //LonH
	if (NMEA.s[lo] == 'W')
		GPS.C[EastC].Raw = -GPS.C[EastC].Raw;

	UpdateField(); //Fix
	GPS.fix = (uint8) (I32(lo, hi));

	UpdateField(); //Sats
	GPS.noofsats = (uint8) (I32(lo, hi));

	UpdateField(); // HDOP
	GPS.hDOP = (real32) I32(lo, hi - 3) + (real32) (I32(hi - 1, hi)) * 0.01;
	GPS.hAcc = GPS.vAcc = GPS.hDOP * GPS_HDOP_TO_HACC;

	UpdateField(); // Alt
	GPS.altitude = (real32) (I32(lo, hi - 2)) + (real32) (I32(hi, hi)) * 0.1f;

	//UpdateField();   // AltUnit - assume Metres!

	//UpdateField();   // GHeight
	//GPS.geoidheight = (real32) (I32(lo, hi - 2)) + (real32) (I32(hi, hi)) * 0.1f;
	//UpdateField();   // GHeightUnit

	F.GPSValid = (GPS.fix > 0) && (GPS.noofsats >= GPS_MIN_SATELLITES)
			&& F.ValidGPSVel;

} // ParseGXGGASentence


void ParseGXRMCSentence() { // main current position and heading
	idx a;

	cc = 0;
	nll = NMEA.length;

	UpdateField();

	UpdateField(); //UTime

	UpdateField();
	if (NMEA.s[lo] == 'A') {

		GPS.missionTime = GPS.lastPosUpdatemS = GPS.lastVelUpdatemS = mSClock(); //ConvertUTime(lo, hi);

		UpdateField(); //Lat
		GPS.C[NorthC].Raw = ConvertLatLon(lo, hi);
		UpdateField(); //LatH
		if (NMEA.s[lo] == 'S')
			GPS.C[NorthC].Raw = -GPS.C[NorthC].Raw;

		UpdateField(); //Lon
		GPS.C[EastC].Raw = ConvertLatLon(lo, hi);
		UpdateField(); //LonH
		if (NMEA.s[lo] == 'W')
			GPS.C[EastC].Raw = -GPS.C[EastC].Raw;

		UpdateField(); // Groundspeed (Knots)
		GPS.gspeed = ((real32) I32(lo, hi - 3) + (real32) I32(hi - 1, hi)
				* 0.01) * 0.5144444; //  MPS/Kt
		GPS.sAcc = GPS_MIN_SACC;

		UpdateField(); // True course made good (Degrees)
		GPS.heading = DegreesToRadians((real32)I32(lo, hi - 3) + (real32)I32(
						hi - 1, hi) * 0.01);
		GPS.cAcc = GPS_MIN_CACC;

		UpdateField();
		GPS.day = I32(lo, lo + 1);
		GPS.month = I32(lo + 2, lo + 3);
		GPS.year = I32(lo + 4, lo + 5) + 2000;

		// usually returns zero - not used
		// UpdateField();
		// GPS.magvariation = DegreesToRadians((real32) I32(lo, hi) * 0.1f);
		// UpdateField(); // magvar sign
		// if (NMEA.s[lo] == 'W')
		// GPS.magvariation = -GPS.magvariation;

		for (a = NorthC; a <= EastC; a++) {
			GPS.C[a].Vel = GPSToM(GPS.C[a].Raw - GPS.C[a].RawP) / GPSdT;
			GPS.C[a].RawP = GPS.C[a].Raw;
		}

		F.ValidGPSVel = GPS.sAcc <= GPS_MIN_SACC;

	}

} // ParseGXRMCSentence


boolean GPSSanityCheck(void) {
	boolean r;

	r = F.Emulation ? true : (Abs(GPS.C[NorthC].Raw) <= 900000000L)
			&& (Abs(GPS.C[EastC].Raw) <= 1800000000L) && (GPS.C[NorthC].Raw
			!= 0) && (GPS.C[EastC].Raw != 0);

	return (r);

} // GPSSanityCheck

void ProcessGPSSentence(void) {
	int32 a;

	if (F.GPSValid && GPSSanityCheck()) {
		if (F.OriginValid) {
			if (!F.Emulation) {
				for (a = NorthC; a <= EastC; a++)
					GPS.C[a].Pos = GPSToM(GPS.C[a].Raw - GPS.C[a].OriginRaw);
				GPS.C[EastC].Pos *= GPS.longitudeCorrection;
			}
		}
	}

	if (F.GPSValid) {

		GPSdT = (real32) (GPS.lastPosUpdatemS - GPSPosUpdatemSP) * 0.001f;
		GPSPosUpdatemSP = GPS.lastPosUpdatemS;

		F.NewGPSPosition = GPSOK();

		if ((State == Ready) || (State == Landed)) {
			LEDOn(ledBlueSel);
			if (F.NewGPSPosition)
				LEDOff(ledYellowSel);
			else
				LEDToggle(ledYellowSel);
		}
		mSTimer(GPSTimeoutmS, NavGPSTimeoutmS);
	}

} // ProcessGPSSentence


void RxGPSUbxPacket(char c) {

	switch (RxState) {
	case WaitSentinel:
		if (c == UBX_PREAMBLE1)
			RxState = WaitSentinel2;
		break;
	case WaitSentinel2:
		RxState = (c == UBX_PREAMBLE2) ? WaitClass : WaitSentinel;
		break;
	case WaitClass:
		ubx.class = c;
		ubx.RxCK_A = ubx.RxCK_B = 0;
		RxUbxCheckSum(c);
		RxState = WaitID;
		break;
	case WaitID:
		ubx.id = c;
		RxUbxCheckSum(c);
		RxState = WaitLength;
		break;
	case WaitLength:
		ubx.length = c;
		RxUbxCheckSum(c);
		RxState = WaitLength2;
		break;
	case WaitLength2:
		ubx.length += (c << 8);
		RxUbxCheckSum(c);
		if (ubx.length > 0) {
			ubx.count = 0;
			RxState = WaitBody;
		} else
			RxState = WaitCheckSum;
		break;
	case WaitBody:
		*((uint8 *) (&ubx.payload) + ubx.count) = c;
		if (++ubx.count == ubx.length)
			RxState = WaitCheckSum;
		RxUbxCheckSum(c);
		break;
	case WaitCheckSum:
		RxState = (c == ubx.RxCK_A) ? WaitCheckSum2 : WaitSentinel;
		break;
	case WaitCheckSum2:
		RxState = WaitSentinel;
		F.GPSPacketReceived = c == ubx.RxCK_B;
		if (F.GPSPacketReceived)
			ParseUbxPacket();
		break;
	} // switch

} // RxGPSUbxPacket

void RxGPSNMEAPacket(char c) {

	switch (RxState) {
	case WaitSentinel:
		if (c == '$') {
			ll = tt = ss = RxCheckSum = 0;
			RxState = WaitNMEAID;
		}
		break;
	case WaitNMEAID:
		RxCheckSum ^= c;
		while ((c != NMEATags[ss][tt]) && (ss < MAX_NMEA_SENTENCES))
			ss++;
		if (c == NMEATags[ss][tt])
			if (tt == NMEA_TAG_INDEX) {
				GPSPacketTag = ss;
				RxState = WaitNMEABody;
			} else
				tt++;
		else
			RxState = WaitSentinel;
		break;
	case WaitNMEABody:
		if (c == '*')
			RxState = WaitNMEACheckSum;
		else if (c == '$') {
			ll = tt = RxCheckSum = 0;
			RxState = WaitNMEAID;
		} else {
			RxCheckSum ^= c;
			NMEA.s[ll++] = c;
			if (ll > (GPSRXBUFFLENGTH - 1))
				RxState = WaitSentinel;
		}
		break;
	case WaitNMEACheckSum:
		if (c >= 'A')
			GPSTxCheckSum = c - ('A' - 10);
		else
			GPSTxCheckSum = c - '0';
		RxState = WaitNMEACheckSum2;
		break;
	case WaitNMEACheckSum2:
		GPSTxCheckSum *= 16;
		if (c >= 'A')
			GPSTxCheckSum += c - ('A' - 10);
		else
			GPSTxCheckSum += c - '0';
		NMEA.length = ll;
		F.GPSPacketReceived = GPSTxCheckSum == RxCheckSum;
		if (F.GPSPacketReceived)
			switch (GPSPacketTag) {
			case GPGGAPacketTag:
			case GNGGAPacketTag:
				ParseGXGGASentence();
				break;
			case GPRMCPacketTag:
			case GNRMCPacketTag:
				ParseGXRMCSentence();
				break;
			default:
				F.GPSPacketReceived = false;
			} // switch

		RxState = WaitSentinel;
		break;
	} // switch

} // RxGPSNMEAPacket


boolean GPSOK(void) {
	return F.GPSValid && (F.OriginValid || (F.IsFixedWing
			&& F.OffsetOriginValid));
} // GPSOK

void CheckGPSTimeouts(void) {

	if (F.Emulation) {
		if ((ArmingMethod != SwitchArming) || (Armed()))
			GPSEmulation(); // real GPS preferably unplugged
	} else {

		if ((mSClock() - GPS.lastPosUpdatemS) > 1000)
			F.GPSValid = F.ValidGPSVel =false; // stale

		if (mSTimeout(GPSTimeoutmS)) {
			ZeroNavCorrections();
			F.GPSValid = F.ValidGPSVel = F.NewGPSPosition = F.NavigationEnabled = false;

			LEDOff(ledBlueSel);
			LEDOff(ledYellowSel);
			mSTimer(GPSTimeoutmS, NavGPSTimeoutmS);
		}
	}

} // CheckGPSTimeouts


void GPSISR(char ch) {

	if (!F.Emulation && F.HaveGPS) {

		switch (CurrGPSType) { // 1.5uS/char
		case UbxM8GPSInit:
		case UbxLegacyGPSInit:
		case UbxGPS:
			RxGPSUbxPacket(ch);
			break;
		case NMEAGPS:
			RxGPSNMEAPacket(ch);
			break;
		case NoGPS:
		default:
			break;
		} // switch

		if (F.GPSPacketReceived) {
			F.GPSPacketReceived = false;
			ProcessGPSSentence(); // ~8.5uS
		}
	}

	CheckGPSTimeouts();

} // GPSISR


void InitGPS(void) {

	cc = 0;
	F.OriginValid = F.OffsetOriginValid = F.GPSValid = F.GPSPacketReceived
			= false;

	if (F.HaveGPS) {
		RxState = WaitSentinel;

		SetBaudRate(GPSSerial, GPSBaud);
		RxEnabled[GPSSerial] = true;

		switch (CurrGPSType) {
		case UbxM8GPSInit:
				InitUbxGPS(GPSSerial, 8);
			break;
		case UbxLegacyGPSInit:
			InitUbxGPS(GPSSerial, 0);
			break;
		case UbxGPS:
		case NMEAGPS:
		case NoGPS:
		default:
			break;
		} // switch
	}

} // InitGPS


