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

uint8 CurrGPSType;

uint8 RxState = WaitSentinel;

uint8 GPSPacketTag;

boolean UseSBAS = false; // false for USA?

GPSRec GPS;
const uint32 GPSBaud = 115200;
const uint32 UbxGPSBaud = 115200; //230400;

timemS GPSdTmS;
real32 GPSLag = 1.0f; // MTK 0.5 for UBlox
real32 GPSMinhAcc = GPS_MIN_HACC;
real32 GPSMinvAcc = GPS_MIN_VACC;

timemS NavGPSTimeoutmS;

uint8 nll, cc, lo, hi, ll, ss, tt, GPSCheckSumChar;
uint8 GPSTxCheckSum, RxCheckSum;
boolean EmptyField;
int16 ValidGPSSentences;

#define DEFAULT_BAUD_RATES 7
const uint32 DefaultBaud[] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400 };

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
#define UBX_UPD_CLASS		0x09
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
#define UBX_CFG_RST			0x04
#define UBX_CFG_DAT			0x09
#define UBX_CFG_TP			0x07
#define UBX_CFG_RATE		0X08
#define UBX_CFG_CFG			0X09
#define UBX_CFG_EKF			0x12
#define UBX_CFG_SBAS		0x16
#define UBX_CFG_NAV5		0x24
#define UBX_CFG_GNSS		0x3e

#define UBX_UPD_SOS			0x14

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
	uint8 swVersion[30];
	uint8 hwVersion[10];
	uint8 extension[30][7];
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
	uint8 sec; // Seconds of minute, range 0..60 (UTC)
	uint8 valid; // Validity flags (see UBX_RX_NAV_PVT_VALID_...)
	uint32 tAcc; // Time accuracy estimate (UTC) [ns]
	int32 nano; // Fraction of second (UTC) [-1e9...1e9 ns]
	uint8 fixtype; // GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix
	uint8 flags1; // Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...)
	uint8 flags2;
	uint8 numSV; // Number of SVs used in Nav Solution
	int32 lon; // Longitude [1e-7 deg]
	int32 lat; // Latitude [1e-7 deg]
	int32 height; // Height above ellipsoid [mm]
	int32 hMSL; // Height above mean sea level [mm]
	uint32 hAcc; // Horizontal accuracy estimate [mm]
	uint32 vAcc; // Vertical accuracy estimate [mm]
	int32 velN; // NED north velocity [mm/s]
	int32 velE; // NED east velocity [mm/s]
	int32 velD; // NED down velocity [mm/s]
	int32 gSpeed; // Ground Speed (2-D) [mm/s]
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
		UbxStructSBAS sbas;
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

void TxUbxuint8s(uint8 s, const void* v,  const uint16 len) {
	idx i;

	for(i = 0; i < len; i++)
		TxUbxu8(s, ((uint8*)v)[i]);
} // TxUbxuint8s

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
		senConf = _b8, // M8
		rinvConf = _b9, // M8
		antConf = _b10,
		logConf = _b11, // M8
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
	TxUbxu8(s, devFlash | devBBR);

	TxUbxCheckSum(s);

	Delay1mS(1000);

} // UbxSaveConfig

// INAV **********************************************************************

typedef struct {
	uint8 gnssId;
	uint8 resTrkCh;
	uint8 maxTrkCh;
	uint8 reserved1;
// flags
	uint8 enabled;
	uint8 undefined0;
	uint8 sigCfgMask;
	uint8 undefined1;
} __attribute__((packed)) ubx_gnss_element_t;


typedef struct {
	uint8_t msgVer;
	uint8_t xx;
	uint8_t yy;
	uint8_t numTrkChHw;
	uint8_t numTrkChUse;
	uint8_t numConfigBlocks;
	ubx_gnss_element_t config[0];
} __attribute__((packed)) ubx_gnss_msg_t;

static const uint8 default_payload[] = { 0xFF, 0xFF, 0x03, 0x03, 0x00, // CFG-NAV5 - Set engine settings (original MWII code)
		0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, // Collected by resetting a GPS unit to defaults. Changing mode to Pedestrian and
		0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, // capturing the data from the U-Center binary console.
		0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

enum gnssids {
	GNSSID_GPS,
	GNSSID_SBAS,
	GNSSID_GALILEO,
	GNSSID_BEIDOU,
	GNSSID_IMES,
	GNSSID_QZSS,
	GNSSID_GLONASS
};

const ubx_gnss_element_t GNSS[] = {
		{ GNSSID_GPS, 8, 16, 0, 		1, 0, 1, 1 }, // true
		{ GNSSID_SBAS, 1, 3, 0, 		0, 0, 0, 1 }, // false for USA
		{ GNSSID_GALILEO, 4, 8, 0,		1, 0, 1, 1 }, // true
		{ GNSSID_BEIDOU, 8, 16, 0, 		0, 0, 0, 1 },
		{ GNSSID_IMES, 0, 8, 0, 		0, 0, 0, 3 },
		{ GNSSID_QZSS, 0, 3, 0, 		0, 0, 0, 5 },
		{ GNSSID_GLONASS, 8, 14, 0, 	1, 0, 1, 1 } // true
 };


void UbxSetGNSS(uint8 s) {
	idx g;

	UbxSendPreamble(s);

	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_GNSS);
	TxUbxu8(s, 0x3c); // msg ver
	TxUbxu8(s, 0); // xxx
	TxUbxu8(s, 0); // yyy
	TxUbxu8(s, 0);// read only, so unset
	TxUbxu8(s, 32);
	TxUbxu8(s, GNSSID_GLONASS + 1);

	for (g = GNSSID_GPS; g <= GNSSID_GLONASS; g++)
		TxUbxuint8s(s, &GNSS[g], sizeof(ubx_gnss_element_t));

	TxUbxCheckSum(s);

	Delay1mS(50);

} // UbxSetGNSS



void UbxSetMode_INAV(uint8 s, uint8 dynModel, uint8 fixMode) {
	idx i;

	UbxSendPreamble(s);

	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_NAV5);
	TxUbxu8(s, 36);
	for (i = 0; i < sizeof(default_payload); i++)
		TxUbxu8(s, default_payload[i]);
	TxUbxu8(s, dynModel);
	TxUbxu8(s, fixMode);

	TxUbxCheckSum(s);

	Delay1mS(50);
} // UbxSetMode_INAV

// INAV END **********************************************************

void UbxReset(uint8 s, uint16 resetType) {

	UbxSendPreamble(s);

	TxUbxu8(s, UBX_CFG_CLASS);
	TxUbxu8(s, UBX_CFG_RST);
	TxUbxu8(s, 4);
	TxUbxu16(s, 0);
	TxUbxu16(s, resetType);

	TxUbxCheckSum(s);

	Delay1mS(50);

	if (resetType == stopGNSS) {
		UbxSendPreamble(s);
		TxUbxu8(s, UBX_UPD_CLASS);
		TxUbxu8(s, UBX_UPD_SOS);
		TxUbxu8(s, 4);
		TxUbxu16(s, 2);
		TxUbxu16(s, 0);

		TxUbxCheckSum(s);

		Delay1mS(50);
	}

} // UbxStopAndSave

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
#if defined(USE_KEN_GPS_M8)
			TxUbxu8(s, Portable);
			TxUbxu8(s, ThreeD);
#else
	if (F.IsFixedWing)
		TxUbxu8(s, Airborne1G);
	else
		TxUbxu8(s, Pedestrian);
	TxUbxu8(s, Automatic);
#endif
	TxUbxu32(s, 0); // altitude
	TxUbxu32(s, 10000); // 0.0001 altitude variance for 2D

	TxUbxu8(s, 5); // min elev deg (i8)
	TxUbxu8(s, 0); // dead reckoning limit s
	TxUbxu16(s, 250); // pdop x 0.1
	TxUbxu16(s, 250); // tdop x 0.1
	TxUbxu16(s, 100); // pacc m
	TxUbxu16(s, 300); // tacc m
	TxUbxu8(s, 0); // hold threshold cm/S
	TxUbxu8(s, 60); // dgnss timeout was 0
	TxUbxu32(s, 0); // reserved
	TxUbxu32(s, 0); // reserved
	TxUbxu32(s, 0); // reserved

	TxUbxCheckSum(s);

	Delay1mS(50);
} // UbxSetMode

void UbxSetSBAS(uint8 s) {

	enum Usage {
		range = _b0, diffcor = _b1, integrity = _b2
	};

		UbxSendPreamble(s);

		TxUbxu8(s, UBX_CFG_CLASS);
		TxUbxu8(s, UBX_CFG_SBAS);
		TxUbxu16(s, 8);
		TxUbxu8(s, UseSBAS); // disable in USA?
		TxUbxu8(s, diffcor | range); // was diffcor and integrity
		TxUbxu8(s, 3); // maxSBAS 3 was 0
		TxUbxu8(s, 0); // scan mode 2
		TxUbxu32(s, 0); // scan mode 1

		TxUbxCheckSum(s);

		Delay1mS(50);

} // UbxSetSBAS

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
			GPS.missionTime = GPS.lastPosUpdatemS = mSClock(); //ubx.payload.pvt.iTOW;
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
			//ubx.payload.valned.iTOW;
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
			GPS.iTOW = ubx.payload.sbas.iTOW;
			//uint8 geo;
			//uint8 mode;
			//int8 sys;
			//uint8 service;
			//uint8 cnt;
			//uint8 res[3];
			//SVStruct SV[16];

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
			memcpy(&GPS.swVersion, ubx.payload.ver.swVersion, 30);
			GPS.hwVersion = atoi(ubx.payload.ver.hwVersion);
			//ubx.payload.ver.extension[30][7];
			//UbxEnableMessage(GPSSerial, UBX_MON_CLASS, UBX_MON_VER, 0);
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

	F.GPSValid = ((GPS.fix == Fix3D) || (GPS.fix == Fix2D)
			|| (GPS.fix == FixGPSDeadReckoning));

#ifdef USE_STRICT_GPS_HACC
	F.GPSValid &= (GPS.hAcc <= GPSMinhAcc);
#endif
#ifdef USE_STRICT_GPS_VACC
	F.GPSValid &= (GPS.vAcc <= GPSMinvAcc);
#endif

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
	UbxSetSBAS(s);
	UbxPollVersion(s);

	if (UbxVersion == 8) {

		UbxSetInterval(s, 100);

		UbxSetGNSS(s);
		UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_PVT, 1);

	} else {
		UbxSetInterval(s, 200);
		UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_SOL, 1); // for fix and # of sats
		UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_VELNED, 1);
		UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_POSLLH, 1);
		// UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_STATUS, 1);
		// Ken reports problems UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_TIMEUTC, 255);
	}

//	UbxSetSBAS(s, UbxVersion != 7); // v1 broken

//	UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_DOP, 1);
//	UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_SVINFO, 5);
//	UbxEnableMessage(s, UBX_NAV_CLASS, UBX_NAV_SAT, 5);

	UbxSaveConfig(s);

	F.GPSPacketReceived = false;

	GPS.lag = 0.5f;

	LEDOff(ledYellowSel);

} // InitUbxGPS

real32 GPSToM(int32 c) {
//return ((real64) c * 0.011131948079f);
	return ((real64) c * ((real64) EARTH_RADIUS_M * PI) / (180.0 * 1e7));
} // GPSToM

int32 MToGPS(real32 c) {
	return ((real64) c / (((real64) EARTH_RADIUS_M * PI) / (180.0 * 1e7)));
} // MToGPS
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


boolean GPSSanityCheck(void) {
	boolean r;

	r = (Abs(GPS.C[NorthC].Raw) <= 900000000L)
			&& (Abs(GPS.C[EastC].Raw) <= 1800000000L)
			&& (GPS.C[NorthC].Raw != 0) && (GPS.C[EastC].Raw != 0);

	return (r);

} // GPSSanityCheck

void ProcessGPSSentence(void) {
	idx a;
	static timemS GPSPosUpdatemSP = 0;

	LEDOn(ledBlueSel);

	if (F.GPSValid) {

		mSTimer(GPSTimeoutmS, NavGPSTimeoutmS);

		if (GPSSanityCheck() && F.OriginValid
				|| (F.IsFixedWing && F.OffsetOriginValid)) {

			for (a = NorthC; a <= DownC; a++) {
				GPS.C[a].Pos = GPSToM(GPS.C[a].Raw - GPS.C[a].OriginRaw);
				GPS.C[a].Vel;
			}
			GPS.C[EastC].Pos *= GPS.longitudeCorrection;

			if (GPSPosUpdatemSP == 0)
				GPSPosUpdatemSP = GPS.lastPosUpdatemS;

			GPSdTmS = GPS.lastPosUpdatemS - GPSPosUpdatemSP;
			GPSPosUpdatemSP = GPS.lastPosUpdatemS;

			//if ((State == Ready) || (State == Landed))
			//	LEDOn(ledBlueSel);
			//else
			//	LEDToggle(ledYellowSel);

			F.NewGPSPosition = true;
		}

	}

} // ProcessGPSSentence

void CheckGPSTimeouts(void) {

	if (mSTimeout(GPSTimeoutmS)) {
		mSTimer(GPSTimeoutmS, NavGPSTimeoutmS);

		ZeroNavCorrections();
		F.GPSValid = F.ValidGPSVel = F.NewNavUpdate = F.NavigationEnabled =
		false;

		LEDOff(ledBlueSel);
	}

} // CheckGPSTimeouts

void GPSISR(char ch) {

	if (EnableGPSPassThru) {

		LEDToggle(ledBlueSel);
		TxChar(TelemetrySerial, ch); // just echo raw GPS to telemetry

	} else if (F.Emulation) {

	} else

	if (F.HaveGPS) {
		switch (CurrGPSType) { // 1.5uS/char
		case UbxM8GPSInit:
		case UbxLegacyGPSInit:
		case UbxGPS:
			RxGPSUbxPacket(ch);
			break;
		case NMEAGPS:
		case NoGPS:
		default:
			break;
		} // switch

		if (F.GPSPacketReceived) {
			F.GPSPacketReceived = false;
			ProcessGPSSentence();
		}

	}

} // GPSISR

void InitGPS(void) {

	cc = 0;
	F.OriginValid = F.OffsetOriginValid = F.GPSValid = F.GPSPacketReceived =
	false;

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
			UbxPollVersion(GPSSerial);
			break;
		case NMEAGPS:
		case NoGPS:
		default:
			break;
		} // switch
	}

} // InitGPS

