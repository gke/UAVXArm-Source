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

#include "UAVX.h"

// FrSky Telemetry implementation by slipstream @ rcgroups rewritten by gke

#define FSHUB_SENTINEL       0x5e

enum {
	// Data IDs  (BP = before decimal point; AP = after decimal point)

	FSHUB_ID_GPS_ALT_BP = 0x01,
	FSHUB_ID_TEMP1 = 0x02, // barometer deg C
	FSHUB_ID_RPM = 0x03,
	FSHUB_ID_FUEL = 0x04,
	FSHUB_ID_TEMP2 = 0x05,
	FSHUB_ID_VOLTS = 0x06, // cells

	// 0x07
	// 0x08

	FSHUB_ID_GPS_ALT_AP = 0x09,
	FSHUB_ID_BARO_ALT_BP = 0x10,

	// 0x0A
	// 0x0B
	// 0x0C
	// 0x0D
	// 0x0E
	// 0x0F // seems to be emitted when there is a buffer overrun in the Rx.

	FSHUB_ID_GPS_SPEED_BP = 0x11,
	FSHUB_ID_GPS_LONG_BP = 0x12,
	FSHUB_ID_GPS_LAT_BP = 0x13,
	FSHUB_ID_GPS_COURS_BP = 0x14,

	FSHUB_ID_GPS_DAY_MONTH = 0x15,
	FSHUB_ID_GPS_YEAR = 0x16,
	FSHUB_ID_GPS_HOUR_MIN = 0x17,
	FSHUB_ID_GPS_SEC = 0x18,

	FSHUB_ID_GPS_SPEED_AP = 0x19, // +8 from BP
	FSHUB_ID_GPS_LONG_AP = 0x1A,
	FSHUB_ID_GPS_LAT_AP = 0x1B,
	FSHUB_ID_GPS_COURS_AP = 0x1C,

	UAVX_ID_GPS_STAT = 0x1d,

	// 0x1e

	// 0x1f
	// 0x20

	FSHUB_ID_BARO_ALT_AP = 0x21,

	FSHUB_ID_GPS_LONG_EW = 0x22,
	FSHUB_ID_GPS_LAT_NS = 0x23,

	FSHUB_ID_ACCEL_X = 0x24, // m/s2
	FSHUB_ID_ACCEL_Y = 0x25,
	FSHUB_ID_ACCEL_Z = 0x26,

	FSHUB_ID_CURRENT = 0x28,

	UAVX_ID_WHERE_DIST = 0x29, // metres the aircraft is way
	UAVX_ID_WHERE_BEAR = 0x2a, // bearing (deg) to aircraft
	UAVX_ID_WHERE_ELEV = 0x2b, // elevation (deg) of the aircraft above the horizon
	UAVX_ID_WHERE_HINT = 0x2c, // which to turn to come home intended for voice guidance

	UAVX_ID_COMPASS = 0x2d, // deg
	// 0x2e
	// 0x2f

	FSHUB_ID_VARIO = 0x30, // cm/sec

	//--------------------

	// UAVX user defined

	UAVX_ID_GYRO_X = 0x31, // deg/sec
	UAVX_ID_GYRO_Y = 0x32,
	UAVX_ID_GYRO_Z = 0x33,

	UAVX_ID_PITCH = 0x34, // deg
	UAVX_ID_ROLL = 0x35,

	UAVX_ID_MAH = 0x36, // mAH battery consumption

	//--------------------

	FSHUB_ID_VFAS = 0x39,
	FSHUB_ID_VOLTS_BP = 0x3A,
	FSHUB_ID_VOLTS_AP = 0x3B,
	FSHUB_ID_FRSKY_LAST = 0x3F,

};

timeuS FrSkyDLinkuS;

uint16 MakeFrac(real32 v, uint16 s) {
	return (Abs((int32)(v * s)) % s);
} // MakeFrac

void TxFrSkyHubHeader(uint8 s) {
	TxChar(s, FSHUB_SENTINEL);
} // TxFrSkyHeader

void TxFrSky(uint8 s, uint8 data) {
	// byte stuffing
	if (data == 0x5e) {
		TxChar(s, 0x5d);
		TxChar(s, 0x3e);
	} else if (data == 0x5d) {
		TxChar(s, 0x5d);
		TxChar(s, 0x3d);
	} else
		TxChar(s, data);
} // TxFrSky


void TxFrSky16(uint8 s, int16 a) {
	TxFrSky(s, a);
	TxFrSky(s, (a >> 8) & 0xff);
} // TxFrSky16


void TxFrSkyHubPacket(uint8 s, uint8 appID, int16 v) {
	TxFrSkyHubHeader(s);
	TxFrSky(s, appID);
	TxFrSky16(s, v);
} // TxFrSkyHubPacket

void TxFrSkyHubPacketPair(uint8 s, uint8 id1, uint8 id2, int16 v, int8 frac) {
	TxFrSkyHubPacket(s, id1, v);
	TxFrSkyHubPacket(s, id2, MakeFrac(v, frac));
}

void TxFrSkyHubAcc(uint8 s) {
	uint8 a;

	for (a = X; a <= Z; a++)
		TxFrSkyHubPacket(s, FSHUB_ID_ACCEL_X + a, Acc[a] * GRAVITY_MPS_S_R
				* 1000.0f);
} // TxFrSkyHubAcc

void TxFrSkyHubGyro(uint8 s) {
	uint8 a;

	for (a = Pitch; a <= Yaw; a++)
		TxFrSkyHubPacket(s, UAVX_ID_GYRO_X + a, RadiansToDegrees(Rate[a]));
} // TxFrSkyHubAcc

void TxFrSkyHubAttitude(uint8 s) {
	uint8 a;

	for (a = Pitch; a <= Roll; a++)
		TxFrSkyHubPacket(s, UAVX_ID_PITCH + a, RadiansToDegrees(Angle[a]));

} // TxFrSkyHubAttitude

void TxFrSkyHubBaro(uint8 s) {

	TxFrSkyHubPacketPair(s, FSHUB_ID_BARO_ALT_BP, FSHUB_ID_BARO_ALT_AP,
			BaroAltitude - OriginAltitude, 10);
} //  TxFrSkyHubBaro

void TxFrSkyHubVario(uint8 s) {
	TxFrSkyHubPacket(s, FSHUB_ID_VARIO, ROC * 100.0f);
} //  TxFrSkyHubBaro

void TxFrSkyHubTemperature1(uint8 s) {
	TxFrSkyHubPacket(s, FSHUB_ID_TEMP1, BaroTemperature);
} // TxFrSkyHubTemperature1

void TxFrSkyHubFuel(uint8 s) {
	TxFrSkyHubPacket(
			s,
			FSHUB_ID_FUEL,
			Limit(100 * ( 1.0f - BatteryChargeUsedmAH / BatteryCapacitymAH), 0, 100));
} // TxFrSkyHubFuel

void TxFrSkyHubmAH(uint8 s) {
	TxFrSkyHubPacket(s, UAVX_ID_MAH, BatteryChargeUsedmAH);
} // TxFrSkyHubmAH

void TxFrSkyHubTemperature2(uint8 s) {
	TxFrSkyHubPacket(s, FSHUB_ID_TEMP2, MPU6XXXTemperature);
} // TxFrSkyHubTemperature2

void TxFrSkyHubTime(uint8 s) {
	timemS seconds = mSClock() / 1000;
	uint8 minutes = (seconds / 60) % 60;

	// if we fly for more than an hour, something's wrong anyway
	TxFrSkyHubPacket(s, FSHUB_ID_GPS_HOUR_MIN, minutes << 8);
	TxFrSkyHubPacket(s, FSHUB_ID_GPS_SEC, seconds % 60);
} // TxFrSkyHubTime

void TxFrSkyHubWhere(uint8 s) {
	if ((Nav.Distance >= 0.0) && (Nav.Distance < 32000.0f)) {
		TxFrSkyHubPacket(s, UAVX_ID_WHERE_DIST, Nav.Distance);
		TxFrSkyHubPacket(s, UAVX_ID_WHERE_BEAR, RadiansToDegrees(Nav.Bearing));
		TxFrSkyHubPacket(s, UAVX_ID_WHERE_ELEV, RadiansToDegrees(Nav.Elevation));
		TxFrSkyHubPacket(s, UAVX_ID_WHERE_HINT, RadiansToDegrees(Nav.Hint));
	}
}

// FrSky uses NMEA form rather than computationally sensible decimal degrees

typedef struct {
	uint16 bp, ap;
} pair_rec;

static void GPStoDDDMM_MMMM(int32 L, pair_rec * c) {
	uint32 d, mf, dm, m;

	L = Abs(L);
	d = L / 10000000L;
	dm = (L % 10000000L) * 60;
	m = dm / 10000000L;
	mf = dm - m * 10000000L;

	c->bp = d * 100 + m;
	c->ap = mf / 1000L; // limited precision
}

void TxFrSkyHubGPSStat(uint8 s) {
	TxFrSkyHubPacket(s, UAVX_ID_GPS_STAT, GPS.noofsats * 1000 + GPS.fix * 100
			+ (F.GPSValid & 1) * 10 + (F.OriginValid & 1));
} // TxFrSkyHubGPSStat

void TxFrSkyHubGPSCoords(uint8 s) {
	pair_rec c;

	GPStoDDDMM_MMMM(GPS.C[NorthC].Raw, &c);
	TxFrSkyHubPacket(s, FSHUB_ID_GPS_LAT_BP, c.bp);
	TxFrSkyHubPacket(s, FSHUB_ID_GPS_LAT_AP, c.ap);
	TxFrSkyHubPacket(s, FSHUB_ID_GPS_LAT_NS, GPS.C[NorthC].Raw < 0 ? 'S' : 'N');

	GPStoDDDMM_MMMM(GPS.C[EastC].Raw, &c);
	TxFrSkyHubPacket(s, FSHUB_ID_GPS_LONG_BP, c.bp);
	TxFrSkyHubPacket(s, FSHUB_ID_GPS_LONG_AP, c.ap);
	TxFrSkyHubPacket(s, FSHUB_ID_GPS_LONG_EW, GPS.C[EastC].Raw < 0 ? 'W' : 'E');

} // TxFrSkyHubGPS

void TxFrSkyHubGPSSpeed(uint8 s) {
	TxFrSkyHubPacketPair(s, FSHUB_ID_GPS_SPEED_BP, FSHUB_ID_GPS_SPEED_AP,
			GPS.gspeed * 3.6f, 10);
} // TxFrSkyHubGPSSpeed

void TxFrSkyHubGPSAlt(uint8 s) {
	TxFrSkyHubPacketPair(s, FSHUB_ID_GPS_ALT_BP, FSHUB_ID_GPS_ALT_AP,
			GPS.altitude, 10);
} // TxFrSkyHubGPSAlt

void TxFrSkyHubCellVoltages(uint8 s) {

	static uint16 currentCell = 0;
	uint32 cellVoltage;
	uint16 payload;

	// A cell packet is formated this way: https://github.com/jcheger/frsky-arduino/blob/master/FrskySP/FrskySP.cpp
	// content    | length
	// ---------- | ------
	// volt[id]   | 12-bits
	// celltotal  | 4 bits
	// cellid     | 4 bits

	cellVoltage = (BatteryVolts * 500.0f) / BatteryCellCount;

	payload = ((cellVoltage & 0x0ff) << 8) | (currentCell << 4) | ((cellVoltage
			& 0xf00) >> 8);

	TxFrSkyHubPacket(s, FSHUB_ID_VOLTS, payload);

	if (++currentCell >= BatteryCellCount)
		currentCell = 0;

} // TxFrSkyHubCellVoltages

void TxFrSkyHubVoltage(uint8 s) {
	TxFrSkyHubPacketPair(s, FSHUB_ID_VOLTS_BP, FSHUB_ID_VOLTS_AP, BatteryVolts
			* 0.5f, 100);
} // TxFrSkyHubVoltage

void TxFrSkyHubCurrent(uint8 s) {
	TxFrSkyHubPacket(s, FSHUB_ID_CURRENT, BatteryCurrent * 10);
} // TxFrSkyHubCurrent

void TxFrSkyHubGPSHeading(uint8 s) {

	TxFrSkyHubPacketPair(s, FSHUB_ID_GPS_COURS_BP, FSHUB_ID_GPS_COURS_AP,
			RadiansToDegrees(GPS.heading), 10);
	//RadiansToDegrees(Heading), 10);

} // TxFrSkyHubGPSHeading

void TxFrSkyHubCompassHeading(uint8 s) {

	TxFrSkyHubPacket(s, UAVX_ID_COMPASS, RadiansToDegrees(Heading));
} // TxFrSkyHubCompassHeading


void SendFrSkyHubTelemetry(uint8 s) {
	static uint8 FrameCount = 0;

	if (++FrameCount == 40) { // FRAME 3 every 8 seconds
		TxFrSkyHubTime(s); // 2
		TxFrSkyHubTemperature1(s); // 1
		//TxFrSkyHubTemperature2(s); // 1
		//TxFrSkyHubFuel(s);
		TxChar(s, FSHUB_SENTINEL);

		FrameCount = 0;

	} else if ((FrameCount % 5) == 0) { // FRAME 2 every second
		if (F.GPSValid) {
			if (F.OriginValid)
				TxFrSkyHubWhere(s); // 4
			TxFrSkyHubGPSSpeed(s); // 2
			TxFrSkyHubGPSAlt(s); // 2
			TxFrSkyHubGPSHeading(s); // 2
			TxFrSkyHubGPSCoords(s); // 6
			TxChar(s, FSHUB_SENTINEL);
		}
		TxFrSkyHubGPSStat(s); // 1
		//TxFrSkyHubCompassHeading(s); // 2 2-> 17
	} else { // FRAME 1 every 200mS
		TxFrSkyHubBaro(s); // 2
		TxFrSkyHubVario(s); // 1
		TxFrSkyHubVoltage(s); // 2
		TxFrSkyHubCellVoltages(s); // 1
		TxFrSkyHubCurrent(s); // 1
		TxFrSkyHubmAH(s); // 1
		//TxFrSkyHubAcc(s); // 3 could add for coordinated turns?
		TxFrSkyHubGyro(s); // 3
		TxFrSkyHubAttitude(s); // 2 ~ 14
		TxChar(s, FSHUB_SENTINEL);
	}

} // TxFrSkyHubTelemetry


//________________________________________________________________________

// SPort rewritten from CleanFlight

enum {
	FSSP_SENTINEL = 0x7E, FSSP_7D = 0x7D, FSSP_DATA_FRAME = 0x10,

	// ID of sensor. Must be something that is polled by FrSky RX
	FSSP_SENSOR_ID1 = 0x1B,
	FSSP_SENSOR_ID2 = 0x0D,
	FSSP_SENSOR_ID3 = 0x34,
	FSSP_SENSOR_ID4 = 0x67,
// there are 32 ID's polled by smartport master
// remaining 3 bits are crc (according to comments in openTx code)
};

// these data identifiers are obtained from:
// https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky.h
enum {
	FSSP_ID_SPEED = 0x0830,
	FSSP_ID_VFAS = 0x0210,
	FSSP_ID_CURRENT = 0x0200,
	FSSP_ID_RPM = 0x050F,
	FSSP_ID_ALTITUDE = 0x0100,
	FSSP_ID_FUEL = 0x0600,
	FSSP_ID_ADC1 = 0xF102,
	FSSP_ID_ADC2 = 0xF103,
	FSSP_ID_LATLONG = 0x0800,
	FSSP_ID_CAP_USED = 0x0600,
	FSSP_ID_VARIO = 0x0110,
	FSSP_ID_CELLS = 0x0300,
	FSSP_ID_CELLS_LAST = 0x030F,
	FSSP_ID_HEADING = 0x0840,
	FSSP_ID_ACCX = 0x0700,
	FSSP_ID_ACCY = 0x0710,
	FSSP_ID_ACCZ = 0x0720,
	FSSP_ID_T1 = 0x0400,
	FSSP_ID_T2 = 0x0410,
	FSSP_ID_GPS_ALT = 0x0820,
	FSSP_ID_A3 = 0x0900,
	FSSP_ID_A4 = 0x0910,

	FSSP_ID_RSSI = 0xf101,
	//	FSSP_ID_ADC1 = 0xf102,
	//	FSSP_ID_ADC2 = 0xf103,
	FSSP_ID_BATT = 0xf104,
	FSSP_ID_SWR = 0xf105,
	FSSP_ID_T1_FIRST = 0x0400,
	FSSP_ID_T1_LAST = 0x040f,
	FSSP_ID_T2_FIRST = 0x0410,
	FSSP_ID_T2_LAST = 0x041f,
	FSSP_ID_RPM_FIRST = 0x0500,
	FSSP_ID_RPM_LAST = 0x050f,
	FSSP_ID_FUEL_FIRST = 0x0600,
	FSSP_ID_FUEL_LAST = 0x060f,
	FSSP_ID_ALT_FIRST = 0x0100,
	FSSP_ID_ALT_LAST = 0x010f,
	FSSP_ID_VARIO_FIRST = 0x0110,
	FSSP_ID_VARIO_LAST = 0x011f,
	FSSP_ID_ACCX_FIRST = 0x0700,
	FSSP_ID_ACCX_LAST = 0x070f,
	FSSP_ID_ACCY_FIRST = 0x0710,
	FSSP_ID_ACCY_LAST = 0x071f,
	FSSP_ID_ACCZ_FIRST = 0x0720,
	FSSP_ID_ACCZ_LAST = 0x072f,
	FSSP_ID_CURR_FIRST = 0x0200,
	FSSP_ID_CURR_LAST = 0x020f,
	FSSP_ID_VFAS_FIRST = 0x0210,
	FSSP_ID_VFAS_LAST = 0x021f,
	FSSP_ID_CELLS_FIRST = 0x0300,
	//	FSSP_ID_CELLS_LAST = 0x030f,
	FSSP_ID_GPS_LONG_LATI_FIRST = 0x0800,
	FSSP_ID_GPS_LONG_LATI_LAST = 0x080f,
	FSSP_ID_GPS_ALT_FIRST = 0x0820,
	FSSP_ID_GPS_ALT_LAST = 0x082f,
	FSSP_ID_GPS_SPEED_FIRST = 0x0830,
	FSSP_ID_GPS_SPEED_LAST = 0x083f,
	FSSP_ID_GPS_COURS_FIRST = 0x0840,
	FSSP_ID_GPS_COURS_LAST = 0x084f,
	FSSP_ID_GPS_TIME_DATE_FIRST = 0x0850,
	FSSP_ID_GPS_TIME_DATE_LAST = 0x085f
};

// FrSky wrong IDs ?
//FSHUB_BETA_VARIO_ID 0x8030
//FSHUB_BETA_BARO_ALT_ID 0x8010


const uint16 SPortID[] = { FSSP_ID_SPEED, FSSP_ID_VFAS, FSSP_ID_CURRENT,
		//FSSP_ID_RPM       ,
		FSSP_ID_ALTITUDE,
		FSSP_ID_FUEL,
		//FSSP_ID_ADC1      ,
		//FSSP_ID_ADC2      ,
		FSSP_ID_LATLONG,
		FSSP_ID_LATLONG, // twice
		//FSSP_ID_CAP_USED  ,
		FSSP_ID_VARIO, FSSP_ID_CELLS,
		//FSSP_ID_CELLS_LAST,
		FSSP_ID_HEADING, FSSP_ID_ACCX, FSSP_ID_ACCY, FSSP_ID_ACCZ, FSSP_ID_T1,
		FSSP_ID_T2, FSSP_ID_GPS_ALT,
		//FSSP_ID_A3	  ,
		FSSP_ID_A4, 0 };

uint16 FrSkyTxCheckSum, FrSkyRxCheckSum;

timeuS FrSkySPortLastTxuS = 0;
uint8 FrSkyRxPacketByteCount;
uint8 FrSkyRxPacketLength;
uint8 FrSkyReceivedPacketTag;

uint32 FrSkyRxErrors = 0;
boolean FrSkyPacketReceived = false;
uint8 FrSkyPacket[32];

boolean FrSkySPortHasRequest = false;

void RxFrSkySPort(uint8 ch) {
	enum {
		WaitRxSentinel, WaitRxLength, WaitRxTag, WaitRxBody, WaitRxESC
	};
	static uint8 FrSkyPacketRxState = 0;

	switch (FrSkyPacketRxState) {
	case WaitRxSentinel:
		if (ch == FSSP_SENTINEL)
			FrSkyPacketRxState = WaitRxLength;
		break;
	case WaitRxLength:
		FrSkyRxPacketLength = ch;
		FrSkyPacketRxState = WaitRxTag;
		break;
	case WaitRxTag:
		if (ch != FSSP_SENTINEL) {
			FrSkyReceivedPacketTag = ch;
			FrSkyRxPacketByteCount = 0;
			FrSkyPacketRxState = WaitRxBody;
		}
		break;
	case WaitRxBody:
		if (((ch == FSSP_7D) || (ch == FSSP_SENTINEL))
				&& (FrSkyRxPacketByteCount != FrSkyRxPacketLength)) // was 5
			FrSkyPacketRxState = WaitRxESC;
		else {
			FrSkyPacket[FrSkyRxPacketByteCount++] = ch;
			if (FrSkyRxPacketByteCount == FrSkyRxPacketLength) // was 6
			{
				if (ch != FSSP_SENTINEL)
					FrSkyRxErrors++;
				else
					FrSkyPacketReceived = true;
				FrSkyPacketRxState = WaitRxSentinel;
			}
		}
		break;
	case WaitRxESC:
		ch ^= 0x60;
		FrSkyPacket[FrSkyRxPacketByteCount++] = ch;
		FrSkyPacketRxState = WaitRxBody;
		break;

	default:
		FrSkyPacketRxState = WaitRxSentinel;
		break;
	}

} // RxFrSkySport


void TxFrSkySPortByte(uint8 s, uint8 ch) {
	// smart port escape sequence
	if (ch == FSSP_7D || ch == FSSP_SENTINEL) {
		TxChar(s, FSSP_7D);
		ch ^= 0x20;
	}

	TxChar(s, ch);

	FrSkyTxCheckSum += ch;
	FrSkyTxCheckSum += FrSkyTxCheckSum >> 8;
	FrSkyTxCheckSum &= 0x00ff;

} // TxFrSkySPortByte


void TxFrSkySPortPacket(uint8 s, uint16 appID, uint32 val) {

	FrSkyTxCheckSum = 0;

	TxFrSkySPortByte(s, FSSP_DATA_FRAME);
	uint8 *u8p = (uint8*) &appID;
	TxFrSkySPortByte(s, u8p[0]);
	TxFrSkySPortByte(s, u8p[1]);
	u8p = (uint8*) &val;
	TxFrSkySPortByte(s, u8p[0]);
	TxFrSkySPortByte(s, u8p[1]);
	TxFrSkySPortByte(s, u8p[2]);
	TxFrSkySPortByte(s, u8p[3]);
	TxFrSkySPortByte(s, 0xff - FrSkyTxCheckSum);

} // TxFrSkySPort


void TxFrSkySPort(uint8 s) {
	static uint8 appIDindex = 0;

	static uint8 currentCell = 0;
	uint32 payload = 0;

	int32 tmpi;

	boolean Skip = false;

	do {
		uint16 appID = SPortID[appIDindex];

		switch (appID) {
		case FSSP_ID_SPEED:
			if (F.HaveGPS && F.GPSValid)
				TxFrSkySPortPacket(s, appID, GPS.gspeed * 10.0); // 0.1 m/s
			break;
		case FSSP_ID_VFAS:
			TxFrSkySPortPacket(s, appID, BatteryVolts * 10); // 0.1V
			break;
		case FSSP_ID_CELLS:
			/*
			 * A cell packet is formated this way:
			 * https://github.com/jcheger/frsky-arduino/blob/master/FrskySP/FrskySP.cpp
			 * content    | length
			 * ---------- | ------
			 * volt[id]   | 12-bits
			 * celltotal  | 4 bits
			 * cellid     | 4 bits
			 */

			// Cells Data Payload
			payload = 0;
			payload |= ((uint16) (BatteryVolts * 100 + BatteryCellCount)
					/ (BatteryCellCount * 2)) & 0x0FFF;
			payload <<= 4;
			payload |= (uint8) BatteryCellCount & 0x0F;
			payload <<= 4;
			payload |= (uint8) currentCell & 0x0F;

			// Send Payload
			TxFrSkySPortPacket(s, appID, payload);

			// Incremental Counter
			currentCell++;
			currentCell %= BatteryCellCount; // Reset counter @ max index
			break;
		case FSSP_ID_CURRENT:
			TxFrSkySPortPacket(s, appID, BatteryCurrent * 100); // 10mA
			break;
			//case FSSP_ID_RPM        :
		case FSSP_ID_ALTITUDE:
			if (F.BaroActive)
				TxFrSkySPortPacket(s, appID, BaroAltitude);
			break;
		case FSSP_ID_FUEL:
			TxFrSkySPortPacket(s, appID, Limit(100 * ( 1.0f - BatteryChargeUsedmAH / BatteryCapacitymAH), 0, 100)); //%?
			break;
			//case FSSP_ID_ADC1       :
			//case FSSP_ID_ADC2       :

		case FSSP_ID_LATLONG:
			if (F.HaveGPS && F.GPSValid) {
				uint32 tmpui = 0;

				if (appIDindex & 1) {
					tmpui = GPS.C[EastC].Raw & 0x3fffffff; // clip top bits
					tmpui = (uint32)((real64)tmpui * 0.06);
					if (GPS.C[EastC].Raw < 0)
						tmpui |= 0x40000000;
					tmpui |= 0x80000000; // longitude
				} else {
					tmpui = GPS.C[NorthC].Raw & 0x3fffffff;
					tmpui = (uint32)((real64)tmpui * 0.06);
					if (GPS.C[NorthC].Raw < 0)
						tmpui |= 0x40000000;
				}
				TxFrSkySPortPacket(s, appID, tmpui);
			}
			break;
			//case FSSP_ID_CAP_USED:
		case FSSP_ID_VARIO:
				TxFrSkySPortPacket(s, appID, ROC * 100); // unknown given unit but requested in 100 = 1m/s
			break;
		case FSSP_ID_HEADING:
			TxFrSkySPortPacket(s, appID, RadiansToDegrees(Angle[Yaw]) * 10); // *10deg
			break;
		case FSSP_ID_ACCX:
			TxFrSkySPortPacket(s, appID, Acc[X] * 100);
			// unknown input and unknown output unit
			// we can only show 00.00 format, another digit won't display right on Taranis
			break;
		case FSSP_ID_ACCY:
			TxFrSkySPortPacket(s, appID, Acc[Y] * 100);
			break;
		case FSSP_ID_ACCZ:
			TxFrSkySPortPacket(s, appID, Acc[Z] * 100);
			break;
		case FSSP_ID_T1:
			// we send all the flags as decimal digits for easy reading

			tmpi = 10000; // force MSD - Taranis 5 digits

			if ((State == Ready) || (State == Starting))
				tmpi += 1;
			if (State == Preflight)
				tmpi += 2;
			if (Armed())
				tmpi += 4;
			if (F.UsingAngleControl)
				tmpi += 10;
			if (!F.UsingAngleControl)
				tmpi += 20;
			if (F.PassThru)
				tmpi += 40;
			if (F.MagnetometerActive)
				tmpi += 100;
			if (F.BaroActive)
				tmpi += 200;
			if (F.RangefinderActive)
				tmpi += 400;
			if (NavState == HoldingStation)
				tmpi += 1000;
			if (NavState == Transiting)
				tmpi += 2000;
			//if (FLIGHT_MODE(HEADFREE_MODE))
			//tmpi += 4000;

			TxFrSkySPortPacket(s, appID, (uint32) tmpi);
			break;
		case FSSP_ID_T2:
			if (F.HaveGPS) // provide GPS lock status
				TxFrSkySPortPacket(s, appID, (F.GPSValid ? 1000 : 0)
						+ (F.OriginValid ? 2000 : 0) + GPS.noofsats);
			break;
		case FSSP_ID_GPS_ALT:
			if (F.HaveGPS && F.GPSValid)
				TxFrSkySPortPacket(s, appID, GPS.altitude * 10); // given in 0.1m , requested in 10 = 1m (should be in mm, probably a bug in opentx, tested on 2.0.1.7)
			break;
		case FSSP_ID_A4:
				TxFrSkySPortPacket(s, appID, BatteryVolts * 10
						/ BatteryCellCount); //sending calculated average cell value with 0.01 precision
			break;
		default:
			Skip = true; // skip over default/empty or unused cases
			break;
			// if nothing is sent, FrSkySPortHasRequest isn't cleared, we already incremented the counter, just loop back to the start
		} // switch

		if (SPortID[++appIDindex] == 0)
			appIDindex = 0;

	} while (Skip);

} // TxFrSkySPort


