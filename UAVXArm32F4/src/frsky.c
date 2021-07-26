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

#define FRSKY_SENTINEL       0x5e

//#define USE_GKE_FRSKY

#if defined(USE_GKE_FRSKY)

enum {
	// Data IDs  (BP = before decimal point; AP = after decimal point)

	ID_GPS_ALT_BP = 0x01,
	ID_TEMP1 = 0x02, // barometer deg C
	ID_RPM = 0x03,
	ID_FUEL = 0x04,
	ID_TEMP2 = 0x05,
	ID_VOLTS = 0x06, // cells

	// 0x07
	// 0x08

	ID_GPS_ALT_AP = 0x09,
	ID_ALT_BP = 0x10,

	// 0x0A
	// 0x0B
	// 0x0C
	// 0x0D
	// 0x0E
	// 0x0F // seems to be emitted when there is a buffer overrun in the Rx.

	ID_GPS_SPEED_BP = 0x11,
	ID_GPS_LONG_BP = 0x12,
	ID_GPS_LAT_BP = 0x13,
	ID_COURSE_BP = 0x14,

	ID_DAY_MONTH = 0x15,
	ID_YEAR = 0x16,
	ID_HOUR_MINUTE = 0x17,
	ID_SECOND = 0x18,

	ID_GPS_SPEED_AP = 0x19, // +8 from BP
	ID_GPS_LONG_AP = 0x1A,
	ID_GPS_LAT_AP = 0x1B,
	ID_COURSE_AP = 0x1C,

	ID_GPS_STAT = 0x1d,

	// 0x1e

	// 0x1f
	// 0x20

	ID_ALT_AP = 0x21,

	ID_E_W = 0x22,
	ID_N_S = 0x23,

	ID_ACC_X = 0x24, // m/s2
	ID_ACC_Y = 0x25,
	ID_ACC_Z = 0x26,

	ID_CURRENT = 0x28,

	ID_WHERE_DIST = 0x29, // metres the aircraft is way
	ID_WHERE_BEAR = 0x2a, // bearing (deg) to aircraft
	ID_WHERE_ELEV = 0x2b, // elevation (deg) of the aircraft above the horizon
	ID_WHERE_HINT = 0x2c, // which to turn to come home intended for voice guidance

	ID_COMPASS = 0x2d, // deg
	// 0x2e
	// 0x2f

	ID_VERT_SPEED = 0x30, // cm/sec

	//--------------------

	// UAVX user defined

	ID_GYRO_X = 0x31, // deg/sec
	ID_GYRO_Y = 0x32,
	ID_GYRO_Z = 0x33,

	ID_PITCH = 0x34, // deg
	ID_ROLL = 0x35,

	ID_MAH = 0x36, // mAH battery consumption

	//--------------------

	ID_VFAS = 0x39,
	ID_VOLTS_BP = 0x3A,
	ID_VOLTS_AP = 0x3B,
	ID_FRSKY_LAST = 0x3F,

};

#else

enum {
	// Data IDs  (BP = before decimal point; AP = after decimal point)
	ID_GPS_ALT_BP = 0x01,
	ID_GPS_ALT_AP = 0x09,
	ID_TEMP1 = 0x02,
	ID_RPM = 0x03,
	ID_FUEL = 0x04,
	ID_TEMP2 = 0x05,
	ID_VOLTS = 0x06,

	ID_WHERE_DIST = 0x07, // used by LUA - metres the aircraft is way

	ID_PITCH = 0x08,

	ID_ALT_BP = 0x10,
	ID_ALT_AP = 0x21,
	ID_GPS_SPEED_BP = 0x11,
	ID_GPS_SPEED_AP = 0x19,
	ID_GPS_LONG_BP = 0x12,
	ID_GPS_LONG_AP = 0x1A,
	ID_E_W = 0x22,
	ID_GPS_LAT_BP = 0x13,
	ID_GPS_LAT_AP = 0x1B,

	ID_COURSE_BP = 0x14,
	ID_COURSE_AP = 0x1C,
	ID_DAY_MONTH = 0x15,
	ID_YEAR = 0x16,
	ID_HOUR_MINUTE = 0x17,
	ID_SECOND = 0x18,

	ID_ROLL = 0x20,

	ID_BEEPER = 0x2E,

	ID_N_S = 0x23,
	ID_ACC_X = 0x24,
	ID_ACC_Y = 0x25,
	ID_ACC_Z = 0x26,
	ID_CURRENT = 0x28,

	// These have to be explicitly added to telemetry
	//ID_WHERE_DIST = 0x29,
	ID_WHERE_BEAR = 0x2a, // bearing (deg) to aircraft
	ID_WHERE_ELEV = 0x2b, // elevation (deg) of the aircraft above the horizon
	ID_WHERE_HINT = 0x2c, // which to turn to come home intended for voice guidance

	ID_COMPASS = 0x2d, // deg

	ID_VERT_SPEED = 0x30,

	ID_MAH = 0x36, // mAH battery consumption

	ID_VFAS = 0x39,
	ID_VOLTS_BP = 0x3A,
	ID_VOLTS_AP = 0x3B,

	// Incompatible with DJT decoder
	//ID_GYRO_X = 0x40,
	//ID_GYRO_Y = 0x41,
	//ID_GYRO_Z = 0x42,

	//ID_HOME_DIST = 0x07,
	//ID_PITCH = 0x08,
	//ID_ROLL = 0x20,

#define ID_VERT_SPEED         0x30 //opentx vario
	ID_FRSKY_LAST = 0x3D,
//opentx vario
};

#endif

#if defined(USE_FRSKY_D_TELEMETRY)

timeuS FrSkyDLinkuS;

uint16 MakeFrac(real32 v, uint16 s) {
	int32 t = (int32) (v * s);
	return (Abs(t) % s);
} // MakeFrac

void TxFrSkyDSentinel(uint8 s) {
	TxChar(s, FRSKY_SENTINEL);
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
	TxFrSky(s, a & 0xff);
	TxFrSky(s, (a >> 8) & 0xff);
} // TxFrSky16


void TxFrSkyDPacket(uint8 s, uint8 appID, int16 v) {
	TxFrSkyDSentinel(s);
	TxFrSky(s, appID);
	TxFrSky16(s, v);
} // TxFrSkyDPacket

void TxFrSkyDPacketPair(uint8 s, uint8 id1, uint8 id2, real32 v, int8 frac) {
	TxFrSkyDPacket(s, id1, v);
	TxFrSkyDPacket(s, id2, MakeFrac(v, frac));
}

void TxFrSkyDAcc(uint8 s) {
	uint8 a;

	for (a = X; a <= Z; a++)
		TxFrSkyDPacket(s, ID_ACC_X + a, Acc[a] * GRAVITY_MPS_S_R * 1000.0f);

} // TxFrSkyDAcc

void TxFrSkyDGyro(uint8 s) {
	uint8 a;

	//	for (a = Pitch; a <= Yaw; a++)
	//		TxFrSkyDPacket(s, ID_GYRO_X + a, RadiansToDegrees(Rate[a]));

} // TxFrSkyDAcc

void TxFrSkyDAttitude(uint8 s) {

	if (CurrTelType == UAVXDJTTelemetry) {
		TxFrSkyDPacket(s, ID_PITCH, RadiansToDegrees(Angle[Pitch]));
		TxFrSkyDPacket(s, ID_ROLL, RadiansToDegrees(Angle[Roll]));
	} else { // oddball pitch sign reversed!
		TxFrSkyDPacket(s, ID_PITCH, -RadiansToDegrees(Angle[Pitch]) * 10.0f); // oddball deci degrees
		TxFrSkyDPacket(s, ID_ROLL, RadiansToDegrees(Angle[Roll]) * 10.0f);
	}

} // TxFrSkyDAttitude

void TxFrSkyDBaro(uint8 s) {

	TxFrSkyDPacketPair(s, ID_ALT_BP, ID_ALT_AP, Limit1(Altitude, 999.9f), 10);
} //  TxFrSkyDBaro

void TxFrSkyDVario(uint8 s) {
	if (CurrTelType == UAVXDJTTelemetry)
		TxFrSkyDPacket(s, ID_VERT_SPEED, Limit1(ROC, 99.9f) * 10.0f);
	else
		TxFrSkyDPacket(s, ID_VERT_SPEED, Limit1(ROC, 99.9f) * 100.0f);
} //  TxFrSkyDBaro

void TxFrSkyDTemperature1(uint8 s) {
	TxFrSkyDPacket(s, ID_TEMP1, BaroTemperature);
} // TxFrSkyDTemperature1

void TxFrSkyDFuel(uint8 s) {
	TxFrSkyDPacket(
			s,
			ID_FUEL,
			Limit(100 * ( 1.0f - BatteryChargeUsedmAH / BatteryCapacitymAH), 0, 100));
} // TxFrSkyDFuel

void TxFrSkyDmAH(uint8 s) {
	TxFrSkyDPacket(s, ID_MAH, BatteryChargeUsedmAH);
} // TxFrSkyDmAH

void TxFrSkyDTemperature2(uint8 s) {
	TxFrSkyDPacket(s, ID_TEMP2, MPU6XXXTemperature);
} // TxFrSkyDTemperature2

void TxFrSkyDTime(uint8 s) {

	//#define FAKE_DATE

#if defined(FAKE_DATE)
	timemS seconds = mSClock() / 1000;
	uint8 minutes = (seconds / 60) % 60;

	// if we fly for more than an hour, something's wrong anyway
	TxFrSkyDPacket(s, ID_HOUR_MINUTE, minutes << 8);
	TxFrSkyDPacket(s, ID_SECOND, seconds % 60);
#else

	if (GPS.datevalid) {
		TxFrSkyDPacket(s, ID_DAY_MONTH, GPS.month << 8 + GPS.day);
		TxFrSkyDPacket(s, ID_YEAR, GPS.year);
		TxFrSkyDPacket(s, ID_HOUR_MINUTE, GPS.minute << 8 + GPS.hour);
		TxFrSkyDPacket(s, ID_SECOND, GPS.second);
	}

#endif

} // TxFrSkyDTime

void TxFrSkyDWhere(uint8 s) {
	if (F.GPSValid && F.OriginValid) {
		TxFrSkyDPacket(s, ID_WHERE_DIST, Limit(Nav.Distance,0,9999));
		TxFrSkyDPacket(s, ID_WHERE_BEAR, RadiansToDegrees(Nav.Bearing));
		TxFrSkyDPacket(s, ID_WHERE_ELEV, RadiansToDegrees(Nav.Elevation));
		TxFrSkyDPacket(s, ID_WHERE_HINT, RadiansToDegrees(Nav.Hint));
	}
}

// FrSky uses NMEA form rather than computationally sensible decimal degrees

typedef struct {
	uint16 bp, ap;
} pair_rec;

void GPStoDDDMM_MMMM(int32 L, pair_rec * c) {
#define GPS_DEGREES_DIVIDER 10000000L
#define FRSKY_FORMAT_DMS
	uint32 d, mf, dm, m;

	L = Abs(L);
	d = L / GPS_DEGREES_DIVIDER;
	L = (L - d * GPS_DEGREES_DIVIDER) * 60;
	m = L / GPS_DEGREES_DIVIDER;

#if defined(FRSKY_FORMAT_DMS)
	c->bp = d * 100 + m;
#else
	c->bp = d * 60 + m;
#endif

	c->ap = (L - m * GPS_DEGREES_DIVIDER) / 1000;

} // GPStoDDDMM_MMMM

void TxFrSkyDGPSStat(uint8 s) {

	switch (CurrTelType) {
	case UAVXDJTTelemetry:
	case iNavLUATelemetry:
		TxFrSkyDPacket(s, ID_TEMP2, GPS.noofsats * 1000 + GPS.fix * 100
				+ (F.GPSValid & 1) * 10 + (F.OriginValid & 1));
		break;

	default:
		break;
	} // switch

} // TxFrSkyDGPSStat

void TxFrSkyDGPSCoords(uint8 s) {
	pair_rec c;

	GPStoDDDMM_MMMM(GPS.C[NorthC].Raw, &c);
	TxFrSkyDPacket(s, ID_GPS_LAT_BP, c.bp);
	TxFrSkyDPacket(s, ID_GPS_LAT_AP, c.ap);
	TxFrSkyDPacket(s, ID_N_S, GPS.C[NorthC].Raw < 0 ? 'S' : 'N');

	GPStoDDDMM_MMMM(GPS.C[EastC].Raw, &c);
	TxFrSkyDPacket(s, ID_GPS_LONG_BP, c.bp);
	TxFrSkyDPacket(s, ID_GPS_LONG_AP, c.ap);
	TxFrSkyDPacket(s, ID_E_W, GPS.C[EastC].Raw < 0 ? 'W' : 'E');

} // TxFrSkyDGPS

void TxFrSkyDGPSSpeed(uint8 s) {

	switch (CurrTelType) {
	case UAVXDJTTelemetry:
		//TxFrSkyDPacketPair(s, ID_GPS_SPEED_BP, ID_GPS_SPEED_AP, GPS.gspeed, 10); // m/s
		TxFrSkyDPacketPair(s, ID_GPS_SPEED_BP, ID_GPS_SPEED_AP, GPS.gspeed
						* 1.94384449f, 10); // knots
		break;
	case iNavLUATelemetry:
		//TxFrSkyDPacketPair(s, ID_GPS_SPEED_BP, ID_GPS_SPEED_AP, GPS.gspeed, 10); // m/s
		TxFrSkyDPacketPair(s, ID_GPS_SPEED_BP, ID_GPS_SPEED_AP, GPS.gspeed
				* 1.94384449f, 10); // knots
		break;
	default:
		break;
	} // switch

} // TxFrSkyDGPSSpeed

void TxFrSkyDGPSAlt(uint8 s) {
	TxFrSkyDPacketPair(s, ID_GPS_ALT_BP, ID_GPS_ALT_AP,
			Limit1(GPS.altitude, 999.9f), 10);
} // TxFrSkyDGPSAlt

void TxFrSkyDCellVoltages(uint8 s) {
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

	TxFrSkyDPacket(s, ID_VOLTS, payload);

	if (++currentCell >= BatteryCellCount)
		currentCell = 0;

} // TxFrSkyDCellVoltages


void TxFrSkyDVoltage(uint8 s) {

	TxFrSkyDPacket(s, ID_VFAS, BatteryVolts * 10.0f);

} // TxFrSkyDVoltage


void TxFrSkyDCurrent(uint8 s) {
	TxFrSkyDPacket(s, ID_CURRENT, Limit(BatteryCurrent * 10.0f, 0,32000));
} // TxFrSkyDCurrent

void FrSkyDGPSCourseOverGround(uint8 s) {

	TxFrSkyDPacketPair(s, ID_COURSE_BP, ID_COURSE_AP,
			RadiansToDegrees(GPS.heading), 10);
	//RadiansToDegrees(Heading), 10);

} // FrSkyGPSCourse

void TxFrSkyDCompassHeading(uint8 s) {

	TxFrSkyDPacket(s, ID_COMPASS, RadiansToDegrees(Heading));
} // TxFrSkyDCompassHeading


#define BLADE_NUMBER_DIVIDER  5 // should set 12 blades in Taranis
uint16 TxFrSkyDGetFlightMode(void) {
	uint16 r;

	// we send all the flags as decimal digits for easy reading

	switch (CurrTelType) {
	case UAVXDJTTelemetry:

		r = Limit(State, 0, UnknownFlightState-1); // bits 0..4
		r |= Limit(NavState, 0, UnknownNavState-1) << 5; // bits 5..9

		// bits 10..14 (5 flags)
		if (F.HoldingAlt)
			r |= 0b000010000000000;
		if (F.RapidDescentHazard)
			r |= 0b000100000000000;
		if (F.LowBatt)
			r |= 0b001000000000000;
		if (Armed())
			r |= 0b0100000000000000;

		break;
	case iNavLUATelemetry:
		r = 10000; // force MSD - Taranis 5 digits

		// ones column
		if (State == Preflight)
			r += 1;
		else
			r += 2;

		if (Armed())
			r += 4;

		// tens column
		if (F.UsingAngleControl)
			r += 10;
		else
			r += 20;
		if (F.PassThru)
			r += 40;

		// hundreds column
		if (F.UsingTurnToWP)
			r += 100;
		if (F.HoldingAlt)
			r += 200;
		if (NavState == HoldingStation)
			r += 400;

		// thousands column
		if (F.ReturnHome)
			r += 1000;
		else if (F.Navigate)
			r += 2000;
		else
			r += 8000; //heading hold - default

		// else if (FLIGHT_MODE(HEADFREE_MODE))
		// r += 4000;

		// ten thousands column
		if (F.RapidDescentHazard)
			r += 10000;
		if (F.ForcedLanding)
			r += 40000;
		//else if (FLIGHT_MODE(AUTO_TUNE)) // intentionally reverse order and 'else-if' to prevent 16-bit overflow
		//r += 20000;

		break;
	default:
		r = 0;
		break;
	} // switch

	return r;

} // TxFrSkyDGetFlightMode


void TxFrSkyDThrottleOrBatterySizeAsRPM(uint8 s) {
	real32 r;

	if (Armed())
		r = Limit(DesiredThrottle + AltHoldThrComp, 0.0f, 1.0f) * 20.0f;
	else
		r = BatteryCapacitymAH * 0.2f;

	TxFrSkyDPacket(s, ID_RPM, r);

} // TxFrSkyDThrottleOrBatterySizeAsRPM

void TxFrSkyDFlightModeAsTemperature1(uint8 s) {
	TxFrSkyDPacket(s, ID_TEMP1, TxFrSkyDGetFlightMode());
} // TxFrSkyDFlightModeAsTemperature1

int16 TxFrSkyDGetGPSState(void) {
	int16 r = 0;

	// ones and tens columns (# of satellites 0 - 99)
	r += Limit(GPS.noofsats, 0, 99);

	// hundreds column (satellite accuracy HDOP: 0 = worst [HDOP > 5.5], 9 = best [HDOP <= 1.0])
	r += 100 * Limit((int16)(10.0f * (1.0f - GPS.hAcc / GPSMinhAcc)), 0, 9);

	//r += 100 * (9 - Limit((GPS.hdop - 0.51f) * 2.0f, 0, 9));

	// thousands column (GPS fix status)
	if (F.GPSValid)
		r += 1000;
	if (F.OriginValid)
		r += 2000;
	if (Armed() && !(F.Navigate || F.ReturnHome))
		r += 4000;

	return r;
} // TxFrSkyDGetGPSState

void TxFrSkyDSatelliteSignalQualityAsTemperature2(uint8 s) {
	TxFrSkyDPacket(s, ID_TEMP2, TxFrSkyDGetGPSState());
} // TxFrSkyDSatelliteSignalQualityAsTemperature2

void TxFrSkyDHeading(uint8 s) {
	TxFrSkyDPacketPair(s, ID_COURSE_BP, ID_COURSE_AP,
			RadiansToDegrees(Heading), 0);
} // TxFrSkyDHeading

void TxFrSkyDBeeper(uint8 s) {
	TxFrSkyDPacket(s, ID_BEEPER, BeeperIsOn());
} // TxFrSkyDBeeper


void SendFrSkyTelemetry(uint8 s) {
	static uint16 cycleNum = 0;
	timemS NowmS;

	NowmS = mSClock();

	if (NowmS >= mS[FrSkyTelemetryUpdatemS]) {
		mSTimer(FrSkyTelemetryUpdatemS, 125);

		cycleNum++;

		TxFrSkyDBeeper(s);
		TxFrSkyDAttitude(s); // every 125mS
		TxFrSkyDVario(s);

		if ((cycleNum % 4) == 0) { // Sent every 500ms
			TxFrSkyDBaro(s);
			FrSkyDGPSCourseOverGround(s);
			TxFrSkyDCompassHeading(s); // breaks DJT telemetry
		}

		if ((cycleNum % 8) == 0) { // Sent every 1s

			if (UAVXAirframe != Instrumentation) {
				TxFrSkyDFlightModeAsTemperature1(s);
				TxFrSkyDThrottleOrBatterySizeAsRPM(s);
			}

			//TxFrSkyDCellVoltages(s);
			TxFrSkyDVoltage(s);
			TxFrSkyDCurrent(s);
			TxFrSkyDFuel(s);

			TxFrSkyDGPSSpeed(s);
			TxFrSkyDWhere(s);
			TxFrSkyDGPSAlt(s);
			TxFrSkyDSatelliteSignalQualityAsTemperature2(s);
			TxFrSkyDGPSCoords(s);
		}

		if (cycleNum >= 40) { //Frame 3: Sent every 5s
			cycleNum = 0;
			// dodgy!! TxFrSkyDTime(s);
		}

		TxFrSkyDSentinel(s);
	}
} // SendFrSkyTelemetry


#elif defined(USE_FRSKY_S_TELEMETRY)

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
//BETA_VARIO_ID 0x8030
//BETA_BARO_ALT_ID 0x8010


const uint16 SPortID[] = {FSSP_ID_SPEED, FSSP_ID_VFAS, FSSP_ID_CURRENT,
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
	FSSP_ID_A4, 0};

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


void TxFrSkyTelemetry(uint8 s) {
	static uint8 appIDindex = 0;

	static uint8 currentCell = 0;
	uint32 payload = 0;

	int32 r;

	boolean Skip = false;

	do {
		uint16 appID = SPortID[appIDindex];

		switch (appID) {
			case FSSP_ID_SPEED:
			if (F.GPSValid)
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
			TxFrSkySPortPacket(s, appID, DensityAltitude);
			break;
			case FSSP_ID_FUEL:
			TxFrSkySPortPacket(
					s,
					appID,
					Limit(100 * ( 1.0f - BatteryChargeUsedmAH / BatteryCapacitymAH), 0, 100)); //%?
			break;
			//case FSSP_ID_ADC1       :
			//case FSSP_ID_ADC2       :

			case FSSP_ID_LATLONG:
			if (F.GPSValid) {
				uint32 tmpui = 0;

				if (appIDindex & 1) {
					tmpui = GPS.C[EastC].Raw & 0x3fffffff; // clip top bits
					tmpui = (uint32) ((real64) tmpui * 0.06);
					if (GPS.C[EastC].Raw < 0)
					tmpui |= 0x40000000;
					tmpui |= 0x80000000; // longitude
				} else {
					tmpui = GPS.C[NorthC].Raw & 0x3fffffff;
					tmpui = (uint32) ((real64) tmpui * 0.06);
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

			r = 10000; // force MSD - Taranis 5 digits

			if ((State == Ready) || (State == Starting))
			r += 1;
			if (State == Preflight)
			r += 2;
			if (Armed())
			r += 4;
			if (F.UsingAngleControl)
			r += 10;
			if (!F.UsingAngleControl)
			r += 20;
			if (F.PassThru)
			r += 40;
			if (F.MagnetometerActive)
			r += 100;
			if (F.BaroActive)
			r += 200;
			if (F.RangefinderActive)
			r += 400;
			if (NavState == HoldingStation)
			r += 1000;
			if (NavState == Transiting)
			r += 2000;
			//if (FLIGHT_MODE(HEADFREE_MODE))
			//r += 4000;

			TxFrSkySPortPacket(s, appID, (uint32) r);
			break;
			case FSSP_ID_T2:
			if (F.HaveGPS) // provide GPS lock status
			TxFrSkySPortPacket(s, appID, (F.GPSValid ? 1000 : 0)
					+ (F.OriginValid ? 2000 : 0) + GPS.noofsats);
			break;
			case FSSP_ID_GPS_ALT:
			if (F.GPSValid)
			TxFrSkySPortPacket(s, appID, GPS.altitude * 10); // given in 0.1m , requested in 10 = 1m (should be in mm, probably a bug in opentx, tested on 2.0.1.7)
			break;
			case FSSP_ID_A4:
			TxFrSkySPortPacket(s, appID, BatteryVolts * 10 / BatteryCellCount); //sending calculated average cell value with 0.01 precision
			break;
			default:
			Skip = true; // skip over default/empty or unused cases
			break;
			// if nothing is sent, FrSkySPortHasRequest isn't cleared, we already incremented the counter, just loop back to the start
		} // switch

		if (SPortID[++appIDindex] == 0)
		appIDindex = 0;

	}while (Skip);

} // TxFrSkySPort

#else

void TxFrSkyTelemetry(uint8 s) {} // TxFrSkyTelemetry

#endif

