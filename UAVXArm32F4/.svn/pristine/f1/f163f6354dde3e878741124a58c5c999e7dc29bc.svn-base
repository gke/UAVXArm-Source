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

#define FAKE_GPS_DT_MS	 200

#define EM_MASS  1.2f // Kg
#define EM_ARM_LEN 0.3f // M
#define EM_MASS_R (1.0f/EM_MASS)
#define EM_MAX_THRUST ((EM_MASS/THR_DEFAULT_CRUISE_STICK)*GRAVITY_MPS_S)
#define EM_MAX_YAW_THRUST (EM_MAX_THRUST*0.015f) // TODO: tweak!
#define EM_CRUISE_MPS ((AS_MIN_MPS+AS_MAX_MPS)*0.5f)
//#define EM_DRAG_SCALE 1.0f
#define EM_DRAG_SCALE (EM_MAX_THRUST/(AS_MAX_MPS*AS_MAX_MPS)) // 0.004f?
#define EM_MASS_FW 2.0f //Kg
#define EM_MAX_THRUST_FW ((EM_MASS_FW/THR_DEFAULT_CRUISE_FW_STICK)*GRAVITY_MPS_S)

const real32 Wind[2] = { 0.0f, 1.0f };

#define HR_GPS

typedef struct {
	real32 Pos;
	real32 Vel;
	real32 Acc;
} EmStruct;

EmStruct Aircraft[3];

real32 NorthHP, EastHP;
real32 ROC = 0.0f;
real32 FakeAltitude = 0.0f;

real32 Drag(real32 v) {
	return (Sign(v) * Sqr(v) * EM_DRAG_SCALE);
} // Drag

real32 Thermal(real32 East, real32 North) {
#define NT 2
	const struct {
		real32 NorthPos;
		real32 EastPos;
		real32 Radius; // could make them oval -> ridge?
		real32 Strength;
	} T[NT] = { { 200, 200, 50, 4 }, { 300, 50, 65, 7 } };
	real32 Uplift;
	int16 t;

	Uplift = 0.0f;

	if (State == InFlight)
		for (t = 0; t < NT; t++)
			Uplift += T[t].Strength * exp(
					-(Sqr(Nav.C[NorthC].Pos - T[t].NorthPos) / (2.0f
							* Sqr(T[t].Radius))
							+ Sqr(Nav.C[EastC].Pos - T[t].EastPos) / (2.0f
									* Sqr(T[t].Radius))));

	return Uplift;

} // Thermal


void DoEmulation(void) {
	static uint32 LastAltUpdatemS = 0;
	const real32 RollPitchInertiaR = (12.0f / (EM_MASS * Sqr(EM_ARM_LEN)));
	const real32 InertiaR[3] = { RollPitchInertiaR, RollPitchInertiaR,
			RollPitchInertiaR * 3.0f };
	real32 Temp, Accel, Thrust;
	int32 a;
	real32 EffSink;
	uint32 NowmS;

	if (F.IsFixedWing) {

		real32
				dThrottle =
						Limit(DesiredThrottle + AltComp - CruiseThrottle, -1.0f, 1.0f);

		Thrust = (DesiredThrottle + AltComp) * EM_MAX_THRUST_FW; // zzz needs tidying up
		Airspeed
				= Limit((1.0 + dThrottle) * EM_CRUISE_MPS, AS_MIN_MPS, AS_MAX_MPS);

		EffSink = (EXP_THERMAL_SINK_MPS + Sl * VRSDescentRateMPS) / cosf(
				A[Roll].Angle);
		ROC = (dThrottle * 30.0f); // + EffSink);

		ROC = Limit(ROC, EffSink, 1000.0f);
#if defined(USE_THERMALS)
		ROC += Thermal(Nav.C[EastC].Pos, Nav.C[NorthC].Pos);
#endif
	} else {
		Thrust = (DesiredThrottle + AltComp) * EM_MAX_THRUST;
		Accel = (Thrust - EM_MASS * GRAVITY_MPS_S - Drag(ROC)) * EM_MASS_R;
		ROC += Accel * dT;
	}

	ROCF = LPFn(&FROCLPF, ROC, AltdT); // used for landing and cruise throttle tracking

	if (((State != InFlight) && (State != Launching)) || ((Altitude <= 0.05f)
			&& (ROC <= 0.0f)))
		FakeAltitude = ROC = 0.0f;
	else
		FakeAltitude += ROC * dT;

	GPS.altitude = BaroAltitude = FakeAltitude + OriginAltitude;
	Altitude = FakeAltitude;
	F.BaroActive = true;

	NowmS = mSClock();
	if (NowmS > mS[AltUpdate]) { // 5 cycles @ 10mS -> 50mS or 20Hz
		mSTimer(NowmS, AltUpdate, ALT_UPDATE_MS);

		AltdT = (NowmS - LastAltUpdatemS) * 0.001f;
		AltdTR = 1.0f / AltdT;
		LastAltUpdatemS = NowmS;
		F.NewAltitudeValue = true;
	}

	if (FakeAltitude <= 0.05f) {
		for (a = Pitch; a <= Yaw; a++)
			Aircraft[a].Vel = Aircraft[a].Acc = GPS.C[a].Vel = Rate[a] = Acc[a]
					= A[a].Angle = A[a].Out = 0.0f;
	} else {

		if (F.IsFixedWing) { // no inertial effects for fixed wing
			for (a = Pitch; a <= Roll; a++)
				Rate[a] -= (A[a].Out * DegreesToRadians(30)) * dT; // was 60

			if (Airspeed > 0.0f)
				Rate[Yaw] = A[Yaw].Out * A[Yaw].R.Max
				//+ A[Roll].Angle / (2.0f
						//		* A[Roll].AngleMax);
						+ Limit1(A[Roll].Angle, A[Roll].P.Max) / (Airspeed
								* GRAVITY_MPS_S_R);
			else
				Rate[Yaw] = 0.0f;

			Aircraft[Pitch].Vel = -Airspeed; //-EM_CRUISE_MPS;

		} else {

			for (a = Pitch; a <= Roll; a++) {
				Rate[a] -= (EM_MAX_THRUST * 0.25f * A[a].Out * EM_ARM_LEN
						* InertiaR[a] - 2.0f * Sign(A[a].Out) * Sqr(Rate[a]))
						* dT;
				Temp = sinf(A[a].Angle) * Thrust;
				Temp = Temp - Drag(Aircraft[a].Vel);
				Aircraft[a].Vel += (Temp * EM_MASS_R) * dT;
			}

			Rate[Yaw] += (EM_MAX_YAW_THRUST * A[Yaw].Out * EM_ARM_LEN
					* InertiaR[a] - 2.0f * Sign(A[Yaw].Out) * Sqr(Rate[Yaw]))
					* dT;
			Rate[Yaw] = Limit1(Rate[Yaw], DegreesToRadians(180.0f));
		}

		Rotate(&GPS.C[NorthC].Vel, &GPS.C[EastC].Vel, -Aircraft[Pitch].Vel,
				Aircraft[Roll].Vel, -Heading);

		Acc[BF] = sinf(A[Pitch].Angle) * GRAVITY_MPS_S; // TODO: needs further work to cover lateral acc.
		Acc[LR] = -sinf(A[Roll].Angle) * GRAVITY_MPS_S;

		for (a = NorthC; a <= EastC; a++) {
			GPS.C[a].Vel += Wind[a];
			GPS.C[a].Pos += GPS.C[a].Vel * dT;
		}
	}

	Acc[UD] = -GRAVITY_MPS_S;

	GPS.C[EastC].Raw = DEFAULT_HOME_LON + MToGPS(GPS.C[EastC].Pos)
			/ GPS.longitudeCorrection;
	GPS.C[NorthC].Raw = DEFAULT_HOME_LAT + MToGPS(GPS.C[NorthC].Pos);

	GPS.gspeed = sqrtf(Sqr(GPS.C[EastC].Vel) + Sqr(GPS.C[NorthC].Vel));
	GPS.velD = -ROC;

} // DoEmulation


void GPSEmulation(void) {
	uint32 NowmS;

	while (SerialAvailable(GPSRxSerial))
		RxChar(GPSRxSerial); // flush

	NowmS = mSClock();
	if (NowmS > mS[FakeGPSUpdate]) {
		GPS.lastPosUpdatemS = GPS.lastVelUpdatemS = mSClock();
		mSTimer(NowmS, FakeGPSUpdate, FAKE_GPS_DT_MS);

		GPS.heading = Heading;
		GPS.cAcc = 0.5f; // degrees

		GPSPacketTag = GPGGAPacketTag;
		GPS.fix = 3;
		GPS.noofsats = 10;
		GPS.hDOP = 0.9f;
		GPS.hAcc = GPS.hDOP * GPS_HDOP_TO_HACC;
		GPS.vAcc = 1.5f;
		GPS.sAcc = 1.0f;
		F.GPSValid = F.GPSPacketReceived = F.NewGPSPosition = true;
	}

} // GPSEmulation


void InitEmulation(void) {
	int32 a;

	if (F.Emulation) {
		for (a = Pitch; a <= Yaw; a++)
			GPS.C[a].Pos = GPS.C[a].Vel = Aircraft[a].Pos = Aircraft[a].Vel
					= Aircraft[a].Acc = 0.0f;

		GPS.C[NorthC].OriginRaw = 0;
		GPS.C[EastC].OriginRaw = 0;
		GPS.C[NorthC].Raw = DEFAULT_HOME_LAT;
		GPS.C[EastC].Raw = DEFAULT_HOME_LON;
		GPS.longitudeCorrection = DEFAULT_LON_CORR;

		mS[FakeGPSUpdate] = 0;

		Altitude = RangefinderAltitude = FakeAltitude = ROC = 0.0f;
		BaroAltitude = OriginAltitude;
		SetDesiredAltitude(0.0f);
	}
} // InitEmulation
