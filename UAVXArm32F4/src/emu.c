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

const real32 EmulatedWind[2] = { 0.0f, 1.0f };

#define HR_GPS

typedef struct {
	real32 Pos;
	real32 Vel;
	real32 Acc;
} EmStruct;

uint32 BINGO = 100000;

EmStruct Aircraft[3];

real32 NorthHP, EastHP;
real32 FakeROC = 0.0f;
real32 FakeAltitude = 0.0f;
real32 FakeAccZ = -GRAVITY_MPS_S;

real32 Drag(real32 v) {
	return (Sign(v) * Sqr(v) * EM_DRAG_SCALE);
} // Drag

real32 Thermal(real32 East, real32 North) {
#if defined(USE_THERMALS)
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
#else
	return 0.0f;
#endif

} // Thermal


void DoEmulatedAltitude(real32 AltdT) {

} // DoEmulatedAltitude

void DoEmulation(void) {
	static real32 PrevFakeROC = 0.0f;
	const real32 RollPitchInertiaR = (12.0f / (EM_MASS * Sqr(EM_ARM_LEN)));
	const real32 InertiaR[3] = { RollPitchInertiaR, RollPitchInertiaR,
			RollPitchInertiaR * 3.0f };
	const real32 FWRollPitchInertiaR = (12.0f / (FW_MASS * Sqr(FW_ARM_LEN)));
	const real32 FWInertiaR[3] = { FWRollPitchInertiaR, FWRollPitchInertiaR,
			FWRollPitchInertiaR * 3.0f };
	real32 Temp, Thrust, EffSink, dThrottle;
	int32 a;

	if (State = InFlight) {
		if (F.IsFixedWing) {

			dThrottle
					= Limit(DesiredThrottle + AltHoldThrComp - CruiseThrottle, -1.0f, 1.0f);

			Thrust = (DesiredThrottle + AltHoldThrComp) * EM_MAX_THRUST_FW; // zzz needs tidying up
			Airspeed
					= Limit((1.0f + dThrottle) * EM_CRUISE_FW_MPS, AS_MIN_MPS, AS_MAX_MPS);

			EffSink = (EXP_THERMAL_SINK_MPS + Sl * VRSDescentRateMPS) / cosf(
					Angle[Roll]);

			PrevFakeROC = FakeROC;
			FakeROC = (dThrottle * 30.0f + EffSink);
			FakeROC = Limit(FakeROC, EffSink, 1000.0f);
			FakeROC += Thermal(Nav.C[EastC].Pos, Nav.C[NorthC].Pos);

			FakeAccZ = (FakeROC - PrevFakeROC) * dTR;
#define NEW_FW
#if defined(NEW_FW)

			for (a = Pitch; a <= Roll; a++)
				Rate[a] -= (A[a].Out * A[a].R.Max * FWInertiaR[a] - 2.0f
						* Sign(A[a].Out) * Sqr(Rate[a])) * dT;

			if (Airspeed > 0.0f)
				Rate[Yaw] += (A[Yaw].Out * A[Yaw].R.Max * FWInertiaR[Yaw]
						- 2.0f // 0.25f *
								* Sign(A[Yaw].Out) * Sqr(Rate[Yaw])) * dT;
			else
				Rate[Yaw] = 0.0f;

#else
			for (a = Pitch; a <= Roll; a++)
			Rate[a] -= A[a].Out * DegreesToRadians(60) * dT;

			if (Airspeed > 0.0f)
			Rate[Yaw] = A[Yaw].Out * DegreesToRadians(30) //A[a].R.Max * 0.2f
#define SUPPRESS_FULL_ROLL_MATH
#if defined(SUPPRESS_FULL_ROLL_MATH)
			+ Angle[Roll] / (2.0f * NAV_MAX_ANGLE_RAD);// Nav.MaxBankAngle);
#else
			+ tanf(Limit1(Angle[Roll], Nav.MaxBankAngle))
			* GRAVITY_MPS_S / Airspeed;
#endif
			else
			Rate[Yaw] = 0.0f;

#endif

			Aircraft[Pitch].Vel = -Airspeed; //-EM_CRUISE_MPS;

		} else {
			Thrust = (DesiredThrottle + AltHoldThrComp) * EM_MAX_THRUST
					* AttitudeCosine();
			FakeAccZ = (Thrust - EM_MASS * GRAVITY_MPS_S - Drag(FakeROC))
					* EM_MASS_R;
			FakeROC += FakeAccZ * dT + Thermal(Nav.C[EastC].Pos,
					Nav.C[NorthC].Pos) * 0.01f;

			for (a = Pitch; a <= Roll; a++) {
				Rate[a] -= (EM_MAX_THRUST * 0.25f * A[a].Out * EM_ARM_LEN
						* InertiaR[a] - 2.0f * Sign(A[a].Out) * Sqr(Rate[a]))
						* dT;
				Temp = sinf(Angle[a]) * Thrust;
				Temp = Temp - Drag(Aircraft[a].Vel);
				Aircraft[a].Vel += (Temp * EM_MASS_R) * dT;
			}

			Rate[Yaw] += (EM_MAX_YAW_THRUST * A[Yaw].Out * EM_ARM_LEN
					* InertiaR[a] - 2.0f * Sign(A[Yaw].Out) * Sqr(Rate[Yaw]))
					* dT;
			Rate[Yaw] = Limit1(Rate[Yaw], DegreesToRadians(180.0f));
		}

		if (FakeAltitude < 0.0f)
			FakeAltitude = FakeROC = 0.0f;
		else
			FakeAltitude += FakeROC * dT;

		Rotate(&GPS.C[NorthC].Vel, &GPS.C[EastC].Vel, -Aircraft[Pitch].Vel,
				Aircraft[Roll].Vel, -Heading);

		Acc[BF] = sinf(Angle[Pitch]) * GRAVITY_MPS_S; // TODO: needs further work to cover lateral acc.
		Acc[LR] = -sinf(Angle[Roll]) * GRAVITY_MPS_S;

		for (a = NorthC; a <= EastC; a++) {
			GPS.C[a].Vel += EmulatedWind[a];
			GPS.C[a].Pos += GPS.C[a].Vel * dT;
		}

		Acc[UD] = -FakeAccZ - GRAVITY_MPS_S;
		GPS.C[DownC].Vel = GPS.velD = -FakeROC;
		GPS.altitude = FakeAltitude;

		GPS.gspeed = sqrtf(Sqr(GPS.C[EastC].Vel) + Sqr(GPS.C[NorthC].Vel));

		GPS.C[NorthC].Raw = GPS.lat = DEFAULT_HOME_LAT + MToGPS(GPS.C[NorthC].Pos);
		GPS.C[EastC].Raw = GPS.lon = DEFAULT_HOME_LON + MToGPS(GPS.C[EastC].Pos) / GPS.longitudeCorrection;

	} else {

		for (a = Pitch; a <= Yaw; a++)
			Aircraft[a].Vel = Aircraft[a].Acc = GPS.C[a].Vel = Rate[a] = Acc[a]
					= Angle[a] = A[a].Out = 0.0f;
		Altitude = FakeROC = FakeAccZ = 0.0f;
	}

} // DoEmulation


void GPSEmulation(void) {

	if (F.HaveGPS)
		while (SerialAvailable(GPSSerial))
			RxChar(GPSSerial); // flush

	if (mSTimeout(FakeGPSUpdatemS)) { // && ((mSClock() < BINGO) || (mSClock() > BINGO + 10000))) {
		GPS.lastPosUpdatemS = GPS.lastVelUpdatemS = mSClock();
		mSTimer(FakeGPSUpdatemS, FAKE_GPS_DT_MS);

		GPS.heading = Heading;
		GPS.cAcc = 0.5f; // degrees

		GPSPacketTag = GPGGAPacketTag;
		GPS.fix = 3;
		GPS.noofsats = 10;
		GPS.hDOP = 0.9f;
		GPS.hAcc = GPSMinhAcc * 0.25f;
		GPS.vAcc = GPSMinvAcc * 0.25f;
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

		GPS.C[NorthC].OriginRaw = GPS.C[NorthC].Raw = DEFAULT_HOME_LAT;
		GPS.C[EastC].OriginRaw = GPS.C[EastC].Raw = DEFAULT_HOME_LON;
		GPS.longitudeCorrection = DEFAULT_LON_CORR;

		mS[FakeGPSUpdatemS] = 0;

		Altitude = OriginAltitude = RangefinderAltitude = FakeAltitude
				= FakeROC = 0.0f;

		SetDesiredAltitude(0.0f);
	}
} // InitEmulation
