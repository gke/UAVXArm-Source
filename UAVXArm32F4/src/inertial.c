// ===============================================================================================
// =                                UAVX Quadrocopter ContRoller                                 =
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

//    You should have received a copy of the GNU General Public License aint32 with this program.
//    If not, see http://www.gnu.org/licenses/

#include "UAVX.h"

real32 dT, dTR, dTOn2, dTROn2;
timeuS LastInertialUpdateuS = 0;
real32 AccConfidenceSDevR = 1.0f / 0.2f;
real32 AccConfidence;
real32 AccU = 0.0f;

HistStruct AccUF;

real32 EstMagHeading = 0.0f;

real32 CalculateAccConfidence(real32 AccMag) {
	// Gaussian decay in accelerometer value belief
	static real32 confp = 1.0f;
	real32 temp;

	if (F.Emulation)
		confp = 1.0f;
	else {
		temp = AccMag * GRAVITY_MPS_S_R;
		temp = expf(-0.5f * Sqr((1.0f - temp) * AccConfidenceSDevR));
		confp = LPF1(confp, temp, 0.1f);
	}

	return (confp);
} // CalculateAccConfidence

//=====================================================================================================
// AHRS
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// Gyroscope units are radians/second, accelerometer and magnetometer units are irrelevant as the
// vector is normalised.
//
// adapted from John Ihlein's AQ version translated to Aerospace Coordinates
//
// integral terms removed and accelerometer confidence scheme by Prof. G.K. Egan 2012
//
//=====================================================================================================

real32 TwoKpAccBase = 2.0f; // was 5.0f
real32 TwoKpAcc, TwoKpAcc2;
real32 KiAccBase = 0.0f; // no integral term
real32 KiAcc, KpMag;

real32 q0, q1, q2, q3;
real32 q0q0, q0q1, q0q2, q0q3;
real32 q1q1, q1q2, q1q3;
real32 q2q2, q2q3;
real32 q3q3;

real32 bi00, bi01, bi02;
real32 bi10, bi11, bi12;
real32 bi20, bi21, bi22;

void ConvertQuaternionToEuler(void) {

	bi00 = q0q0 + q1q1 - q2q2 - q3q3; // yaw
	//bi01 = 2.0f * (q1q2 - q0q3);
	//bi02 = 2.0f * (q0q2 + q1q3);

	bi10 = 2.0f * (q1q2 + q0q3);
	//bi11 = q0q0 - q1q1 + q2q2 - q3q3; // yaw
	//bi12 = 2.0f * (q2q3 - q0q1);

	bi20 = 2.0f * (q1q3 - q0q2); // pitch gx
	bi21 = 2.0f * (q0q1 + q2q3); // roll gy
	bi22 = q0q0 - q1q1 - q2q2 + q3q3; // yaw gz

	Angle[Roll] = atan2f(bi21, bi22);
	Angle[Pitch] = -asinf(bi20);
	Angle[Yaw] = atan2f(bi10, bi00);

} // ConvertQuaternionToEuler

void ConvertEulerToQuaternion(void) {
	real32 t0, t1, t2, t3, t4, t5, normR;

	t0 = cosf(Angle[Yaw] * 0.5f);
	t1 = sinf(Angle[Yaw] * 0.5f);
	t2 = cosf(Angle[Roll] * 0.5f);
	t3 = sinf(Angle[Roll] * 0.5f);
	t4 = cosf(Angle[Pitch] * 0.5f);
	t5 = sinf(Angle[Pitch] * 0.5f);

	q0 = t0 * t2 * t4 + t1 * t3 * t5;
	q1 = t0 * t3 * t4 - t1 * t2 * t5;
	q2 = t0 * t2 * t5 + t1 * t3 * t4;
	q3 = t1 * t2 * t4 - t0 * t3 * t5;

	// Just in case
	normR = invSqrt(Sqr(q0) + Sqr(q1) + Sqr(q2) + Sqr(q3));
	q0 *= normR;
	q1 *= normR;
	q2 *= normR;
	q3 *= normR;

} // ConvertEulerToQuaternion

real32 GravityCompensatedAccU(void) {

	return 2.0f * (q1q3 - q0q2) * Acc[X] + 2.0f * (q0q1 + q2q3) * Acc[Y]
			+ (q0q0 - q1q1 - q2q2 + q3q3) * Acc[Z];
} // GravityCompensatedAccU

real32 AttitudeCosine(void) { // for attitude throttle compensation

	return q0q0 - q1q1 - q2q2 + q3q3;

} // AttitudeCosine

void UpdateAccUVariance(real32 AccU) {
#define ACC_HIST_VAR_LENGTH 128
	const real32 AccU_K = 0.85f; // was 0.995

	static real32 A[ACC_HIST_VAR_LENGTH] = { 0.0f, };
	static uint16 i = 0;

	if (F.Hovering) {
		A[i++] = AccU;

		if (i >= ACC_HIST_VAR_LENGTH) {
			TrackAccUVariance = TrackAccUVariance * AccU_K
					+ CalculateVariance(A, ACC_HIST_VAR_LENGTH)
							* (1.0f - AccU_K);
			i = 0;
		}
	}

} // UpdateAccUVariance

void UpdateAccU(real32 AccUdT) { // vertical upwards acceleration in Earth coordinates

	// Accs are already filtered to CurrAccLPFHz ~20Hz.
	if (F.Emulation) {
		AccU = -(Acc[UD] + GRAVITY_MPS_S);
		AccUBias = 0.0f;
	} else {
		AccU = real32Median3Filter(&AccUM3F,
				-(GravityCompensatedAccU() + GRAVITY_MPS_S)); // attempt to strip outliers
		AccU = MAF(&AccUMAF, AccU);
	}

} // UpdateAccU

// REVISED MADGWICK

void InitMadgwick(void) {

	Angle[Roll] = asinf(Acc[LR]);
	Angle[Pitch] = asinf(-Acc[BF]);

	if (F.MagnetometerFailure)
		Angle[Yaw] = 0.0f;
	else {
		InitialMagHeading = CalculateMagneticHeading();
		Angle[Yaw] = InitialMagHeading;
	}

	ConvertEulerToQuaternion();

} // InitMadgwick

void UpdateHeading(void) {

	EstMagHeading += Rate[Yaw] * dT;
	EstMagHeading = Make2Pi(EstMagHeading);

	if (F.Emulation) {
		MagHeading = EstMagHeading;
		Heading = Make2Pi(MagHeading - MagVariation);
	} else {
		if (F.IsFixedWing && F.ValidGPSHeading) { // no wind adjustment for now

			Heading = GPS.heading;
			MagHeading = Make2Pi(Heading + MagVariation); // fake for compass reading

			// lock Yaw angle to GPS
			Angle[Yaw] = MagHeading;
			ConvertEulerToQuaternion();

		} else {
			MagHeading = Angle[Yaw];
			Heading = Make2Pi(MagHeading - MagVariation);
		}
	}

} // UpdateHeading

//___________________________________________________________________________

// Madgwick Quaternion version of Mahony et al.  Discrete Cosine Transform code
// rewritten by Prof G.K. Egan

void MadgwickUpdate(real32 gx, real32 gy, real32 gz, real32 ax, real32 ay,
		real32 az, real32 mx, real32 my, real32 mz) {

	real32 AccMag;
	real32 normR;

	real32 q0i, q1i, q2i, q3i;
	real32 hx, hy, hz;
	real32 bx, bz;
	real32 vx, vy, vz;
	real32 wx, wy, wz;

	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;

// estimated direction of gravity (v)
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
//vz = q0q0 - q1q1 - q2q2 + q3q3;
	vz = 2.0f * (q0q0 + q3q3) - 1.0f;

// error is sum of cross product between reference direction
// of fields and direction measured by sensors

	AccMag = sqrtf(Sqr(ax) + Sqr(ay) + Sqr(az));
	AccConfidence = CalculateAccConfidence(AccMag);
	TwoKpAcc = TwoKpAccBase * (State == InFlight ? AccConfidence : 15.0f);

// renormalise to attempt to remove a little acc vibration noise
	normR = 1.0f / AccMag;
	ax *= normR;
	ay *= normR;
	az *= normR;

	gx += (vy * az - vz * ay) * TwoKpAcc;
	gy += (vz * ax - vx * az) * TwoKpAcc;
	gz += (vx * ay - vy * ax) * TwoKpAcc;

	if (F.NewMagValues && !F.MagnetometerFailure) { // no compensation for latency
		F.NewMagValues = false;

		normR = invSqrt(Sqr(mx) + Sqr(my) + Sqr(mz));
		mx *= normR;
		my *= normR;
		mz *= normR;

		// reference direction of flux
		hx = 2.0f
				* (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3)
						+ mz * (q1q3 + q0q2));
		hy = 2.0f
				* (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3)
						+ mz * (q2q3 - q0q1));
		hz = 2.0f
				* (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1)
						+ mz * (0.5f - q1q1 - q2q2));

		bx = invSqrt(Sqr(hx) + Sqr(hy));
		bz = hz;

		// estimated direction of flux (w)
		wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
		wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
		wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

		gx += (my * wz - mz * wy) * KpMag;
		gy += (mz * wx - mx * wz) * KpMag;
		gz += (mx * wy - my * wx) * KpMag;
	}

// integrate quaternion rate
	gx *= dTOn2;
	gy *= dTOn2;
	gz *= dTOn2;

	q0i = -q1 * gx - q2 * gy - q3 * gz;
	q1i = q0 * gx + q2 * gz - q3 * gy;
	q2i = q0 * gy - q1 * gz + q3 * gx;
	q3i = q0 * gz + q1 * gy - q2 * gx;

// two steps to preserve old to new q
	q0 += q0i;
	q1 += q1i;
	q2 += q2i;
	q3 += q3i;

// normalize quaternion
	normR = invSqrt(Sqr(q0) + Sqr(q1) + Sqr(q2) + Sqr(q3));
	q0 *= normR;
	q1 *= normR;
	q2 *= normR;
	q3 *= normR;

} // MadgwickUpdate

void TrackPitchAttitude(void) {

	if (F.IsFixedWing && (DesiredThrottle < IdleThrottle)
			&& (State == InFlight)) { // gliding
		if (mSTimeout(GlidingTimemS))
			FWGlideAngleOffsetRad = LPF1(FWGlideAngleOffsetRad, Angle[Pitch],
					0.1f);
	} else
		mSTimer(GlidingTimemS, 5000);
} // TrackPitchAttitude

void UpdateInertial(void) {
	idx a;

	if ((F.Emulation) && (State == InFlight))
		DoEmulation(); // produces Accs, ROC, Altitude etc.
	else {
		ReadFilteredGyroAndAcc();
		ScaleRateAndAcc();
		GetMagnetometer();
	}

	UpdateAccU(dT);

	if (F.IMUCalibrated) {
		MadgwickUpdate(Rate[Roll], Rate[Pitch], Rate[Yaw], Acc[BF], Acc[LR],
				Acc[UD], Mag[X], Mag[Y], Mag[Z]);
		ConvertQuaternionToEuler();
	}
#if defined(MADGWICK_TESTING)

	IntRate[Pitch] += Rate[Pitch] * dT;
	IntRate[Yaw] += Rate[Yaw] * dT;

#endif

	DoControl();

// one cycle delay OK
	UpdateHeading();

	if (F.NewGPSPosition) {
		F.NewGPSPosition = false;

		for (a = NorthC; a <= DownC; a++) {
			Nav.C[a].Pos = GPS.C[a].Pos;
			Nav.C[a].Vel = GPS.C[a].Vel;
		}

		UpdateWhere();

		F.NavigationEnabled = true;
		F.NewNavUpdate = true;
	}

	UpdateAltitudeEstimates();

	UpdateAirspeed();

	TrackPitchAttitude();

} // UpdateInertial

