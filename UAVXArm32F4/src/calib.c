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

// Calibration extensions

#include "UAVX.h"

// Least squares fit of a sphere to 3D data, ImaginaryZ's blog,
// Miscellaneous banter, Useful mathematics, game programming tools and the occasional kink or two.
// 22 April 2011.
// http: imaginaryz.blogspot.com.au/2011/04/least-squares-fit-sphere-to-3d-data.html

// Substantially rewritten for UAVXArm by Prof. G.K. Egan (C) 2012.


uint16 SphereFit(real32 d[][3], uint16 N, uint16 MaxIterations, real32 Err,
		real32 SphereOrigin[], real32 * SphereRadius) {

	idx a;
	uint16 i, Iterations;
	real32 s[3], s2[3], s3[3], sum[3], sum2[3], sum3[3];
	real32 x2sum[3], y2sum[3], z2sum[3];
	real32 xy_sum, xz_sum, yz_sum;
	real32 xy, xz, yz, x2z, y2x, y2z, z2x, x2y, z2y;
	real32 qs, qb, q0, q1, q2;
	real32 r2, c[3], c2[3], Delta[3], Denom[3];
	real32 f0, f1, f2, f3, f4;
	real32 di2[3];
	real32 SizeR;

	for (a = X; a <= Z; a++)
		s[a] = s2[a] = s3[a] = sum[a] = x2sum[a] = y2sum[a] = z2sum[a] = 0.0f;

	xy_sum = xz_sum = yz_sum = 0.0f;

	for (i = 0; i < N; i++) {

		for (a = X; a <= Z; a++) {
			di2[a] = Sqr(d[i][a]);
			s[a] += d[i][a];
			s2[a] += di2[a];
			s3[a] += di2[a] * d[i][a];
		}

		///zzzzzzzzzz

		xy_sum += d[i][X] * d[i][Y];
		xz_sum += d[i][X] * d[i][Z];
		yz_sum += d[i][Y] * d[i][Z];

		x2sum[Y] += di2[X] * d[i][Y];
		x2sum[Z] += di2[X] * d[i][Z];

		y2sum[X] += di2[Y] * d[i][X];
		y2sum[Z] += di2[Y] * d[i][Z];

		z2sum[X] += di2[Z] * d[i][X];
		z2sum[Y] += di2[Z] * d[i][Y];
	}

	SizeR = 1.0f / (real32) N;
	for (a = X; a <= Z; a++) {
		sum[a] = s[a] * SizeR; //sum( X[n] )
		sum2[a] = s2[a] * SizeR; //sum( X[n]^2 )
		sum3[a] = s3[a] * SizeR; //sum( X[n]^3 )
	}

	xy = xy_sum * SizeR; //sum( X[n] * Y[n] )
	xz = xz_sum * SizeR; //sum( X[n] * Z[n] )
	yz = yz_sum * SizeR; //sum( Y[n] * Z[n] )

	x2y = x2sum[Y] * SizeR; //sum( X[n]^2 * Y[n] )
	x2z = x2sum[Z] * SizeR; //sum( X[n]^2 * Z[n] )
	y2x = y2sum[X] * SizeR; //sum( Y[n]^2 * X[n] )
	y2z = y2sum[Z] * SizeR; //sum( Y[n]^2 * Z[n] )
	z2x = z2sum[X] * SizeR; //sum( Z[n]^2 * X[n] )
	z2y = z2sum[Y] * SizeR; //sum( Z[n]^2 * Y[n] )

	//Reduction of multiplications
	f0 = sum2[X] + sum2[Y] + sum2[Z];
	f1 = 0.5f * f0;
	f2 = -8.0f * (sum3[X] + y2x + z2x);
	f3 = -8.0f * (x2y + sum3[Y] + z2y);
	f4 = -8.0f * (x2z + y2z + sum3[Z]);

	for (a = X; a <= Z; a++) {
		c[a] = sum[a];
		c2[a] = Sqr(c[a]);
	}

	qs = c2[X] + c2[Y] + c2[Z];
	qb = -2.0f * (Sqr(c[X]) + Sqr(c[Y]) + Sqr(c[Z]));
	r2 = f0 + qb + qs;
	q0 = 0.5f * (qs - r2);
	q1 = f1 + q0;
	q2 = 8.0f * (qs - r2 + qb + f0);

	Iterations = 0;
	do {
		for (a = X; a <= Z; a++) {
			Denom[a] = q2 + 16.0f * (c2[a] - 2.0f * c[a] * sum[a] + sum2[a]);
			if (Denom[a] == 0.0f)
				Denom[a] = 1.0f;
		}

		Delta[X] = -((f2 + 16.0f * (c[Y] * xy + c[Z] * xz + sum[X] * (-c2[X]
				- q0) + c[X] * (sum2[X] + q1 - c[Z] * sum[Z] - c[Y] * sum[Y])))
				/ Denom[X]);
		Delta[Y] = -((f3 + 16.0f * (c[X] * xy + c[Z] * yz + sum[Y] * (-c2[Y]
				- q0) + c[Y] * (sum2[Y] + q1 - c[X] * sum[X] - c[Z] * sum[Z])))
				/ Denom[Y]);
		Delta[Z] = -((f4 + 16.0f * (c[X] * xz + c[Y] * yz + sum[Z] * (-c2[Z]
				- q0) + c[Z] * (sum2[Z] + q1 - c[X] * sum[X] - c[Y] * sum[Y])))
				/ Denom[Z]);

		for (a = X; a <= Z; a++) {
			c[a] += Delta[a];
			c2[a] = Sqr(c[a]);
		}

		qs = c2[X] + c2[Y] + c2[Z];
		qb = -2.0f * (c[X] * sum[X] + c[Y] * sum[Y] + c[Z] * sum[Z]);
		r2 = f0 + qb + qs;
		q0 = 0.5f * (qs - r2);
		q1 = f1 + q0;
		q2 = 8.0f * (qs - r2 + qb + f0);

		Iterations++;
	} while ((Iterations < MaxIterations) && ((Sqr(Delta[X]) + Sqr(Delta[Y])
			+ Sqr(Delta[Z])) > Err));

	for (a = X; a <= Z; a++)
		SphereOrigin[a] = c[a];

	//	*SphereRadius = sqrtf(Abs(r2));

	return (Iterations);
}
// SphereFit


#if defined(RETIRED_STUFF)

int8 getCurrDir(real32 s[3]) {
	const real32 r = atanf(DegreesToRadians(10));

	if ((Abs(s[Z]) * r) > Abs(s[X]) && (Abs(s[Z]) * r) > Abs(s[Y]))
		return (s[Z] > 0) ? 0 : 1;
	else if ((Abs(s[X]) * r) > Abs(s[Y]) && (Abs(s[X]) * r) > Abs(s[Z]))
		return (s[X] > 0) ? 2 : 3;
	else if ((Abs(s[Y]) * r) > Abs(s[X]) && (Abs(s[Y]) * r) > Abs(s[Z]))
		return (s[Y] > 0) ? 4 : 5;
	else
		return -1;
} // getCurrDir

real32 sp[ACC_CAL_SPHERE_CYCLES * 6][3];

void CalibrateAccSixPointSphere(uint8 s, uint8 imuSel) {

#define KPACC 0.9f

	int16 calDirCnt = 0;
	idx a;
	int8 Currdp, dp;
	uint16 N;
	real32 SphereOrigin[3];
	real32 SphereRadius;
	boolean axisCalibrated[3];
	uint16 calCycles;

	LEDOn(ledBlueSel);

	for (a = X; a <= Z; a++)
		axisCalibrated[a] = false;

	N = 0;
	calDirCnt = 0;

	do {

		ReadFilteredGyroAndAcc(imuSel);
		dp = getCurrDir(RawAcc);

		if (dp >= 0) {

			if (!axisCalibrated[dp]) { // assume stays in window
				Currdp = dp;

				DoBeep(2, 1);
				LEDOff(ledRedSel);

				Delay1mS(100); // short delay to reduce post move shake

				calCycles = 0;
				do {
					Delay1mS(2);

					ReadFilteredGyroAndAcc(imuSel);
					dp = getCurrDir(RawAcc);

					if (Currdp == dp) {
						LEDOff(ledYellowSel);
						LEDOn(ledGreenSel);
						for (a = X; a <= Z; a++)
							sp[N][a] = RawAcc[a];

						N++;
						calCycles++;
					} else {
						LEDOn(ledYellowSel);
						LEDOff(ledGreenSel);
					}

				} while (calCycles < ACC_CAL_SPHERE_CYCLES);

				axisCalibrated[dp] = true;
				calDirCnt++;

				DoBeep(5, 1);
				LEDOff(ledYellowSel);
				LEDOff(ledGreenSel);
			} else
				LEDOn(ledRedSel);
		}

	} while (calDirCnt != 6);

	LEDOff(ledBlueSel);

	SphereFit(sp, ACC_CAL_SPHERE_CYCLES * 6, 50, 0.01f, SphereOrigin, &SphereRadius);

	for (a = X; a <= Z; a++) {
		Config.AccCal.Scale[a] = SphereRadius * DEF_ACC_SCALE;
		Config.AccCal.Bias[a] = SphereOrigin[a];
	}

	Config.AccCal.Calibrated = 1;
	F.AccCalibrated = true;

	ConfigChanged = true;
	RefreshConfig();

	DoBeeps(2);

} // CalibrateAccSixPointSphere

#endif



