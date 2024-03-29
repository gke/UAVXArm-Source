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

//const
real32 AccUBiasVariance = 0.000001f; // fairly slow convergence but less noisy
real32 BaroVariance = 0.35f;
real32 AccUVariance = 2.0f;
real32 GPSVariance = 2.0f;

real32 AccUBias = 0.0f; // maybe should capture and use slower tracking?

real32 TrackBaroVariance;
real32 TrackAccUVariance;

real32 KFROC, KFDensityAltitude;

// https://github.com/har-in-air
// Pretty much a common implementation

// Tracks the position z and velocity v of an object moving in a straight line,
// (here assumed to be vertical) that is perturbed by random accelerations.
// sensor measurement of z is assumed to have constant measurement noise
// variance zVariance,

// Updates state given a sensor measurement of z, acceleration a,
// and the time in seconds dT since the last measurement.

// Assumes MEAS MS5611 barometeric pressure sensor, Maxim MAX21100 gyro+accel, Madgwick AHRS algorithm

// Z_VARIANCE and ZACCEL_VARIANCE can be configured to shape the response of the filter to your specific application and personal
// preferences

// Z_VARIANCE is the measured sensor altitude metre noise variance, with sensor at rest, normal sampling rate, and 1-2 seconds max samples
// I personally use a somewhat smaller value than the measured variance as I favour a faster response to step inputs and am willing to
// tolerate a bit of jitter with the unit at rest.
#define Z_VARIANCE		    3.0f

// The filter models acceleration as an external environmental disturbance to the system, ZACCEL_VARIANCE is the estimated
// variance of the perturbations. For a paragliding application, this would be the expected acceleration variance due to thermal
// activity, how sharp the thermal edges are, etc.  This is NOT the accelerometer sensor noise variance. Increase this value and the
// filter will respond faster to acceleration inputs.
#define ZACCEL_VARIANCE	    2.0f

// The accelerometer bias (offset between true acceleration and measured value) is not likely to change rapidly, so a low value
// of ZACCELBIAS_VARIANCE will enforce that. But too low a value, and the filter will take longer on reset to settle to the estimated
// value. For an audio variometer, the symptom would be the variometer beeping for several seconds after reset, with the unit at rest.
#define ZACCELBIAS_VARIANCE 0.01f

real32 Pzh = 1.0f;
real32 Pzv = 0.0f;
real32 Pza_ = 0.0f;

real32 Pvh = 0.0f;
real32 Pvv = 1.0f;
real32 Pva_ = 0.0f;

real32 Pah = 0.0f;
real32 Pav = 0.0;
real32 Paa_ = 100000.0f;

void AltitudeKF(real32 DensityAlt, real32 GPSAlt, real32 AccU, real32 dT) {
	static boolean First = true;
	real32 Acceleration;

	if (First) {
		KFDensityAltitude = DensityAlt;
		KFROC = AccUBias = 0.0f;
		First = false;
	}

	// Predict state
	Acceleration = AccU - AccUBias;
	KFROC += Acceleration * dT;
	KFDensityAltitude += KFROC * dT;

	// Original tracking scheme
	// AccUVariance = Abs(Acceleration) * 2.0f;
	// AccUVariance = Limit(AccUVariance, 0.01f, 0.5f);

	// Predict State Covariance matrix
	real32 t00, t01, t02;
	real32 t10, t11, t12;
	real32 t20, t21, t22;

	real32 dT2div2 = Sqr(dT) * 0.5f;
	real32 dT3div2 = dT2div2 * dT;
	real32 dT4div4 = dT2div2 * dT2div2;

	t00 = Pzh + dT * Pvh - dT2div2 * Pah;
	t01 = Pzv + dT * Pvv - dT2div2 * Pav;
	t02 = Pza_ + dT * Pva_ - dT2div2 * Paa_;

	t10 = Pvh - dT * Pah;
	t11 = Pvv - dT * Pav;
	t12 = Pva_ - dT * Paa_;

	t20 = Pah;
	t21 = Pav;
	t22 = Paa_;

	Pzh = t00 + dT * t01 - dT2div2 * t02;
	Pzv = t01 - dT * t02;
	Pza_ = t02;

	Pvh = t10 + dT * t11 - dT2div2 * t12;
	Pvv = t11 - dT * t12;
	Pva_ = t12;

	Pah = t20 + dT * t21 - dT2div2 * t22;
	Pav = t21 - dT * t22;
	Paa_ = t22;

	Pzh += dT4div4 * AccUVariance;
	Pzv += dT3div2 * AccUVariance;

	Pvh += dT3div2 * AccUVariance;
	Pvv += dT * dT * AccUVariance;

	Paa_ += AccUBiasVariance;

	// Error
	real32 innov = DensityAlt - KFDensityAltitude;
	real32 sInv = 1.0f / (Pzh + BaroVariance);

	// Kalman gains
	real32 kz = Pzh * sInv;
	real32 kv = Pvh * sInv;
	real32 ka = Pah * sInv;

	// Update state
	KFDensityAltitude += kz * innov;
	KFROC += kv * innov;
	AccUBias += ka * innov;

	// Update state covariance matrix
	Pah -= ka * Pzh;
	Pav -= ka * Pzv;
	Paa_ -= ka * Pza_;

	Pvh -= kv * Pzh;
	Pvv -= kv * Pzv;
	Pva_ -= kv * Pza_;

	Pzh -= kz * Pzh;
	Pzv -= kz * Pzv;
	Pza_ -= kz * Pza_;

} // AltitudeKF

void AltitudeKFChatGBT(real32 DensityAlt, real32 GPSAlt, real32 AccU, real32 dT) { // ChatGBT 3.5 after a LOT of discussion
	static boolean First = true;
	real32 Acceleration;

	if (First) {
		KFDensityAltitude = DensityAlt;
		KFROC = AccUBias = 0.0f;
		First = false;
	}

	// Predict state
	Acceleration = AccU - AccUBias;
	KFROC += Acceleration * dT;
	KFDensityAltitude += KFROC * dT;

	// Predict State Covariance matrix
	real32 t00, t01, t02;
	real32 t10, t11, t12;
	real32 t20, t21, t22;

	real32 dT2div2 = Sqr(dT) * 0.5f;
	real32 dT3div2 = dT2div2 * dT;
	real32 dT4div4 = dT2div2 * dT2div2;

	t00 = Pzh + dT * Pvh - dT2div2 * Pah;
	t01 = Pzv + dT * Pvv - dT2div2 * Pav;
	t02 = Pza_ + dT * Pva_ - dT2div2 * Paa_;

	t10 = Pvh - dT * Pah;
	t11 = Pvv - dT * Pav;
	t12 = Pva_ - dT * Paa_;

	t20 = Pah;
	t21 = Pav;
	t22 = Paa_;

	Pzh = t00 + dT * t01 - dT2div2 * t02;
	Pzv = t01 - dT * t02;
	Pza_ = t02;

	Pvh = t10 + dT * t11 - dT2div2 * t12;
	Pvv = t11 - dT * t12;
	Pva_ = t12;

	Pah = t20 + dT * t21 - dT2div2 * t22;
	Pav = t21 - dT * t22;
	Paa_ = t22;

	Pzh += dT4div4 * AccUVariance;
	Pzv += dT3div2 * AccUVariance;

	Pvh += dT3div2 * AccUVariance;
	Pvv += dT * dT * AccUVariance;

	Paa_ += AccUBiasVariance;

	// Error
	real32 innovAlt = DensityAlt - KFDensityAltitude;
	real32 innovGPS = GPSAlt - KFDensityAltitude;
	real32 sInvAlt = 1.0f / (Pzh + BaroVariance);

	// Kalman gains
	real32 kzAlt = Pzh * sInvAlt;
	real32 kv = Pvh * sInvAlt;
	real32 ka = Pah * sInvAlt;

	// Update state based on barometric pressure sensor measurements
	KFDensityAltitude += kzAlt * innovAlt;
	KFROC += kv * innovAlt;
	AccUBias += ka * innovAlt;

	// Conditionally include GPS-derived terms even if GPS.Valid is false
	if (F.GPSValid) {
		real32 sInvGPS = 1.0f / (Pzh + GPS.vAcc);
		real32 kzGPS = Pzh * sInvGPS;

		// Update state based on GPS altitude measurements
		KFDensityAltitude += kzGPS * innovGPS;

		// Update state covariance matrix for GPS altitude measurements
		Pzh -= kzGPS * Pzh;
		Pzv -= kzGPS * Pzv;
		Pza_ -= kzGPS * Pza_;
	}

	// Update state covariance matrix for barometric pressure sensor measurements
	Pah -= ka * Pzh;
	Pav -= ka * Pzv;
	Paa_ -= ka * Pza_;

	Pvh -= kv * Pzh;
	Pvv -= kv * Pzv;
	Pva_ -= kv * Pza_;

	Pzh -= kzAlt * Pzh;
	Pzv -= kzAlt * Pzv;
	Pza_ -= kzAlt * Pza_;

}

