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

void TrackerReset(TrackerStruct * Tracker) {
	Tracker->lastUpdatedmS = mSClock();
	Tracker->total = 0;
	Tracker->count = 0;
	Tracker->value = 0;
} // TrackerReset

void TrackerAccumulate(TrackerStruct * Tracker, int16 rawValue) {
	const timemS NowmS = mSClock();

	if (((NowmS - Tracker->lastUpdatedmS) > TRACKER_INTERVAL_MS)
			&& Tracker->count) {
		TrackerSet(Tracker, Tracker->total / Tracker->count);
		Tracker->total = 0;
		Tracker->count = 0;
	}

	Tracker->total += rawValue;
	Tracker->count++;
} // TrackerAccumulate

void TrackerSet(TrackerStruct * Tracker, int16 rawValue) {
	Tracker->value = rawValue;
	Tracker->lastUpdatedmS = mSClock();
} // TrackerSet

int16 TrackerGet(TrackerStruct * Tracker) {
	if ((mSClock() - Tracker->lastUpdatedmS) > TRACKER_TIMEOUT_MS)
		Tracker->value = 0;

	return Tracker->value;
} // TrackerGet

real32 SensorNoise(real32 N) {

	return (N * (((real64) rand() / (real64) RAND_MAX) - 0.5f));

} // SensorNoise

real32 CalculateVariance(real32 *x, uint16 n) {

	uint16 i;
	real32 average, variance, std_deviation, sum = 0.0f, sum1 = 0.0f;

	for (i = 0; i < n; i++)
		sum = sum + x[i];
	average = sum / (real32) n;

	for (i = 0; i < n; i++)
		sum1 = sum1 + pow((x[i] - average), 2);

	variance = sum1 / (real32) n;
	std_deviation = sqrt(variance);

	return variance;

} // CalculateVariance

void DFTn(real32 *DFT, real32 * v) {
#define DFT_WINDOW_SIZE 8
	const real32 mR = 1.0f / (real32) DFT_WINDOW_SIZE;
	static boolean Primed = false;
	static real32 cosarg[DFT_WINDOW_SIZE][DFT_WINDOW_SIZE],
			sinarg[DFT_WINDOW_SIZE][DFT_WINDOW_SIZE];
	static real32 inp[DFT_WINDOW_SIZE];
	long i, k;
	double arg;
	double x[DFT_WINDOW_SIZE], y[DFT_WINDOW_SIZE];

	if (!Primed) {
		for (i = 0; i < DFT_WINDOW_SIZE; i++) {
			arg = -2.0f * PI * (real32) i * mR;
			for (k = 0; k < DFT_WINDOW_SIZE; k++) {
				cosarg[i][k] = cosf(arg * (real32) k);
				sinarg[i][k] = sinf(arg * (real32) k);
			}
		}
		Primed = true;
	}

	for (i = 0; i < DFT_WINDOW_SIZE; i++) {
		x[i] = y[i] = 0.0;
		for (k = 0; k < DFT_WINDOW_SIZE; k++) {
			x[i] += v[k] * cosarg[i][k];
			y[i] += v[k] * sinarg[i][k];
		}
	}

	for (i = 0; i < DFT_WINDOW_SIZE; i++)
		DFT[i] = sqrtf(Sqr(x[i]) + Sqr(y[i])) * mR;

} // DFT

void LMSQFit(real32 * M, real32 * C, real32 X[], real32 Y[], uint16 Samples) {
	uint16 i;
	real32 xMean, yMean, n, d;
	real32 SamplesR;

	SamplesR = 1.0f / Samples;
	xMean = yMean = 0.0f;

	for (i = 0; i < Samples; i++) {
		xMean += X[i];
		yMean += Y[i];
	}

	xMean *= SamplesR;
	yMean *= SamplesR;

	n = d = 0.0f;
	for (i = 0; i < Samples; i++) {
		n += (X[i] - xMean) * (Y[i] - yMean);
		d += Sqr(X[i] - xMean);
	}

	*M = n / d;
	*C = yMean - *M * xMean;

} // LMSQFit

// Algorithm from N. Wirth's book, implementation by N. Devillard.
// This code in public domain.

typedef real32 elem_type;

#define ELEM_SWAP(a,b) { register elem_type t=(a);(a)=(b);(b)=t; }

/*---------------------------------------------------------------------------
 Function :   kth_smallest()
 In       :   array of elements, # of elements in the array, rank k
 Out      :   one element
 Job      :   find the kth smallest element in the array
 Notice   :   use the median() macro defined below to get the median.

 Reference:

 Author: Wirth, Niklaus
 Title: Algorithms + data structures = programs
 Publisher: Englewood Cliffs: Prentice-Hall, 1976
 Physical description: 366 p.
 Series: Prentice-Hall Series in Automatic Computation

 ---------------------------------------------------------------------------*/

real32 kth_smallest(real32 a[], const uint16 n, const uint16 k) {
	uint16 i, j, l, m;
	real32 x;

	l = 0;
	m = n - 1;
	while (l < m) {
		x = a[k];
		i = l;
		j = m;
		do {
			while (a[i] < x)
				i++;
			while (x < a[j])
				j--;
			if (i <= j) {
				ELEM_SWAP(a[i], a[j]);
				i++;
				j--;
			}
		} while (i <= j);
		if (j < k)
			l = i;
		if (k < i)
			m = j;
	}
	return a[k];
} // kth_smallest

real32 real32Median3Filter(filterM3Struct * F, real32 r) {
	// https://embeddedgurus.com/stack-overflow/2010/10/median-filtering/
	idx i;
	real32 middle;

	if (!F->initialised) {
		F->initialised = true;
		for (i = 0; i < 3; i++)
			F->h[i] = r;
		middle = F->h[0];
		F->head = 0;
	} else {

		F->h[++F->head] = r;
		if (F->head >= 3)
			F->head = 0;

		if ((F->h[0] <= F->h[1]) && (F->h[0] <= F->h[2]))
			middle = (F->h[1] <= F->h[2]) ? F->h[1] : F->h[2];
		else if ((F->h[1] <= F->h[0]) && (F->h[1] <= F->h[2]))
			middle = (F->h[0] <= F->h[2]) ? F->h[0] : F->h[2];
		else
			middle = (F->h[0] <= F->h[1]) ? F->h[0] : F->h[1];

	}
	return middle;
} // real32Median3Filter

real32 LeadFilter(real32 Pos, real32 VelP, real32 Vel, real32 Lag) {

	return (Pos + Vel * Lag + (Vel - VelP) * Sqr(Lag));
} // LeadFilter

void initMAF(filterStruct * F, const idx order) {
	idx i;

	F->order = (order > MAX_FILTER_ORDER) ? MAX_FILTER_ORDER : order;
	for (i = 0; i < F->order; i++)
		F->h[i] = 0.0f;

	F->s = 0.0f;
	F->head = 0;

	F->initialised = false;

} // initMA

real32 MAF(filterStruct * F, real32 v) {
	idx i, p;

	if (F->order <= 1)
		return (v);
	else {
		if (!F->initialised) {

			for (i = 0; i < F->order; i++)
				F->h[i] = v;

			F->s = v * F->order;
			F->head = 0;
			F->initialised = true;
		} else {

			F->s -= F->h[F->head];
			F->h[F->head] = v;
			F->s += v;
			if (++F->head >= F->order)
				F->head = 0;
		}
		return (F->s / (real32) F->order);
	}
} // MAF

void initFIRF(filterStruct *F, const idx order, const real32 * C) {
	// Pavel Holoborodko, see http://www.holoborodko.com/pavel/numerical-methods/
	// numerical-derivative/smooth-low-noise-differentiators/

	idx i;

	F->initialised = false;
	F->order = (order > MAX_FILTER_ORDER) ? MAX_FILTER_ORDER : order;

	for (i = 0; i < F->order; i++)
		F->c[i] = C[i];

} // initFIRF

real32 FIRF(filterStruct *F, real32 v) {

	idx i;

	if (!F->initialised) {

		for (i = 0; i < F->order; i++)
			F->h[i] = v;

		F->initialised = true;
	}

	// inefficient but OK for short vectors
	for (i = F->order; i > 0; --i)
		F->h[i] = F->h[i - 1];
	F->h[0] = v;

	real32 r = 0.0f;
	for (i = 0; i < F->order; ++i)
		r += F->c[i] * F->h[i];

	return (r);
} // FIRF

real32 initLPF1Coefficient(real32 CutHz, real32 dT) {

	return (dT / ((1.0f / (TWO_PI * CutHz)) + dT));
} // initLPF1Coefficient

real32 LPF1(real32 O, real32 N, const real32 K) {

	return (O + (N - O) * K);
} // LPF1

void initLPFn(filterStruct * F, const idx order, const real32 CutHz) {
	idx n;

	// AdjCutHz = CutHz /(sqrtf(powf(2, 1/order) -1))
	const real32 ScaleF[] = { 1.0f, 1.553773974f, 1.961459177f, 2.298959223f };

	F->order = Limit(order, 1, 4); //MAX_LPF_ORDER);

	for (n = 1; n <= F->order; n++)
		F->h[n] = 0.0f;

	F->tau = 1.0f / (TWO_PI * CutHz * ScaleF[F->order - 1]);

	F->initialised = false;

} // initLPFn

void setLPFn(filterStruct * F, real32 v) {
	idx n;

	for (n = 0; n <= F->order; n++)
		F->h[n] = v;
	F->initialised = true;

} // setLPFn

real32 LPFn(filterStruct * F, real32 v, real32 dT) {
	idx n;

	if (!F->initialised) {
		for (n = 1; n <= F->order; n++)
			F->h[n] = v;
		F->initialised = true;
	}

	real32 s = dT / (F->tau + dT); // more compute but deals with sampling jitter

	F->h[0] = v;

	for (n = 1; n <= F->order; n++)
		F->h[n] += (F->h[n - 1] - F->h[n]) * s;

	return (F->h[F->order]);

} // LPFn


//________________________________________________________________________________________

// Robert Bouwens

// www.megamanual.com/alphabeta.htm
// Alpha, Beta and Gamma can be considered to proportional, first and second derivative terms respectively
// Alpha 0.5..1.5 (0.9) Beta < 1.0 (0.8)  Gamma < 0.5 (0.1)

const real32 ABGalpha = 0.5f;
const uint8 ABGType = UNDER_DAMPED;

void initABGLPF(filterStructABG *F, real32 alpha, eABGF ftype) {

	F->xk_1 = 0.0f;
	F->vk_1 = 0.0f;
	F->ak_1 = 0.0f;

	if (ftype == CRITICAL_DAMPED) {
		// near critically damped F
		const real32 beta = 0.8f
				* (2.0f - Sqr(alpha) - 2.0f * sqrtf(1.0f - Sqr(alpha)))
				/ (Sqr(alpha));
		F->a = alpha;
		F->b = beta;
		F->g = Sqr(beta) / (alpha * 2.0f);
	} else {
		const real32 beta = Sqr(alpha) / (2.0f - alpha); //  standard, under damped beta value
		F->a = alpha;
		F->b = beta;
		F->g = Sqr(beta) / (alpha * 2.0f);
	}

} // ABGInit

real32 ABGLPF(filterStructABG *F, real32 input, real32 dT) {
	real32 x0 = F->xk_1;
	const real32 dT2 = Sqr(dT);

	// update our (estimated) state 'x' from the system (ie pos = pos + vel (last).dT)
	F->xk_1 += dT * F->vk_1 + (0.5f * dT2 * F->ak_1);
	// update (estimated) velocity
	F->vk_1 += dT * F->ak_1;
	// what is our residual error (measured - estimated)
	const real32 rk = input - F->xk_1;
	// update our estimates given the residual error.
	F->xk_1 += F->a * rk;                 // prediction
	F->xk_1 += F->a * (x0 - F->xk_1);     // correction

	real32 v0 = F->vk_1;
	F->vk_1 += (F->b / dT) * rk;        // prediction
	F->vk_1 += F->b * (v0 - F->vk_1);   // correction

	if (F->g != 0.0f) {
		F->ak_1 += (F->g / (2.0f * dT2)) * rk;   // prediction
		F->ak_1 += (F->g / dT) * (v0 - F->vk_1); // correction
	}

	return F->xk_1;

} // ABG

//--------------------------------------------------

// One Euro Filter, C version
//  Jonathan Aceituno <join@oin.name>

// For details, see http://www.lifl.fr/~casiez/1euro

//The state data for a simple low pass filter.
typedef struct {
	real32 hatxprev;
	real32 xprev;
	boolean usedBefore;
} SFLowPassFilter;

//The configuration for an instance of a 1 Euro Filter.
typedef struct {
	real32 rate;
	real32 minCutoffFrequency;
	real32 cutoffSlope;
	real32 derivativeCutoffFrequency;
} SF1eFilterConfiguration;

/**
 A 1 Euro Filter with configuration and state data.

 Example 1: Heap allocation, using a configuration struct, and a fixed frequency.
 SF1eFilterConfiguration config;
 config.frequency = 120;
 config.cutoffSlope = 1;
 config.derivativeCutoffFrequency = 1;
 config.minCutoffFrequency = 1;
 SF1eFilter *F = SF1eFilterCreateWithConfig(config);
 // ...
 real32 filtered = SF1eFilterDo(filter, x);
 // ...
 filter = SF1eFilterDestroy(filter);

 Example 2: Heap allocation, using a variable frequency, without configuration struct.
 SF1eFilter *F = SF1eFilterCreate(120, 1, 1, 1);
 // ...
 real32 time = // ... (get current timestamp)
 real32 filtered = SF1eFilterDoAtTime(filter, x, time);
 // ...
 filter = SF1eFilterDestroy(filter);

 Example 3: Stack allocation.
 SF1eFilter filter;
 filter.config.frequency = 120;
 filter.config.cutoffSlope = 1;
 filter.config.derivativeCutoffFrequency = 1;
 filter.config.minCutoffFrequency = 1;
 SF1eFilterInit(&filter);
 // ...
 real32 filtered = SF1eFilterDo(filter, x);
 // ...
 // (filter goes away with stack)
 */
typedef struct {
	SF1eFilterConfiguration config;
	SFLowPassFilter xfilt;
	SFLowPassFilter dxfilt;
	real32 alpha;
	real32 fc;
	real32 fmax;
} SF1eFilter;

//Allocate and initialize a new 1 Euro Filter with the given properties.
SF1eFilter *SF1eFilterCreate(real32 frequency, real32 minCutoffFrequency,
		real32 cutoffSlope, real32 derivativeCutoffFrequency);
//Allocate and initialize a new 1 Euro Filter with the given configuration.
SF1eFilter *SF1eFilterCreateWithConfig(SF1eFilterConfiguration config);
//Initialize a 1 Euro Filter instance. Make sure to do this on a stack-allocated One Euro Filter before using it.
void SF1eFilterInit(SF1eFilter *F);
//Filter a real32 using the given One Euro Filter.
real32 SF1eFilterDo(SF1eFilter *F, real32 x);
//Filter a real32 using the given One Euro Filter and the given timestamp. Frequency will be automatically recomputed.
real32 SF1eFilterDoAtTime(SF1eFilter *F, real32 x, double timestamp);
//Compute Alpha for a given One Euro Filter and a given cutoff frequency.
real32 SF1eFilterAlpha(real32 rate, real32 cutoff);
//Create a simple low-pass filter instance.
SFLowPassFilter *SFLowPassFilterCreate();
//Initialize a low-pass filter instance. Make sure to do this on a stack-allocated low-pass filter instance before using it.
void SFLowPassFilterInit(SFLowPassFilter *F);
//Filter a real32 using the given low-pass filter and the given alpha value.
real32 SFLowPassFilterDo(SFLowPassFilter *F, real32 x, real32 alpha);

void SFLowPassFilterInit(SFLowPassFilter *F) {
	F->usedBefore = false;
	F->hatxprev = 0.0f;
	F->xprev = 0.0f;
}

real32 SFLowPassFilterDo(SFLowPassFilter *F, real32 x, real32 alpha) {
	if (!F->usedBefore) {
		F->usedBefore = true;
		F->hatxprev = x;
	}

	const real32 hatx = alpha * x + (1.0f - alpha) * F->hatxprev;
	F->xprev = x;
	F->hatxprev = hatx;
	return hatx;
}

void SF1eFilterInit(SF1eFilter *F) {
//	F->rate = F->config.rate;
//	F->lastTime = 0;
	SFLowPassFilterInit(&(F->xfilt));
	SFLowPassFilterInit(&(F->dxfilt));

	F->alpha = SF1eFilterAlpha(F->config.rate,
			F->config.derivativeCutoffFrequency);
}

real32 SF1eFilterDo(SF1eFilter *F, real32 x) {
	real32 dx = 0.0f;

	if (F->xfilt.usedBefore)
		dx = (x - F->xfilt.xprev) * F->config.rate;

	const real32 edx = SFLowPassFilterDo(&(F->dxfilt), dx, F->alpha);
	F->fc = F->config.minCutoffFrequency + F->config.cutoffSlope * fabsf(edx);
	//F->fc = MIN(F->fc, F->fmax);
	return SFLowPassFilterDo(&(F->xfilt), x,
			SF1eFilterAlpha(F->config.rate, F->fc));
}

real32 SF1eFilterAlpha(real32 rate, real32 cutoff) {
	const real32 tau = 1.0f / (2.0f * PI * cutoff);
//	const real32 te  = 1.0f / F->rate;
//
//	return 1.0f / (1.0f + tau / te);
	return 1.0f / (1.0f + tau * rate);
}

//-----------------------------------------------------------------


real32 SlewLimit(real32 O, real32 N, const real32 Rate, real32 dT) {
	real32 Low, High, Slew;

	Slew = Rate * dT;

	Low = O - Slew;
	High = O + Slew;
	O = (N < Low) ? Low : ((N > High) ? High : N);
	return (O);
} // SlewLimit


//============================CHATGPT

#define MAX_CONTROL_SURFACE_ANGLE 60.0 // Maximum control surface angle in degrees
#define MAX_HEADING_CHANGE 180.0       // Maximum heading change in degrees


real32 s1updateValue(real32 oldValue, real32 newValue, real32 rate, real32 dT) {
    // Calculate the absolute difference between the new value and old value
    real32 absDiff = fabs(newValue - oldValue);

    // Calculate the maximum change allowed based on the rate and time step
    real32 maxChange = rate * dT;

    // Calculate the adjusted change based on the sign of newValue - oldValue
    real32 adjustedChange = (newValue > oldValue) ? fmin(absDiff, maxChange) : -fmin(absDiff, maxChange);

    // Update the old value with the adjusted change
    oldValue += adjustedChange;

    return oldValue;
}



#define s3MAX_CONTROL_SURFACE_ANGLE (PI / 3.0) // Maximum control surface angle in radians (60 degrees)
#define s3MAX_HEADING_CHANGE (PI)             // Maximum heading change in radians (180 degrees)


real32 s3wrapToPi(real32 angle) {
    // Wrap the angle to the range [-π, π]
    return fmod(angle + PI, TWO_PI) - PI;
}

real32 s3updateAngle(real32 oldValue, real32 newValue, real32 maxAngle, real32 rate, real32 dT) {
    // Wrap oldValue and newValue to the range [-π, π]
    oldValue = s3wrapToPi(oldValue);
    newValue = s3wrapToPi(newValue);

    // Calculate the angular difference between the new value and old value
    real32 diff = newValue - oldValue;

    // Handle cases where the absolute difference is greater than π
    if (fabs(diff) > PI)
        if (diff > 0)
            diff -= TWO_PI;
       else
            diff += TWO_PI;

    // Calculate the maximum change allowed based on the rate and time step
    real32 maxChange = rate * dT;

    // Apply the maximum change
    diff = fmin(maxChange, fabs(diff)) * (diff < 0 ? -1 : 1);

    // Update the old value with the adjusted difference
    oldValue += diff;

    // Clamp the old value to the range [-maxAngle, maxAngle]
    oldValue = fmax(-maxAngle, fmin(maxAngle, oldValue));

    return oldValue;
}

int s3main() {
    const real32 rate = 30.0; // Desired rate of change in radians per second
    const real32 dT = 1.0;    // Time step in seconds

    real32 oldValue = 0.0; // Initial value in radians
    real32 newValue = -PI / 3.0; // New value in radians

    // For control surfaces
    oldValue = s3updateAngle(oldValue, newValue, MAX_CONTROL_SURFACE_ANGLE, rate, dT);
    printf("Updated control surface angle: %.2f\n", oldValue); // Output: Updated control surface angle: -1.05

    // For heading changes
    oldValue = s3updateAngle(oldValue, newValue, MAX_HEADING_CHANGE, rate, dT);
    printf("Updated heading: %.2f\n", oldValue); // Output: Updated heading: -1.05

    return 0;
}


//=================================================================


real32 Threshold(real32 v, real32 t) {

	if (v > t)
		v -= t;
	else if (v < -t)
		v += t;
	else
		v = 0.0f;

	return (v);
} // Threshold

real32 DeadZone(real32 v, real32 t) {

	if (Abs(v) <= t)
		v = 0.0f;

	return (v);
} // DeadZone

real32 Make2Pi(real32 A) {
	while (A < 0)
		A += DegreesToRadians(360.0f);
	while (A >= DegreesToRadians(360.0f))
		A -= DegreesToRadians(360.0f);

	return (A);
} // Make2Pi

real32 MakePi(real32 A) {
	while (A < -DegreesToRadians(180.0f))
		A += DegreesToRadians(360.0f);
	while (A >= DegreesToRadians(180.0f))
		A -= DegreesToRadians(360.0f);

	return (A);
} // MakePi

real32 invSqrt(real32 x) {

#if defined(STM32F4)
	return (1.0f / sqrtf(x)); // hopefully gcc uses VSQRT for M4
#else
#if defined(ORIGINAL_INVSQRT)
	// Fast inverse square-root
	// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

	real32 halfx = 0.5f * x;
	real32 y = x;
	uint32 i = *(uint32*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(real32*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
#else
	real32 InvSqrt(real32 x) {
		uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
		real32 tmp = *(real32*)&i;
		return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
	}
#endif

#endif
} // invSqrt

void Rotate(real32 * nx, real32 * ny, real32 x, real32 y, real32 A) { // A rotation CW
	static real32 cA = 1.0f;
	static real32 sA = 0.0f;
	static real32 AP = 0.0f;
	real32 Temp;

	if (A != AP) { // saves ~7uS for successive calls with same angle
		cA = cosf(A);
		sA = sinf(A);
		AP = A;
	}

	Temp = x * cA + y * sA;
	*ny = -x * sA + y * cA;
	*nx = Temp;

} // Rotate

void RotateSensor(int16 * x, int16 * y, const uint8 Q) {
	int16 Temp;

	if (Q > 0)
		switch (Q) {
		case 1:
			Temp = *x;
			*x = *y;
			*y = -Temp;
			break;
		case 2:
			*x = -*x;
			*y = -*y;
			break;
		case 3:
			Temp = *x;
			*x = -*y;
			*y = Temp;
			break;
		default:
			break;
		}

} // RotateSensor

real32 DecayX(real32 v, real32 rate, real32 dT) {
	real32 d = rate * dT;

	if (v < -d)
		v += d;
	else if (v > d)
		v -= d;
	else
		v = 0.0f;

	return (v);
}
// DecayX

real32 scaleRangef(real32 v, real32 srcMin, real32 srcMax, real32 destMin,
		real32 destMax) {
	real32 a = (destMax - destMin) * (v - srcMin);
	real32 b = srcMax - srcMin;
	return ((a / b) + destMin);
} // scaleRangef

