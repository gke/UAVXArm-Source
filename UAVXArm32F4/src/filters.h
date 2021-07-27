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


#ifndef _filters_h
#define _filters_h

#define MAX_FILTER_ORDER 10 //6
typedef struct {
	boolean initialised;
	uint8 order;
	real32 tau;
	uint8 head, tail;
	real32 sum;
	real32 s;
	real32 h[MAX_FILTER_ORDER]; // for rate of change use
	real32 c[MAX_FILTER_ORDER]; //zzz

	real32 q, r, p, x, xP, k;

} filterStruct;

typedef struct {
	boolean initialised;
	uint8 head;
	real32 h[3]; // for rate of change use
} filterM3Struct;

real32 SensorNoise(real32 N);

real32 CalculateVariance(real32 *x, uint16 n);

void DFTn(real32 *DFT, real32 * v);

real32 kth_smallest(real32 a[], const uint16 n, const uint16 k);
#define median(a,n) kth_smallest(a,n,(((n)&1)?((n)/2):(((n)/2)-1)))

real32 real32Median3Filter(filterM3Struct * F, real32 r);
real32 int32Median3Filter(boolean * Primed, int32 v[]);
real32 uint32Median3Filter(boolean * Primed, uint32 v[]);

real32 PavelDifferentiator(filterStruct *F, real32 v);

void initFIRF(filterStruct *F, const idx order, const real32 * C);
real32 FIRF(filterStruct *F, real32 v);
void initMAF(filterStruct * F, const idx order);
real32 MAF(filterStruct *F, real32 v);

real32 Smoothr32xn(filterStruct * F, real32 v);

real32 initLPF1Coefficient(real32 CutHz, real32 dT);
real32 LPF1(real32 O, real32 N, const real32 K);

void initLPFn(filterStruct * F, const idx order, const real32 CutHz);
void setLPFn(filterStruct * F, real32 v);
real32 LPFn(filterStruct * F, real32 v, real32 dT);

typedef struct ABGF_s {
    real32 a, b, g;
    real32 ak_1, vk_1, xk_1;
   // real32 dT, dT2;
} filterStructABG;

typedef enum {
    CRITICAL_DAMPED = 0,
    UNDER_DAMPED,
} eABGF;

extern const real32 ABGAlpha;
extern const uint8 ABGType;

void initABGLPF(filterStructABG *F, real32 alpha, eABGF ftype);
real32 ABGLPF(filterStructABG *F, real32 input, real32 dT);

const real32 ABGalpha;
const uint8 ABGType;

real32 gen_random(int32 seed, real32 max);

real32 Threshold(real32 v, real32 t);
real32 DeadZone(real32 v, real32 t);
real32 SlewLimit(real32 Old, real32 New, const real32 Rate, real32 dT);

real32 Make2Pi(real32);
real32 MakePi(real32);
real32 invSqrt(real32 x);
void Rotate(real32 * nx, real32 * ny, real32 x, real32 y, real32 A);
void RotateSensor(int16 * x, int16 * y, const uint8 Q);
real32 DecayX(real32 v, real32 d, real32 dT);
real32 scaleRangef(real32 v, real32 srcMin, real32 srcMax, real32 destMin,
		real32 destMax);

//__________________________________________________________________________


extern real32 CurrAccLPFHz, CurrGyroLPFHz, CurrYawLPFHz, CurrServoLPFHz;

#endif

