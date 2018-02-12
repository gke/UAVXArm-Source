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

void DFT8(real32 v, real32 *DFT);

real32 kth_smallest(real32 a[], uint16 n, uint16 k);
#define median(a,n) kth_smallest(a,n,(((n)&1)?((n)/2):(((n)/2)-1)))

void InitSmoothr32xn(HistStruct * F);
real32 Smoothr32xn(HistStruct * F, uint8 n, real32 v);

real32 LPF1(real32 O, real32 N, const real32 K);
real32 LPF1Coefficient(real32 CutHz, real32 dT);
real32 LPFn(HistStruct * F, const idx Order, real32 v, const real32 CutHz, real32 dT);
real32 LPD5BW(HistStruct * F, real32 v, const real32 CutHz, real32 dT);
real32 PavelDifferentiator(HistStruct *F, real32 v);

typedef struct {
	boolean enable;
	real32 q, r, p, est, estP, k;
} KFStruct;

void InitAccGyroKF(KFStruct *F, real32 Q, real32 R);
real32 DoAccGyroKF(KFStruct *F, real32 v);

real32 Threshold(real32 v, real32 t);
real32 DeadZone(real32 v, real32 t);
real32 SlewLimit(real32 * Old, real32 New, const real32 Rate, real32 dT);
int16 SensorSlewLimit(uint8 sensor, int16 * Old, int16 New, int16 Slew);

real32 Make2Pi(real32);
real32 MakePi(real32);
real32 invSqrt(real32 x);
void Rotate(real32 * nx, real32 * ny, real32 x, real32 y, real32 A);
real32 DecayX(real32 v, real32 d, real32 dT);
real32 scaleRangef(real32 v, real32 srcMin, real32 srcMax, real32 destMin,
		real32 destMax);

#endif


