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


#ifndef _altfilt_h
#define _altfilt_h

void AltitudeSimpleLPF(real32 Alt, real32 dT);
void AltitudeKF(real32 DensityAlt, real32 GPSAlt, real32 AccU, real32 dT);
void AltitudeKFChatGBT(real32 DensityAlt, real32 GPSAlt, real32 AccU, real32 dT);

extern real32 KFROC, KFDensityAltitude, BaroVariance, GPSVariance, AccUVariance, AccUBias, AccUBiasVariance, TrackBaroVariance, TrackAccUVariance;

#endif

