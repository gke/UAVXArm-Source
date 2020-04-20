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


#ifndef _calibration_h
#define _calibration_h

#define ACC_CAL_CYCLES 400
#define ACC_CAL_SPHERE_CYCLES 150
#define MAG_CAL_CYCLES 400

uint16 SphereFit(real32 d[][3], uint16 N, uint16 MaxIterations,
		real32 Err, real32 SphereOrigin[3],
		real32 *SphereRadius);

int8 getCurrDir(real32 s[3]);

void CalibrateAccSixPoint(uint8 s);
void CalibrateAccSixPointSphere(uint8 s);

#endif

