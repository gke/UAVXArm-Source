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


#ifndef _emulation_h
#define _emulation_h

#define BARO_NOISE 0.25f
#define ACC_NOISE 5.0f

#define FAKE_GPS_DT_MS	 100

#define EM_THR_CRUISE_FW_STICK 0.3f
#define EM_THR_CRUISE_STICK 0.5f

#define EM_MASS  1.2f // Kg
#define EM_ARM_LEN 0.3f // M
#define EM_MASS_R (1.0f/EM_MASS)
#define EM_MAX_THRUST ((EM_MASS/EM_THR_CRUISE_STICK)*GRAVITY_MPS_S)
#define EM_MAX_YAW_THRUST (EM_MAX_THRUST*0.015f) // TODO: tweak!
#define EM_CRUISE_FW_MPS ((AS_MIN_MPS+AS_MAX_MPS)*0.5f)
#define EM_CRUISE_MPS 5.0f

#define EM_DRAG_SCALE (EM_MAX_THRUST/(AS_MAX_MPS*AS_MAX_MPS)) // 0.004f?
#define EM_MASS_FW 2.0f //Kg
#define EM_MAX_THRUST_FW ((EM_MASS_FW/EM_THR_CRUISE_FW_STICK)*GRAVITY_MPS_S)

#define FW_MASS  1.0f // Kg
#define FW_ARM_LEN 0.5f // M
#define FW_MASS_R (1.0f/FW_MASS)

void InitEmulation(void);
void DoEmulation(void);
void GPSEmulation(void);

extern uint32 BINGO;
extern real32 FakeAltitude, FakeROC, FakeAccZ;

#endif
