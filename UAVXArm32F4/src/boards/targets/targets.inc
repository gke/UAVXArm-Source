// DO NOT FORMAT  DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT
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

//    UAVX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.
//    If not, see http://www.gnu.org/licenses/

#ifndef _targets_h
#define _targets_h

#include "UAVX.h"

#if defined(UAVXF4V3)
#include "uavxf4v3.inc"
//
#elif defined(UAVXF4V4)
#include "uavxf4v4.inc"
//
#elif defined(OMNIBUSF4V1)
#include "omnibusf4v1.inc"
#elif defined(OMNIBUSF4NXT)
#include "omnibusf4nxt.inc"
//
#elif defined(DISCOVERYF4)
#include "discoveryf4.inc"
//
#else
// mbed
#endif

#endif




