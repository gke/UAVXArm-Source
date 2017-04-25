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

const uint8 TuneMap[] = { NoTuning, RollAngleKp, RollAngleKi, RollRateKp,
		RollRateKd, PitchAngleKp, PitchAngleKi, PitchRateKp, PitchRateKd,
		YawAngleKp, YawRateKp, AltPosKp, AltPosKi, AltVelKp, AltVelKd, NavPosKp,
		NavPosKi, NavVelKp, NavCrossTrackKp};

uint8 CurrTuningSel = NoTuning; // index into map to actual parameter
real32 TuningScale = 1.0f;
boolean Tuning = false;
uint8 TuningParamIndex; // target parameter
uint8 OldUntunedParam;
uint8 NewParameterTuning;


void Tune(void) {

	if (Tuning && (State == InFlight)) {
		NewParameterTuning = (uint8)(TuningScale * (real32)OldUntunedParam);
		SetP(TuningParamIndex, NewParameterTuning);
		RegeneratePIDCoeffs(); // Hellish expensive 4uS :)
	}

} // Tune

void InitTune(void) {

	Tuning = CurrTuningSel != NoTuning;

	if (Tuning) {
		TuningParamIndex = TuneMap[CurrTuningSel];
		NewParameterTuning = OldUntunedParam = P(TuningParamIndex);
	}

} // InitTune


