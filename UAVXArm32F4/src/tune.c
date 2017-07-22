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
		YawAngleKp, YawRateKp, AltPosKp, AltPosKi, AltVelKp, AltVelKd,
		NavPosKp, NavPosKi, NavVelKp, NavCrossTrackKp };

uint8 CurrTuningSel = NoTuning; // index into map to actual parameter
real32 TuningScale = 1.0f;
boolean Tuning = false;
uint8 TuningParamIndex; // target parameter
uint8 OldUntunedParam;
uint8 NewParameterTuning;

void Tune(void) {

	if (Tuning && (State == InFlight)) {
		NewParameterTuning = (uint8) (TuningScale * (real32) OldUntunedParam);
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

//#define INAV_TUNE
#if defined(INAV_TUNE)

#define AUTOTUNE_FIXED_WING

#define AUTOTUNE_FIXED_WING_OVERSHOOT_TIME      100
#define AUTOTUNE_FIXED_WING_UNDERSHOOT_TIME     200
#define AUTOTUNE_FIXED_WING_INTEGRATOR_TC       600
#define AUTOTUNE_FIXED_WING_DECREASE_STEP       8           // 8%
#define AUTOTUNE_FIXED_WING_INCREASE_STEP       5           // 5%
#define AUTOTUNE_FIXED_WING_MIN_FF              10
#define AUTOTUNE_FIXED_WING_MAX_FF              200

/*
 PG_REGISTER_WITH_RESET_TEMPLATE(pidAutotuneConfig_t, pidAutotuneConfig, PG_PID_AUTOTUNE_CONFIG, 0);

 PG_RESET_TEMPLATE(pidAutotuneConfig_t, pidAutotuneConfig,
 .fw_overshoot_time = AUTOTUNE_FIXED_WING_OVERSHOOT_TIME,
 .fw_undershoot_time = AUTOTUNE_FIXED_WING_UNDERSHOOT_TIME,
 .fw_max_rate_threshold = 50,
 .fw_ff_to_p_gain = 10,
 .fw_ff_to_i_time_constant = AUTOTUNE_FIXED_WING_INTEGRATOR_TC,
 );
 */

typedef enum {
	DEMAND_TOO_LOW, DEMAND_UNDERSHOOT, DEMAND_OVERSHOOT,
} pidAutotuneState_e;

typedef struct {
	pidAutotuneState_e state;
	uint32 stateEnterTime;

	boolean pidSaturated;
	real32 gainP;
	real32 gainI;
	real32 gainD;
} pidAutotuneData_t;

#define AUTOTUNE_SAVE_PERIOD        5000        // Save interval is 5 seconds - when we turn off autotune we'll restore values from previous update at most 5 sec ago
#if defined(AUTOTUNE_FIXED_WING) || defined(AUTOTUNE_MULTIROTOR)

static pidAutotuneData_t tuneCurrent[XYZ_AXIS_COUNT];
static pidAutotuneData_t tuneSaved[XYZ_AXIS_COUNT];
static uint32 lastGainsUpdateTime;

void autotuneUpdateGains(pidAutotuneData_t * data) {
	for (int a = 0; a < XYZ_AXIS_COUNT; a++) {
		pidBankMutable()->pid[a].P = lrintf(data[a].gainP);
		pidBankMutable()->pid[a].I = lrintf(data[a].gainI);
		pidBankMutable()->pid[a].D = lrintf(data[a].gainD);
	}
	schedulePidGainsUpdate();
}

void autotuneCheckUpdateGains(void) {

	if (mSClock() > nextGainsUpdateTime) {
		// If pilot will exit autotune we'll restore values we are flying now
		memcpy(tuneSaved, tuneCurrent, sizeof(pidAutotuneData_t)
				* XYZ_AXIS_COUNT);
		autotuneUpdateGains(tuneSaved);
		nextGainsUpdateTime = mSClock() + AUTOTUNE_SAVE_PERIOD;
	}
}

void autotuneStart(void) {
	for (int a = 0; a < XYZ_AXIS_COUNT; a++) {
		tuneCurrent[a].gainP = A[a].O.Kp;
		tuneCurrent[a].gainI = A[a].O.Ki;
		tuneCurrent[a].gainD = A[a].O.Kd;
		tuneCurrent[a].pidSaturated = false;
		tuneCurrent[a].stateEnterTime = mSClock();
		tuneCurrent[a].state = DEMAND_TOO_LOW;
	}

	memcpy(tuneSaved, tuneCurrent, sizeof(pidAutotuneData_t) * XYZ_AXIS_COUNT);
	lastGainsUpdateTime = mSClock();
}

void autotuneUpdateState(void) {
	if (IS_RC_MODE_ACTIVE(BOXAUTOTUNE) && ARMING_FLAG(ARMED)) {
		if (!Tuning) {
			autotuneStart();
			Tuning = true;
		} else
			autotuneCheckUpdateGains();

	} else {
		if (Tuning)
			autotuneUpdateGains(tuneSaved);

		Tuning = false;
	}
}

void autotuneFixedWingUpdate(const flight_dynamics_index_t a,
		real32 desiredRate, real32 reachedRate, real32 pidOutput) {
	const uint32 NowmS = mSClock();
	const real32 absDesiredRate = fabsf(desiredRate);
	real32 maxDesiredRate = currentControlRateProfile->rates[a] * 10.0f;
	pidAutotuneState_e newState;

	// Use different max desired rate in ANGLE for pitch and roll
	// Maximum reasonable error in ANGLE mode is 200% of angle inclination (control doublet),
	// but we are conservative and tune for control singlet.
	if (FLIGHT_MODE(ANGLE_MODE) && (a == Pitch || a == Roll)) {
		real32 maxDesiredRateInAngleMode = MaxAttitudeAngleRad * A[a].O.Kp
				/ FP_PID_LEVEL_P_MULTIPLIER;
		maxDesiredRate = MIN(maxDesiredRate, maxDesiredRateInAngleMode);
	}

	if (fabsf(pidOutput) >= pidProfile()->pidSumLimit)
		// If we have saturated the pid output by P+FF don't increase the gain
		tuneCurrent[a].pidSaturated = true;

	if (absDesiredRate < (pidAutotuneConfig()->fw_max_rate_threshold * 0.01f)
			* maxDesiredRate)
		// We can make decisions only when we are demanding at least 50% of max configured rate
		newState = DEMAND_TOO_LOW;
	else if (fabsf(reachedRate) > absDesiredRate)
		newState = DEMAND_OVERSHOOT;
	else
		newState = DEMAND_UNDERSHOOT;

	if (newState != tuneCurrent[a].state) {
		const timeDelta_t stateTimemS = NowmS - tuneCurrent[a].stateEnterTime;
		boolean gainsUpdated = false;

		switch (tuneCurrent[a].state) {
		case DEMAND_TOO_LOW:
			break;
		case DEMAND_OVERSHOOT:
			if (stateTimemS >= pidAutotuneConfig()->fw_overshoot_time) {
				tuneCurrent[a].gainD = tuneCurrent[a].gainD * (100
						- AUTOTUNE_FIXED_WING_DECREASE_STEP) * 0.01f;

				if (tuneCurrent[a].gainD < AUTOTUNE_FIXED_WING_MIN_FF)
					tuneCurrent[a].gainD = AUTOTUNE_FIXED_WING_MIN_FF;

				gainsUpdated = true;
			}
			break;
		case DEMAND_UNDERSHOOT:
			if (stateTimemS >= pidAutotuneConfig()->fw_undershoot_time
					&& !tuneCurrent[a].pidSaturated) {
				tuneCurrent[a].gainD = tuneCurrent[a].gainD * (100
						+ AUTOTUNE_FIXED_WING_INCREASE_STEP) * 0.01f;
				if (tuneCurrent[a].gainD > AUTOTUNE_FIXED_WING_MAX_FF)
					tuneCurrent[a].gainD = AUTOTUNE_FIXED_WING_MAX_FF;
				gainsUpdated = true;
			}
			break;
		}

		if (gainsUpdated) {
			// Set P-gain to 10% of FF gain (quite agressive - FIXME)
			tuneCurrent[a].gainP = tuneCurrent[a].gainD
					* (pidAutotuneConfig()->fw_ff_to_p_gain * 0.01f);

			// Set integrator gain to reach the same response as FF gain in 0.667 second
			tuneCurrent[a].gainI = (tuneCurrent[a].gainD
					/ FP_PID_RATE_FF_MULTIPLIER) * (1000.0f
					/ pidAutotuneConfig()->fw_ff_to_i_time_constant)
					* FP_PID_RATE_I_MULTIPLIER;
			tuneCurrent[a].gainI
					= constrainf(tuneCurrent[a].gainI, 2.0f, 50.0f);
			autotuneUpdateGains(tuneCurrent);

		}
	}

	// Change state and reset saturation flag
	tuneCurrent[a].state = newState;
	tuneCurrent[a].stateEnterTime = NowmS;
	tuneCurrent[a].pidSaturated = false;
}
}

#endif
#endif



