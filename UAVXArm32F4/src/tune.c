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

real32 TuningScale = 1.0f;
boolean Tuning = false;

void Tune(void) {

	if (Tuning && (State == InFlight)) {

	}

} // Tune

void InitTune(void) {

	Tuning = false;

	if (Tuning) {

	}

} // InitTune

//#define INAV_TUNE
#if defined(INAV_TUNE)

// Note: This tunes inner RATE loops only

#define AUTOTUNE_FIXED_WING

#define XYZ_AXIS_COUNT 3
#define timeMs_t uint32

#define AUTOTUNE_FIXED_WING_OVERSHOOT_TIME      100
#define AUTOTUNE_FIXED_WING_UNDERSHOOT_TIME     200
#define AUTOTUNE_FIXED_WING_INTEGRATOR_TC       600
#define AUTOTUNE_FIXED_WING_DECREASE_STEP       8           // 8%
#define AUTOTUNE_FIXED_WING_INCREASE_STEP       5           // 5%
#define AUTOTUNE_FIXED_WING_MIN_FF              10
#define AUTOTUNE_FIXED_WING_MAX_FF              200

//PG_REGISTER_WITH_RESET_TEMPLATE(pidAutotuneConfig_t, pidAutotuneConfig, PG_PID_AUTOTUNE_CONFIG, 0);

//PG_RESET_TEMPLATE(pidAutotuneConfig_t, pidAutotuneConfig,
//		.fw_overshoot_time = AUTOTUNE_FIXED_WING_OVERSHOOT_TIME,
//		.fw_undershoot_time = AUTOTUNE_FIXED_WING_UNDERSHOOT_TIME,
//		.fw_max_rate_threshold = 50,
//		.fw_ff_to_p_gain = 10,
//		.fw_ff_to_i_time_constant = AUTOTUNE_FIXED_WING_INTEGRATOR_TC,
//);

typedef enum {
	DEMAND_TOO_LOW, DEMAND_UNDERSHOOT, DEMAND_OVERSHOOT,
} pidAutotuneState_e;

typedef struct {
	pidAutotuneState_e state;
	timeMs_t stateEnterTime;

	boolean pidSaturated;
	real32 gainP;
	real32 gainI;
	real32 gainD;
} pidAutotuneData_t;

#define AUTOTUNE_SAVE_PERIOD        5000        // Save interval is 5 seconds - when we turn off autotune we'll restore values from previous update at most 5 sec ago


static pidAutotuneData_t tuneCurrent[XYZ_AXIS_COUNT];
static pidAutotuneData_t tuneSaved[XYZ_AXIS_COUNT];
static timeMs_t lastGainsUpdateTime;

void autotuneUpdateGains(pidAutotuneData_t * data) {
	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		pidBankMutable()->pid[axis].P = lrintf(data[axis].gainP);
		pidBankMutable()->pid[axis].I = lrintf(data[axis].gainI);
		pidBankMutable()->pid[axis].D = lrintf(data[axis].gainD);
	}
	schedulePidGainsUpdate();
}

void autotuneCheckUpdateGains(void) {
	const timeMs_t currentTimeMs = mSClock();

	if ((currentTimeMs - lastGainsUpdateTime) >= AUTOTUNE_SAVE_PERIOD) {
		// If pilot will exit autotune we'll restore values we are flying now
		memcpy(tuneSaved, tuneCurrent, sizeof(pidAutotuneData_t)
				* XYZ_AXIS_COUNT);
		autotuneUpdateGains(tuneSaved);
		lastGainsUpdateTime = currentTimeMs;
	}
}

void autotuneStart(void) {
	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		tuneCurrent[axis].gainP = pidBank()->pid[axis].P;
		tuneCurrent[axis].gainI = pidBank()->pid[axis].I;
		tuneCurrent[axis].gainD = pidBank()->pid[axis].D;
		tuneCurrent[axis].pidSaturated = false;
		tuneCurrent[axis].stateEnterTime = mSClock();
		tuneCurrent[axis].state = DEMAND_TOO_LOW;
	}

	memcpy(tuneSaved, tuneCurrent, sizeof(pidAutotuneData_t) * XYZ_AXIS_COUNT);
	lastGainsUpdateTime = mSClock();
}

void autotuneUpdateState(void) {
	if (IS_RC_MODE_ACTIVE(BOXAUTOTUNE) && ARMING_FLAG(ARMED)) {
		if (!FLIGHT_MODE(AUTO_TUNE)) {
			autotuneStart();
			ENABLE_FLIGHT_MODE(AUTO_TUNE);
		} else
			autotuneCheckUpdateGains();
	} else {
		if (FLIGHT_MODE(AUTO_TUNE))
			autotuneUpdateGains(tuneSaved);

		DISABLE_FLIGHT_MODE(AUTO_TUNE);
	}
}

void autotuneFixedWingUpdate(const flight_dynamics_index_t axis,
		real32 desiredRateDps, real32 reachedRateDps, real32 pidOutput) {
	const timeMs_t currentTimeMs = mSClock();
	const real32 absDesiredRateDps = fabsf(desiredRateDps);
	real32 maxDesiredRate = currentControlRateProfile->rates[axis] * 10.0f;
	pidAutotuneState_e newState;

	// Use different max desired rate in ANGLE for pitch and roll
	// Maximum reasonable error in ANGLE mode is 200% of angle inclination (control dublet), but we are conservative and tune for control singlet.
	if (FLIGHT_MODE(ANGLE_MODE) && (axis == FD_PITCH || axis == FD_ROLL)) {
		real32 maxDesiredRateInAngleMode = DECIDEGREES_TO_DEGREES(
				pidProfile()->max_angle_inclination[axis] * 1.0f)
				* pidBank()->pid[PID_LEVEL].P / FP_PID_LEVEL_P_MULTIPLIER;
		maxDesiredRate = MIN(maxDesiredRate, maxDesiredRateInAngleMode);
	}

	if (fabsf(pidOutput) >= pidProfile()->pidSumLimit) {
		// If we have saturated the pid output by P+FF don't increase the gain
		tuneCurrent[axis].pidSaturated = true;
	}

	if (absDesiredRateDps < (pidAutotuneConfig()->fw_max_rate_threshold
			/ 100.0f) * maxDesiredRate)
		// We can make decisions only when we are demanding at least 50% of max configured rate
		newState = DEMAND_TOO_LOW;

	else if (fabsf(reachedRateDps) > absDesiredRateDps)
		newState = DEMAND_OVERSHOOT;
	else
		newState = DEMAND_UNDERSHOOT;

	if (newState != tuneCurrent[axis].state) {
		const timeDelta_t stateTimeMs = currentTimeMs
				- tuneCurrent[axis].stateEnterTime;
		boolean gainsUpdated = false;

		switch (tuneCurrent[axis].state) {
		case DEMAND_TOO_LOW:
			break;
		case DEMAND_OVERSHOOT:
			if (stateTimeMs >= pidAutotuneConfig()->fw_overshoot_time) {
				tuneCurrent[axis].gainD = tuneCurrent[axis].gainD * (100
						- AUTOTUNE_FIXED_WING_DECREASE_STEP) / 100.0f;
				if (tuneCurrent[axis].gainD < AUTOTUNE_FIXED_WING_MIN_FF)
					tuneCurrent[axis].gainD = AUTOTUNE_FIXED_WING_MIN_FF;
				gainsUpdated = true;
			}
			break;
		case DEMAND_UNDERSHOOT:
			if (stateTimeMs >= pidAutotuneConfig()->fw_undershoot_time
					&& !tuneCurrent[axis].pidSaturated) {
				tuneCurrent[axis].gainD = tuneCurrent[axis].gainD * (100
						+ AUTOTUNE_FIXED_WING_INCREASE_STEP) / 100.0f;
				if (tuneCurrent[axis].gainD > AUTOTUNE_FIXED_WING_MAX_FF)
					tuneCurrent[axis].gainD = AUTOTUNE_FIXED_WING_MAX_FF;
				gainsUpdated = true;
			}
			break;
		}

		if (gainsUpdated) {
			// Set P-gain to 10% of FF gain (quite agressive - FIXME)
			tuneCurrent[axis].gainP = tuneCurrent[axis].gainD
					* (pidAutotuneConfig()->fw_ff_to_p_gain / 100.0f);

			// Set integrator gain to reach the same response as FF gain in 0.667 second
			tuneCurrent[axis].gainI = (tuneCurrent[axis].gainD
					/ FP_PID_RATE_FF_MULTIPLIER) * (1000.0f
					/ pidAutotuneConfig()->fw_ff_to_i_time_constant)
					* FP_PID_RATE_I_MULTIPLIER;
			tuneCurrent[axis].gainI = constrainf(tuneCurrent[axis].gainI, 2.0f,
					50.0f);
			autotuneUpdateGains(tuneCurrent);

			switch (axis) {
			case FD_ROLL:
				blackboxLogAutotuneEvent(ADJUSTMENT_ROLL_D,
						tuneCurrent[axis].gainD);
				break;

			case FD_PITCH:
				blackboxLogAutotuneEvent(ADJUSTMENT_PITCH_D,
						tuneCurrent[axis].gainD);
				break;

			case FD_YAW:
				blackboxLogAutotuneEvent(ADJUSTMENT_YAW_D,
						tuneCurrent[axis].gainD);
				break;
			}
		}

		// Change state and reset saturation flag
		tuneCurrent[axis].state = newState;
		tuneCurrent[axis].stateEnterTime = currentTimeMs;
		tuneCurrent[axis].pidSaturated = false;
	}
}

#endif



