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
boolean TuningEnabled = false;

void Tune(void) {

	//if (Tuning && (State == InFlight)) {

	//}

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

#define AUTOTUNE_FIXED_WING_OVERSHOOT_TIME      100
#define AUTOTUNE_FIXED_WING_UNDERSHOOT_TIME     200
#define AUTOTUNE_FIXED_WING_INTEGRATOR_TC       600
#define AUTOTUNE_FIXED_WING_DECREASE_STEP       8           // 8%
#define AUTOTUNE_FIXED_WING_INCREASE_STEP       5           // 5%
#define AUTOTUNE_FIXED_WING_MIN_FF              10
#define AUTOTUNE_FIXED_WING_MAX_FF              200

#define FW_UNDERSHOOT_MS	AUTOTUNE_FIXED_WING_OVERSHOOT_TIME

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
	uint32 stateEnterTime;
	boolean pidSaturated;
	PIDStruct R;
} pidAutotuneData_t;

#define AUTOTUNE_SAVE_PERIOD        5000        // Save interval is 5 seconds - when we turn off autotune we'll restore values from previous update at most 5 sec ago
static pidAutotuneData_t tuneCurrent[3];
static pidAutotuneData_t tuneSaved[3];
static uint32 lastGainsUpdateTime;

void autotuneUpdateGains(pidAutotuneData_t * data) {
	for (int a = Pitch; a <= Yaw; a++) {
		A[a].R.Kp = lrintf(data[a].R.Kp);
		A[a].R.Ki = lrintf(data[a].R.Ki);
		A[a].R.Kd = lrintf(data[a].R.Kd);
	}
	//schedulePidGainsUpdate();
}

void autotuneCheckUpdateGains(void) {
	const uint32 NowmS = mSClock();

	if ((NowmS - lastGainsUpdateTime) >= AUTOTUNE_SAVE_PERIOD) {
		// If pilot will exit autotune we'll restore values we are flying now
		memcpy(tuneSaved, tuneCurrent, sizeof(pidAutotuneData_t) * 3);
		autotuneUpdateGains(tuneSaved);
		lastGainsUpdateTime = NowmS;
	}
}

void autotuneStart(void) {
	for (int a = Pitch; a <= Yaw; a++) {
		tuneCurrent[a].R.Kp = A[a].R.Kp;
		tuneCurrent[a].R.Ki = A[a].R.Ki;
		tuneCurrent[a].R.Kd = A[a].R.Kd;
		tuneCurrent[a].pidSaturated = false;
		tuneCurrent[a].stateEnterTime = mSClock();
		tuneCurrent[a].state = DEMAND_TOO_LOW;
	}

	memcpy(tuneSaved, tuneCurrent, sizeof(pidAutotuneData_t) * 3);
	lastGainsUpdateTime = mSClock();
}

void autotuneUpdateState(void) {
	if (TuningEnabled && Armed()) {
		if (!Tuning) {
			autotuneStart();
			Tuning = true;
			//ENABLE_FLIGHT_MODE(AUTO_TUNE);
		} else
			autotuneCheckUpdateGains();
	} else {
		if (TuningEnabled)
			autotuneUpdateGains(tuneSaved);

		Tuning = false;
		//DISABLE_FLIGHT_MODE(AUTO_TUNE);
	}
}

void autotuneUpdate(const idx a, real32 desiredRateDps, real32 reachedRateDps,
		real32 pidOutput) {
	const uint32 NowmS = mSClock();
	const real32 absDesiredRateDps = Abs(desiredRateDps);
	real32 maxDesiredRate = 123; //zzz A[]] * 10.0f;
	pidAutotuneState_e newState;

	// Use different max desired rate in ANGLE for pitch and roll
	// Maximum reasonable error in ANGLE mode is 200% of angle inclination (control dublet), but we are conservative and tune for control singlet.
	if ((AttitudeMode == AngleMode) && (a == Pitch || a == Roll)) {
		real32 maxDesiredRateInAngleMode = A[a].P.Max * A[a].P.Kp;
		maxDesiredRate = Min(maxDesiredRate, maxDesiredRateInAngleMode);
	}

	if (Abs(pidOutput) >= 1.0f)
		// If we have saturated the pid output by P+FF don't increase the gain
		tuneCurrent[a].pidSaturated = true;

	if (absDesiredRateDps <  maxDesiredRate)
		// We can make decisions only when we are demanding at least 50% of max configured rate
		newState = DEMAND_TOO_LOW;

	else
		newState = (Abs(reachedRateDps) > absDesiredRateDps) ? DEMAND_OVERSHOOT
				: DEMAND_UNDERSHOOT;

	if (newState != tuneCurrent[a].state) {
		const uint32 stateTimeMs = NowmS - tuneCurrent[a].stateEnterTime;
		boolean gainsUpdated = false;

		switch (tuneCurrent[a].state) {
		case DEMAND_TOO_LOW:
			break;
		case DEMAND_OVERSHOOT:
			if (stateTimeMs >= 123) { //pidAutotuneConfig()->fw_overshoot_time) {
				tuneCurrent[a].R.Kd = tuneCurrent[a].R.Kd * (100
						- AUTOTUNE_FIXED_WING_DECREASE_STEP) * 0.01f;
				if (tuneCurrent[a].R.Kd < AUTOTUNE_FIXED_WING_MIN_FF)
					tuneCurrent[a].R.Kd = AUTOTUNE_FIXED_WING_MIN_FF;
				gainsUpdated = true;
			}
			break;
		case DEMAND_UNDERSHOOT:
			if (stateTimeMs >= FW_UNDERSHOOT_MS && !tuneCurrent[a].pidSaturated) {
				tuneCurrent[a].R.Kd = tuneCurrent[a].R.Kd * (100
						+ AUTOTUNE_FIXED_WING_INCREASE_STEP) * 0.01f;
				if (tuneCurrent[a].R.Kd > AUTOTUNE_FIXED_WING_MAX_FF)
					tuneCurrent[a].R.Kd = AUTOTUNE_FIXED_WING_MAX_FF;
				gainsUpdated = true;
			}
			break;
		}

		if (gainsUpdated) {
			// Set P-gain to 10% of FF gain (quite agressive - FIXME)
			tuneCurrent[a].R.Kp = tuneCurrent[a].R.Kd * 0.1f;

			// Set integrator gain to reach the same response as FF gain in 0.667 second
			tuneCurrent[a].R.Ki = tuneCurrent[a].R.Kd * (1000.0f
					/ 123); //pidAutotuneConfig()->fw_ff_to_i_time_constant);
			tuneCurrent[a].R.Ki = Limit(tuneCurrent[a].R.Ki, 2.0f, 50.0f);
			autotuneUpdateGains(tuneCurrent);

			switch (a) {
			case Roll:
				//blackboxLogAutotuneEvent(ADJUSTMENT_ROLL_D, tuneCurrent[a].R.Kd);
				break;

			case Pitch:
				//blackboxLogAutotuneEvent(ADJUSTMENT_PITCH_D, tuneCurrent[a].R.Kd);
				break;

			case Yaw:
				//blackboxLogAutotuneEvent(ADJUSTMENT_YAW_D, tuneCurrent[a].Kd);
				break;
			}
		}

		// Change state and reset saturation flag
		tuneCurrent[a].state = newState;
		tuneCurrent[a].stateEnterTime = NowmS;
		tuneCurrent[a].pidSaturated = false;
	}
}

#endif

