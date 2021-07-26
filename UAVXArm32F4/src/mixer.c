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

const real32 AFOrientation[AFUnknown + 1] = { // K1 arm relative to board North
		-180, -180, -180, // TriAF, TriCoaxAF, VTailAF
				0, -45, 0, -45, // QuadAF, QuadXAF, QuadCoaxAF, QuadCoaxXAF
				0, -30, // HexAF, HexXAF
				0, -22.5, // OctAF, OctXAF
				0, 0, // Heli90AF, BiAF
				0, 0, 0, 0, 0, 0, // ElevonAF, DeltaAF, AileronAF, AileronSpoilerFlapsAF, AileronVTailAF, RudderElevatorAF,
				0, // DifferentialTwinAF
				0, 0, // VTOLAF, VTOL2AF
				0, 0, 0, // TrackedAF, FourWheelAF,TwoWheelAF,
				0, // GimbalAF,
				0, // Instrumentation
				0 }; // AFUnknown

const uint8 SM[] = { RightAileronC, LeftAileronC, ElevatorC, RudderC, SpoilerC,
		CamRollC, Aux1CamPitchC };

real32 PWSense[MAX_PWM_OUTPUTS];
real32 FWAileronDifferentialFrac = 0.0f;

real32 OrientationRad = 0.0f;
real32 OrientS = 0.0f;
real32 OrientC = 1.0f;

real32 IdleThrottlePW;
real32 NetThrottle;
real32 CGOffset;
boolean VTOLMode = false;

void RotateOrientation(real32 * nx, real32 * ny, real32 x, real32 y) {
	real32 Temp;

	Temp = x * OrientC + y * OrientS;
	*ny = -x * OrientS + y * OrientC;
	*nx = Temp;
}
// RotateOrientation

void DoDifferential(uint8 R, uint8 L) {

	real32 d = 1.0f - FWAileronDifferentialFrac;
	if (PW[R] > 0.0f)
		PW[R] *= d;

	if (PW[L] > 0.0f)
		PW[L] *= d;

} // DoDifferential

void MixAndLimitCam(void) {
	real32 NewCamPitch, NewCamRoll;

	if (UAVXAirframe == GimbalAF) { // Gimbal not commissioned
		NewCamRoll = Angle[Roll];
		NewCamPitch = Angle[Pitch];

		PW[CamRollC] = PW[Aux1CamPitchC] = OUT_NEUTRAL;
	} else {
		NewCamRoll = Angle[Roll] * Cam.RollKp + (real32) P(RollCamTrim) * 0.01f;
		NewCamRoll = (real32) PWSense[CamRollC] * NewCamRoll * OUT_MAXIMUM
				+ OUT_NEUTRAL;

		NewCamPitch = Angle[Pitch] * Cam.PitchKp + OrbitCamAngle + CamPitchTrim;
		NewCamPitch = PWSense[Aux1CamPitchC] * NewCamPitch * OUT_MAXIMUM
				+ OUT_NEUTRAL;

		PW[CamRollC] = NewCamRoll;
		PW[Aux1CamPitchC] = NewCamPitch;
	}

} // MixAndLimitCam

void DoVTOLMix(void) {
	real32 TempThrottle, TempElevator, TempRudder;

	TempThrottle = (F.PassThru) ? DesiredThrottle : DesiredThrottle + AltHoldThrComp;
	NetThrottle = TempThrottle = Limit(TempThrottle * OUT_MAXIMUM, 0.0f, 1.0f);

	if (VTOLMode) {

		PW[LeftThrottleC] = TempThrottle + Rl;
		PW[RightThrottleC] = TempThrottle - Rl;

		TempRudder = -PWSense[RudderC] * Yl;
		TempElevator = PWSense[ElevatorC] * Pl;

		// assume servos are opposite hand
		PW[RightPitchYawC] = PWSense[RightPitchYawC]
				* (TempElevator + TempRudder) + OUT_NEUTRAL;
		PW[LeftPitchYawC] = PWSense[LeftPitchYawC]
				* (-TempElevator + TempRudder) + OUT_NEUTRAL;

	} else {

		TempRudder = -PWSense[RudderC] * Yl; // use Roll not Yaw
		PW[LeftThrottleC] = TempThrottle + Yl;
		PW[RightThrottleC] = TempThrottle - Yl;

		TempElevator = PWSense[ElevatorC]
				* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)));
		// assume servos are opposite hand
		PW[RightPitchYawC] = PWSense[RightPitchYawC]
				* (TempElevator + TempRudder + Rl) + OUT_NEUTRAL;
		PW[LeftPitchYawC] = PWSense[LeftPitchYawC]
				* (-TempElevator + TempRudder + Rl) + OUT_NEUTRAL;
	}
} // DoVTOLMix

#if defined(DO_TILT)

/*
 control code for tiltrotors and tiltwings. Enabled by setting
 Q_TILT_MASK to a non-zero value
 */

/*
 calculate maximum tilt change as a proportion from 0 to 1 of tilt
 */
float QuadPlane::tilt_max_change(bool up)
{
	float rate;
	if (up || tilt.max_rate_down_dps <= 0) {
		rate = tilt.max_rate_up_dps;
	} else {
		rate = tilt.max_rate_down_dps;
	}
	if (tilt.tilt_type != TILT_TYPE_BINARY && !up) {
		bool fast_tilt = false;
		if (plane.control_mode == &plane.mode_manual) {
			fast_tilt = true;
		}
		if (hal.util->get_soft_armed() && !in_vtol_mode() && !assisted_flight) {
			fast_tilt = true;
		}
		if (fast_tilt) {
			// allow a minimum of 90 DPS in manual or if we are not
			// stabilising, to give fast control
			rate = MAX(rate, 90);
		}
	}
	return rate * plane.G_Dt / 90.0f;
}

/*
 output a slew limited tiltrotor angle. tilt is from 0 to 1
 */
void QuadPlane::tiltrotor_slew(float newtilt)
{
	float max_change = tilt_max_change(newtilt<tilt.current_tilt);
	tilt.current_tilt = constrain_float(newtilt, tilt.current_tilt-max_change, tilt.current_tilt+max_change);

	// translate to 0..1000 range and output
	SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * tilt.current_tilt);
}

/*
 update motor tilt for continuous tilt servos
 */
void QuadPlane::tiltrotor_continuous_update(void)
{
	// default to inactive
	tilt.motors_active = false;

	// the maximum rate of throttle change
	float max_change;

	if (!in_vtol_mode() && (!hal.util->get_soft_armed() || !assisted_flight)) {
		// we are in pure fixed wing mode. Move the tiltable motors all the way forward and run them as
		// a forward motor
		tiltrotor_slew(1);

		max_change = tilt_max_change(false);

		float new_throttle = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01, 0, 1);
		if (tilt.current_tilt < 1) {
			tilt.current_throttle = constrain_float(new_throttle,
					tilt.current_throttle-max_change,
					tilt.current_throttle+max_change);
		} else {
			tilt.current_throttle = new_throttle;
		}
		if (!hal.util->get_soft_armed()) {
			tilt.current_throttle = 0;
		} else {
			// prevent motor shutdown
			tilt.motors_active = true;
		}
		if (!motor_test.running) {
			// the motors are all the way forward, start using them for fwd thrust
			uint8_t mask = is_zero(tilt.current_throttle)?0:(uint8_t)tilt.tilt_mask.get();
			motors->output_motor_mask(tilt.current_throttle, mask, plane.rudder_dt);
		}
		return;
	}

	// remember the throttle level we're using for VTOL flight
	float motors_throttle = motors->get_throttle();
	max_change = tilt_max_change(motors_throttle<tilt.current_throttle);
	tilt.current_throttle = constrain_float(motors_throttle,
			tilt.current_throttle-max_change,
			tilt.current_throttle+max_change);

	/*
	 we are in a VTOL mode. We need to work out how much tilt is
	 needed. There are 4 strategies we will use:

	 1) without manual forward throttle control, the angle will be set to zero
	 in QAUTOTUNE QACRO, QSTABILIZE and QHOVER. This
	 enables these modes to be used as a safe recovery mode.

	 2) with manual forward throttle control we will set the angle based on
	 the demanded forward throttle via RC input.

	 3) in fixed wing assisted flight or velocity controlled modes we
	 will set the angle based on the demanded forward throttle,
	 with a maximum tilt given by Q_TILT_MAX. This relies on
	 Q_VFWD_GAIN being set.

	 4) if we are in TRANSITION_TIMER mode then we are transitioning
	 to forward flight and should put the rotors all the way forward
	 */

	if (plane.control_mode == &plane.mode_qautotune) {
		tiltrotor_slew(0);
		return;
	}

	// if not in assisted flight and in QACRO, QSTABILIZE or QHOVER mode
	if (!assisted_flight &&
			(plane.control_mode == &plane.mode_qacro ||
					plane.control_mode == &plane.mode_qstabilize ||
					plane.control_mode == &plane.mode_qhover)) {
		if (rc_fwd_thr_ch == nullptr) {
			// no manual throttle control, set angle to zero
			tiltrotor_slew(0);
		} else {
			// manual control of forward throttle
			float settilt = .01f * forward_throttle_pct();
			tiltrotor_slew(settilt);
		}
		return;
	}

	if (assisted_flight &&
			transition_state >= TRANSITION_TIMER) {
		// we are transitioning to fixed wing - tilt the motors all
		// the way forward
		tiltrotor_slew(1);
	} else {
		// until we have completed the transition we limit the tilt to
		// Q_TILT_MAX. Anything above 50% throttle gets
		// Q_TILT_MAX. Below 50% throttle we decrease linearly. This
		// relies heavily on Q_VFWD_GAIN being set appropriately.
		float settilt = constrain_float(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) / 50.0f, 0, 1);
		tiltrotor_slew(settilt * tilt.max_angle_deg / 90.0f);
	}
}

/*
 output a slew limited tiltrotor angle. tilt is 0 or 1
 */
void QuadPlane::tiltrotor_binary_slew(bool forward)
{
	// The servo output is binary, not slew rate limited
	SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, forward?1000:0);

	// rate limiting current_tilt has the effect of delaying throttle in tiltrotor_binary_update
	float max_change = tilt_max_change(!forward);
	if (forward) {
		tilt.current_tilt = constrain_float(tilt.current_tilt+max_change, 0, 1);
	} else {
		tilt.current_tilt = constrain_float(tilt.current_tilt-max_change, 0, 1);
	}
}

/*
 update motor tilt for binary tilt servos
 */
void QuadPlane::tiltrotor_binary_update(void)
{
	// motors always active
	tilt.motors_active = true;

	if (!in_vtol_mode()) {
		// we are in pure fixed wing mode. Move the tiltable motors
		// all the way forward and run them as a forward motor
		tiltrotor_binary_slew(true);

		float new_throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)*0.01f;
		if (tilt.current_tilt >= 1) {
			uint8_t mask = is_zero(new_throttle)?0:(uint8_t)tilt.tilt_mask.get();
			// the motors are all the way forward, start using them for fwd thrust
			motors->output_motor_mask(new_throttle, mask, plane.rudder_dt);
		}
	} else {
		tiltrotor_binary_slew(false);
	}
}

/*
 update motor tilt
 */
void QuadPlane::tiltrotor_update(void)
{
	if (tilt.tilt_mask <= 0) {
		// no motors to tilt
		return;
	}

	if (tilt.tilt_type == TILT_TYPE_BINARY) {
		tiltrotor_binary_update();
	} else {
		tiltrotor_continuous_update();
	}

	if (tilt.tilt_type == TILT_TYPE_VECTORED_YAW) {
		tiltrotor_vectored_yaw();
	}
}

/*
 compensate for tilt in a set of motor outputs

 Compensation is of two forms. The first is to apply _tilt_factor,
 which is a compensation for the reduces vertical thrust when
 tilted. This is supplied by set_motor_tilt_factor().

 The second compensation is to use equal thrust on all tilted motors
 when _tilt_equal_thrust is true. This is used when the motors are
 tilted by a large angle to prevent the roll and yaw controllers from
 causing instability. Typically this would be used when the motors
 are tilted beyond 45 degrees. At this angle it is assumed that roll
 control can be achieved using fixed wing control surfaces and yaw
 control with the remaining multicopter motors (eg. tricopter tail).

 By applying _tilt_equal_thrust the tilted motors effectively become
 a single pitch control motor.

 Note that we use a different strategy for when we are transitioning
 into VTOL as compared to from VTOL flight. The reason for that is
 we want to lean towards higher tilted motor throttle when
 transitioning to fixed wing flight, in order to gain airspeed,
 whereas when transitioning to VTOL flight we want to lean to towards
 lower fwd throttle. So we raise the throttle on the tilted motors
 when transitioning to fixed wing, and lower throttle on tilted
 motors when transitioning to VTOL
 */
void QuadPlane::tilt_compensate_down(float *thrust, uint8_t num_motors)
{
	float inv_tilt_factor;
	if (tilt.current_tilt > 0.98f) {
		inv_tilt_factor = 1.0 / cosf(radians(0.98f*90));
	} else {
		inv_tilt_factor = 1.0 / cosf(radians(tilt.current_tilt*90));
	}

	// when we got past Q_TILT_MAX we gang the tilted motors together
	// to generate equal thrust. This makes them act as a single pitch
	// control motor while preventing them trying to do roll and yaw
	// control while angled over. This greatly improves the stability
	// of the last phase of transitions
	float tilt_threshold = (tilt.max_angle_deg/90.0f);
	bool equal_thrust = (tilt.current_tilt > tilt_threshold);

	float tilt_total = 0;
	uint8_t tilt_count = 0;

	// apply inv_tilt_factor first
	for (uint8_t i=0; i<num_motors; i++) {
		if (is_motor_tilting(i)) {
			thrust[i] *= inv_tilt_factor;
			tilt_total += thrust[i];
			tilt_count++;
		}
	}

	float largest_tilted = 0;

	// now constrain and apply _tilt_equal_thrust if enabled
	for (uint8_t i=0; i<num_motors; i++) {
		if (is_motor_tilting(i)) {
			if (equal_thrust) {
				thrust[i] = tilt_total / tilt_count;
			}
			largest_tilted = MAX(largest_tilted, thrust[i]);
		}
	}

	// if we are saturating one of the tilted motors then reduce all
	// motors to keep them in proportion to the original thrust. This
	// helps maintain stability when tilted at a large angle
	if (largest_tilted > 1.0f) {
		float scale = 1.0f / largest_tilted;
		for (uint8_t i=0; i<num_motors; i++) {
			thrust[i] *= scale;
		}
	}
}

/*
 tilt compensation when transitioning to VTOL flight
 */
void QuadPlane::tilt_compensate_up(float *thrust, uint8_t num_motors)
{
	float tilt_factor = cosf(radians(tilt.current_tilt*90));

	// when we got past Q_TILT_MAX we gang the tilted motors together
	// to generate equal thrust. This makes them act as a single pitch
	// control motor while preventing them trying to do roll and yaw
	// control while angled over. This greatly improves the stability
	// of the last phase of transitions
	float tilt_threshold = (tilt.max_angle_deg/90.0f);
	bool equal_thrust = (tilt.current_tilt > tilt_threshold);

	float tilt_total = 0;
	uint8_t tilt_count = 0;

	// apply tilt_factor first
	for (uint8_t i=0; i<num_motors; i++) {
		if (!is_motor_tilting(i)) {
			thrust[i] *= tilt_factor;
		} else {
			tilt_total += thrust[i];
			tilt_count++;
		}
	}

	// now constrain and apply _tilt_equal_thrust if enabled
	for (uint8_t i=0; i<num_motors; i++) {
		if (is_motor_tilting(i)) {
			if (equal_thrust) {
				thrust[i] = tilt_total / tilt_count;
			}
		}
	}
}

/*
 choose up or down tilt compensation based on flight mode When going
 to a fixed wing mode we use tilt_compensate_down, when going to a
 VTOL mode we use tilt_compensate_up
 */
void QuadPlane::tilt_compensate(float *thrust, uint8_t num_motors)
{
	if (tilt.current_tilt <= 0) {
		// the motors are not tilted, no compensation needed
		return;
	}
	if (in_vtol_mode()) {
		// we are transitioning to VTOL flight
		tilt_compensate_up(thrust, num_motors);
	} else {
		tilt_compensate_down(thrust, num_motors);
	}
}

/*
 return true if the rotors are fully tilted forward
 */
bool QuadPlane::tiltrotor_fully_fwd(void)
{
	if (tilt.tilt_mask <= 0) {
		return false;
	}
	return (tilt.current_tilt >= 1);
}

/*
 control vectored yaw with tilt multicopters
 */
void QuadPlane::tiltrotor_vectored_yaw(void)
{
	// total angle the tilt can go through
	float total_angle = 90 + tilt.tilt_yaw_angle;
	// output value (0 to 1) to get motors pointed straight up
	float zero_out = tilt.tilt_yaw_angle / total_angle;

	// calculate the basic tilt amount from current_tilt
	float base_output = zero_out + (tilt.current_tilt * (1 - zero_out));

	// for testing when disarmed, apply vectored yaw in proportion to rudder stick
	// Wait TILT_DELAY_MS after disarming to allow props to spin down first.
	constexpr uint32_t TILT_DELAY_MS = 3000;
	uint32_t now = AP_HAL::millis();
	if (!hal.util->get_soft_armed() && (plane.quadplane.options & OPTION_DISARMED_TILT)) {
		// this test is subject to wrapping at ~49 days, but the consequences are insignificant
		if ((now - hal.util->get_last_armed_change()) > TILT_DELAY_MS) {
			float yaw_out = plane.channel_rudder->get_control_in();
			yaw_out /= plane.channel_rudder->get_range();
			float yaw_range = zero_out;

			SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 1000 * (base_output + yaw_out * yaw_range));
			SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * (base_output - yaw_out * yaw_range));
		}
		return;
	}

	float tilt_threshold = (tilt.max_angle_deg/90.0f);
	bool no_yaw = (tilt.current_tilt > tilt_threshold);
	if (no_yaw) {
		SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 1000 * base_output);
		SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * base_output);
	} else {
		float yaw_out = motors->get_yaw();
		float yaw_range = zero_out;

		SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 1000 * (base_output + yaw_out * yaw_range));
		SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 1000 * (base_output - yaw_out * yaw_range));
	}
}

/*
 control bicopter tiltrotors
 */
void QuadPlane::tiltrotor_bicopter(void)
{
	if (tilt.tilt_type != TILT_TYPE_BICOPTER || motor_test.running) {
		// don't override motor test with motors_output
		return;
	}

	if (!in_vtol_mode() && tiltrotor_fully_fwd()) {
		SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, -SERVO_MAX);
		SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
		return;
	}

	float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
	if (assisted_flight) {
		hold_stabilize(throttle * 0.01f);
		motors_output(true);
	} else {
		motors_output(false);
	}

	// bicopter assumes that trim is up so we scale down so match
	float tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
	float tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

	if (is_negative(tilt_left)) {
		tilt_left *= tilt.tilt_yaw_angle / 90.0f;
	}
	if (is_negative(tilt_right)) {
		tilt_right *= tilt.tilt_yaw_angle / 90.0f;
	}

	// reduce authority of bicopter as motors are tilted forwards
	const float scaling = cosf(tilt.current_tilt * M_PI_2);
	tilt_left *= scaling;
	tilt_right *= scaling;

	// add current tilt and constrain
	tilt_left = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_left, -SERVO_MAX, SERVO_MAX);
	tilt_right = constrain_float(-(tilt.current_tilt * SERVO_MAX) + tilt_right, -SERVO_MAX, SERVO_MAX);

	SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
	SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
}

#endif

void DoGroundVehicleMix(void) {
#if defined(INCLUDE_BALANCING)
	// Derived from BalancingWii

	/****************** PI_speed + PD_angle regulator *****************/
	int16 targetSpeed = Limit1(rcCommand[Pitch], MAX_SPEED);
	int16 steering = Limit1(rcCommand[Roll]>>2, MAX_STEERING);
	steering = f.SIMPLE_MODE ? (steering * 2 / 3) : steering;

	actualSpeed = (actualMotorSpeed[1] - actualMotorSpeed[0]) / 2; // Positive: forward

	/**** position hold mode ****/
	static real32 positionError = 0.0f;
	if (f.POSHOLD_MODE && abs(targetSpeed) < 15 && abs(steering) < 15)
	positionError += actualSpeed * (real32) cycleTime * 0.000001f;
	else
	positionError = 0.0f;

	/**** PI_speed regulator ****/
	static real32 actualAveragedSpeed = 0.0f;
	actualAveragedSpeed = actualAveragedSpeed * 0.92f + (real32) actualSpeed
	* 0.08f;
	error = targetSpeed - actualAveragedSpeed - (positionError
			* conf.pid[PIDPOS].P8 * 0.01f); //16 bits is ok here

	speedErrorI
	= Limit1(speedErrorI + (int16)(((int32)error * cycleTime)>>11), 20000);//16 bits is ok here

	int16 maxTargetAngle = f.SIMPLE_MODE ? (MAX_TARGET_ANGLE * 2 / 3)
	: MAX_TARGET_ANGLE;

	int16
	targetAngle =// PTerm + ITerm
	(((int32) error * conf.pid[PIDSPEED].P8) >> 7)// 32 bits is needed for calculation: angleError*P8 could exceed 32768   16 bits is ok for result
	+ Limit1( (((int32)speedErrorI * conf.pid[PIDSPEED].I8)>>14), maxTargetAngle/6);// 32 bits is needed for calculation:10000*I8 could exceed 32768   16 bits is ok for result

	targetAngle = Limit1(targetAngle, maxTargetAngle);

	/**** PD_angle regulator ****/
	int16 currAngle = att.angle[CURRENT_AXIS] + conf.angleTrim[CURRENT_AXIS];
#ifdef INVERT_CURRENT_AXIS
	currAngle = -currAngle;
#endif
	int16 angleError = targetAngle - currAngle; //16 bits is ok here

	int16 acceleration =// PTerm - DTerm
	(((int32) angleError * conf.pid[PIDANGLE].P8) >> 4)// 32 bits is needed for calculation: error*P8 could exceed 32768   16 bits is ok for result
	- (((int32) imu.gyroData[CURRENT_AXIS]
					* conf.pid[PIDANGLE].D8) >> 5);// 32 bits is needed for calculation

	static real32 speed = 0.0f;
	speed
	= Limit1(speed + ((real32)acceleration * (real32)cycleTime * 0.000001f), MAX_SPEED);

	/**** rise mode ****/

#define MAX_RISE_SPEED		140
#define MAX_REVERSED_RISE_SPEED	100

	static uint8 risePhase = 2; // to prevent rising without switching off before
	real32 dynK = 0.0f;
	if (Armed()) {
		real32 currAbsAngle = Abs(Angle[Pitch]);
		if (currAbsAngle < DegreesToRadians(25.0f)) { // if angle less than 25 degree
			dynK = 1.0f;

		} else if (currAbsAngle < DegreesToRadians(80.0f)) { // help to rise with less speed but more torque
			dynK = (1000.0f - currAbsAngle) * 0.0001f + 0.08f;
			risePhase = 2;// to prevent rising without switching off before

		} else {
			dynK = 1.0f;

			if (f.RISE_MODE) { // if robot fell, use it to auto rise! ;)
				static real32 riseSpeed = 0;
				if (risePhase == 0) { // get direct acceleration
					riseSpeed
					= Limit(riseSpeed + (0.7f * RISE_SPEED_K), 0, MAX_RISE_SPEED);
					speed = (currAngle > 0) ? riseSpeed : -riseSpeed;// forward direction
					if (riseSpeed >= MAX_RISE_SPEED) {
						riseSpeed = 0.0f; // force stop (it will throw up the robot) and prepare for next phase in reverse
						risePhase = 1;
					}
				} else if (risePhase == 1) { // get reversed acceleration to rise
					riseSpeed
					= Limit(riseSpeed + (0.85f * RISE_SPEED_K), 0, MAX_REVERSED_RISE_SPEED);
					speed = (currAngle > 0) ? -riseSpeed : riseSpeed;// backward direction
					if (riseSpeed >= MAX_REVERSED_RISE_SPEED)
					risePhase = 2;

				} else if (risePhase == 2) // prepare for the next rise
				riseSpeed = speed = 0.0f;

				steering = 0;// to prevent turning during auto rising

			} else { // if manual mode for rising
				speed = Limit1(-targetSpeed/2, MAX_SPEED/2);
				steering = (abs(targetSpeed) < 100) ? steering / 2 : 0;// to prevent turning during acceleration
				risePhase = 0;// reset rise phase
			}
		}

	} else { // turn off the motors
		speed = 0.0f;
		steering = 0;
		risePhase = 2;// to prevent rising without switching off before
	}

	int16 outputSpeed = Limit1(speed * dynK, MAX_SPEED);

	// to don't lost a control on big speeds and not overlimit the MAX_SPEED
	if ((abs(outputSpeed) + abs(steering)) > MAX_SPEED)
	outputSpeed = (outputSpeed > 0) ? (MAX_SPEED - abs(steering))
	: (-MAX_SPEED + abs(steering));

	// apply both motor speed
	PW[0] = outputSpeed + steering;// right motor
	PW[1] = -outputSpeed + steering;// left motor

#endif

} // DoVehicleMix

void DoMix(void) {
#define OUT_MAX_SPOILER 0.3f // so we still have some aileron control left
	real32 TempThrottle, TempRudder, TempElevator, TempAileron,
			TempSpoilerFlaps;

	if (F.PassThru) // do here at lowest level rather than complicating higher level logic
		TempThrottle = DesiredThrottle;
	else
		TempThrottle = ThrottleSuppressed ? 0.0f : DesiredThrottle + AltHoldThrComp;

	NetThrottle = PW[RightThrottleC] = PW[LeftThrottleC] = Limit(
			TempThrottle * OUT_MAXIMUM, 0.0f, 1.0f);

	switch (UAVXAirframe) {
	case Heli90AF:
		PW[RudderC] = PWSense[RudderC] * Yl + OUT_NEUTRAL;
		PW[RightAileronC] = PWSense[RightAileronC] * Rl + OUT_NEUTRAL;
		// left aileron not used
		PW[ElevatorC] = PWSense[ElevatorC] * Pl + OUT_NEUTRAL;

		break;
	case BiAF:
		PW[LeftThrottleC] -= Rl;
		PW[RightThrottleC] += Rl;

		PW[LeftTiltServoC] = PWSense[LeftTiltServoC] * (Pl + Yl) + OUT_NEUTRAL;
		PW[RightTiltServoC] = PWSense[RightTiltServoC] * (-Pl - Yl) + OUT_NEUTRAL;

		break;
	case ElevonAF:
	case DeltaAF:
		if (VTOLMode) {
			PW[RudderC] = -PWSense[RudderC] * Rl + OUT_NEUTRAL;

			TempElevator = PWSense[ElevatorC] * Pl;
			// assume servos are opposite hand
			PW[RightElevonC] = PWSense[RightElevonC]
					* (TempElevator + Yl) + OUT_NEUTRAL;
			PW[LeftElevonC] = PWSense[LeftElevonC]
					* (-TempElevator + Yl) + OUT_NEUTRAL;
		} else {
			PW[RudderC] = -PWSense[RudderC] * Yl + OUT_NEUTRAL;

			TempElevator = PWSense[ElevatorC]
					* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)));
			// assume servos are opposite hand
			PW[RightElevonC] = PWSense[RightElevonC]
					* (TempElevator + Rl) + OUT_NEUTRAL;
			PW[LeftElevonC] = PWSense[LeftElevonC]
					* (-TempElevator + Rl) + OUT_NEUTRAL;
		}
		// assume servos are opposite hand
		PW[SpoilerC] = PWSense[SpoilerC] * Sl;

		break;
	case AileronAF:
		PW[RudderC] = PWSense[RudderC] * Yl + OUT_NEUTRAL;

		PW[RightAileronC] = Rl;
		PW[LeftAileronC] = -Rl;
		DoDifferential(RightAileronC, LeftAileronC);
		PW[RightAileronC] *= PWSense[RightAileronC];
		PW[LeftAileronC] *= -PWSense[LeftAileronC];
		PW[RightAileronC] += OUT_NEUTRAL;
		PW[LeftAileronC] += OUT_NEUTRAL;

		PW[ElevatorC] =
				PWSense[ElevatorC]
						* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)))+ OUT_NEUTRAL;

		// assume servos are opposite hand
		PW[SpoilerC] = PWSense[SpoilerC] * Sl + OUT_NEUTRAL;

		break;
	case AileronSpoilerFlapsAF:

		PW[RudderC] = PWSense[RudderC] * Yl + OUT_NEUTRAL;

		PW[RightAileronC] = Rl;
		PW[LeftAileronC] = -Rl;
		DoDifferential(RightAileronC, LeftAileronC);
		PW[RightAileronC] *= PWSense[RightAileronC];
		PW[LeftAileronC] *= -PWSense[LeftAileronC];

		TempSpoilerFlaps = -PWSense[SpoilerC] * Sl * OUT_MAX_SPOILER;
		PW[RightAileronC] =
				(TempSpoilerFlaps + PW[RightAileronC]) + OUT_NEUTRAL;
		PW[LeftAileronC] +=
				(-TempSpoilerFlaps + PW[LeftAileronC]) + OUT_NEUTRAL;

		PW[ElevatorC] =
				PWSense[ElevatorC]
						* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)))+ OUT_NEUTRAL;
		break;
	case AileronVTailAF:
		PW[RightAileronC] = Rl;
		PW[LeftAileronC] = -Rl;
		DoDifferential(RightAileronC, LeftAileronC);
		PW[RightAileronC] *= PWSense[RightAileronC];
		PW[LeftAileronC] *= -PWSense[LeftAileronC];
		PW[RightAileronC] += OUT_NEUTRAL;
		PW[LeftAileronC] += OUT_NEUTRAL;

		TempElevator = (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)));
		PW[RightRudderElevatorC] = PWSense[RightRudderElevatorC]
				* TempElevator + OUT_NEUTRAL;
		PW[LeftRudderElevatorC] = -PWSense[LeftRudderElevatorC]
				* TempElevator + OUT_NEUTRAL;

		PW[RightRudderElevatorC] -= Yl;
		PW[LeftRudderElevatorC] -= Yl;

		PW[SpoilerC] = PWSense[SpoilerC] * Sl + OUT_NEUTRAL;

		break;
	case RudderElevatorAF:
		TempAileron = PWSense[RightAileronC] * Rl;

		PW[RudderC] = PWSense[RudderC] * (TempAileron + Yl) + OUT_NEUTRAL;

		PW[ElevatorC] =
				PWSense[ElevatorC]
						* (F.PassThru ? Pl : (Pl + FWRollPitchFFFrac * Abs(Rl)))+ OUT_NEUTRAL;

		PW[SpoilerC] = PWSense[SpoilerC] * Sl + OUT_NEUTRAL;
		break;
	case DifferentialTwinAF:
		TempThrottle = PW[RightThrottleC];
		TempAileron = PWSense[RightAileronC] * Rl;
		PW[RightThrottleC] = TempThrottle + TempAileron;
		PW[LeftThrottleC] = TempThrottle - TempAileron;
		break;
	case VTOL2AF:
	case VTOLAF:
		DoVTOLMix();
		break;

	default:
		break;
	} // switch

} // DoMix

void MixMulti(void) {
	real32 R, P, Y;
	idx m;

	switch (UAVXAirframe) {
	case TriAF: // usually flown K1 motor to the rear - use orientation of 24
		R = Rl * 1.1547f;
		P = Pl * (1.0f + CGOffset);
		PW[LeftC] = -R + P;
		PW[RightC] = R + P;

		PW[FrontC] = -(Pl * (1.0f - CGOffset));

		PW[YawC] = PWSense[YawC] * Yl + OUT_NEUTRAL; // * 1.3333 yaw servo
		break;
	case TriCoaxAF: // Y6
		R = Rl * 1.1547f;
		PW[FrontBC] = PW[FrontTC] = -Pl;
		PW[LeftBC] = PW[LeftTC] = -R + Pl;
		PW[RightBC] = PW[RightTC] = R + Pl;

		Y = Yl * 0.6667f;
		PW[FrontTC] += Y;
		PW[LeftTC] += Y;
		PW[RightTC] += Y;

		PW[FrontBC] -= Y;
		PW[LeftBC] -= Y;
		PW[RightBC] -= Y;
		break;
	case VTailAF: // usually flown VTail (K1+K4) to the rear
		P = Pl * (1.0f + CGOffset);
		PW[LeftC] = P - Rl; // right rear
		PW[RightC] = P + Rl; // left rear

		P = Pl * (1.0f - CGOffset);
		PW[FrontLeftC] = -(P + PWSense[RudderC] * Yl);
		PW[FrontRightC] = -(P - PWSense[RudderC] * Yl);
		break;
	case QuadAF:
	case QuadXAF:
	case QuadCoaxAF:
	case QuadCoaxXAF: // not commissioned
		Yl *= MultiPropSense;
		PW[LeftC] = -Rl - Yl;
		PW[RightC] = Rl - Yl;
		PW[FrontC] = -Pl + Yl;
		PW[BackC] = Pl + Yl;
		break;
	case HexAF:
	case HexXAF:
		Yl *= MultiPropSense;
		P = Pl * 0.5f;
		R = Rl * 0.5773503f;
		Y = Yl; //* 0.6667f;
		PW[HFrontC] = -P + Y;
		PW[HLeftFrontC] = -R - P - Y;
		PW[HRightFrontC] = R - P - Y;

		PW[HLeftBackC] = -R + P + Y;
		PW[HRightBackC] = R + P + Y;
		PW[HBackC] = P - Y;
		break;
	case OctAF:
	case OctXAF: // use Y leads
		Yl *= MultiPropSense;
		PW[LeftC] = (-Rl - Yl) * 0.5f;
		PW[RightC] = (Rl - Yl) * 0.5f;
		PW[FrontC] = (-Pl + Yl) * 0.5f;
		PW[BackC] = (Pl + Yl) * 0.5f;
		break;
	default:
		break;
	} // switch
} // MixMulti

real32 MaxMotorSwing(void) {
	real32 DemandSwing, pw;
	idx m;

	DemandSwing = Abs(PW[0]);
	for (m = 1; m < NoOfDrives; m++) {
		pw = Abs(PW[m]);
		if (pw > DemandSwing)
			DemandSwing = pw;
	}

	return (DemandSwing);
} // MaxMotorSwing

void RescaledMultiMix(real32 CurrThrottlePW) {
#define	MIN_PRESERVED_YAW_PW FromPercent(10)
	idx m;
	real32 Scale, DemandSwing, AvailableSwing, TempYl;

	TempYl = Yl;
	Yl = 0.0f;

	MixMulti(); // without yaw

	AvailableSwing = Min(OUT_MAXIMUM - CurrThrottlePW,
			CurrThrottlePW - THR_START_PW);
	DemandSwing = MaxMotorSwing();

	F.Saturation = DemandSwing > AvailableSwing;

	if (F.Saturation) { // nothing left for yaw!

		Scale = AvailableSwing / (DemandSwing - MIN_PRESERVED_YAW_PW);
		Rl *= Scale;
		Pl *= Scale;

		Yl = Limit1(TempYl, MIN_PRESERVED_YAW_PW);
		MixMulti();
	} else {
		Yl = Limit1(TempYl, AvailableSwing - DemandSwing);
		MixMulti();
	}

} // RescaledMultiMix

void DoMulticopterMix(void) {
	real32 CurrThrottlePW, AvailableSwing, MinThrottle;
	idx m;

	RotateOrientation(&Rl, &Pl, Rl, Pl);

	CurrThrottlePW =
			(State == InFlight) ?
					(DesiredThrottle + AltHoldThrComp) * OUT_MAXIMUM
							* TiltThrScale:
					DesiredThrottle * OUT_MAXIMUM;

	CurrThrottlePW = Limit(CurrThrottlePW, IdleThrottlePW, OUT_MAXIMUM);

	if ((CurrThrottlePW < IdleThrottlePW) || !F.DrivesArmed) {
		CurrThrottlePW = 0.0f;
		for (m = 0; m < NoOfDrives; m++)
			PW[m] = PWp[m] = 0;
	} else {
		F.EnforceDriveSymmetry = false; //TODO: BROKEN at low throttle - maybe try yaw preserve?;
		if (F.EnforceDriveSymmetry)
			RescaledMultiMix(CurrThrottlePW);
		else
			MixMulti();

		MinThrottle = (State == InFlight) ? IdleThrottlePW : 0.0f;
		for (m = 0; m < NoOfDrives; m++)
			PW[m] = Limit(PW[m] + CurrThrottlePW, MinThrottle, OUT_MAXIMUM);
	}
	NetThrottle = CurrThrottlePW; // for logging only

} // DoMulticopterMix

void InitServoSense(void) {
	idx b, m;

	for (m = 0; m < MAX_PWM_OUTPUTS; m++)
		PWSense[m] = 1.0f;

	b = P(ServoSense);
	for (m = 0; m <= 6; m++) { // RightAileron .. CamPitch
		PWSense[SM[m]] = ((b & 1) ? -1.0f : 1.0f);
		b >>= 1;
	}

} // InitServoSense

