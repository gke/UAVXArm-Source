/*
 *       Based originally on AP_OpticalFlow_ADNS3080.cpp
 *       ADNS3080 OpticalFlow Library for Ardupilot Mega
 *       Code by Randy Mackay. DIYDrones.com
 *
 *       Adapted for UAVX by Prof. Greg Egan 2015
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 */

#include "UAVX.h"

#define ADNS_ID 0xff

#if defined(USE_OPTICAL_ADNS3080)

typedef union {
	int16 i16;
	uint16 u16;
	uint8 b[2];
}numUnion;

const real32 field_of_view = ADNS3080_08_FOV; // field of view in Radians

real32 scaler = ADNS3080_SCALER; // number returned from sensor when moved one pixel
int16 raw_dx, raw_dy; // raw sensor change in x and y position (i.e. unrotated)
int16 surface_quality; // image quality (below 15 you really can't trust the x,y values returned)
int16 x, y; // total x,y position
int16 dx, dy; // rotated change in x and y position

real32 vlon, vlat; // position as offsets from original position

int16 num_pixels; // number of pixels of resolution in the sensor
uint32 adns3080PrevUpdatemS; // millis() time of last update

real32 orientation, _orientation;
real32 conv_factor; // multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
real32 radians_to_pixels;

boolean motion; // true if there has been motion
boolean overflow; // true if the x or y data buffers overflowed

// reset sensor by holding a pin high (or is it low?) for 10us.
void adns3080reset(void) {
	if (PWMPins[Aux2Sel].Used) {
	// return immediately if the reset pin is not defined
	//if (_reset_pin != 0) {
	DigitalWrite(&PWMPins[Aux2Sel].P, true); // reset sensor
	Delay1uS(10);
	DigitalWrite(&PWMPins[Aux2Sel].P, false); // return sensor to normal
	//}
	}
} // adns3080reset


// rotate raw values to arrive at final x, y, dx and dy values
void adns3080apply_orientation_matrix(void) {
	//  Vector3f rot_vector;
	//  rot_vector(raw_dx, raw_dy, 0);

	// next rotate dx and dy
	//  rot_vector.rotate(_orientation);

	//  dx = rot_vector.x;
	//  dy = rot_vector.y;

	// add rotated values to totals (perhaps this is pointless as we need to take into account yaw, roll, pitch)
	x += dx;
	y += dy;
} // adns3080apply_orientation_matrix


void adns3080update(uint32 NowmS) {

	surface_quality = SIORead(flowSel, ADNS3080_SQUAL);

	Delay1uS(50);

	overflow = (SIORead(flowSel, ADNS3080_MOTION) & 0x10) != 0;
	motion = (SIORead(flowSel, ADNS3080_MOTION) & 0x80) != 0;

	if (motion) {
		raw_dx = SIORead(flowSel, ADNS3080_DELTA_X);
		Delay1uS(50);
		raw_dy = SIORead(flowSel, ADNS3080_DELTA_Y);
	} else
	raw_dx = raw_dy = 0;

	adns3080PrevUpdatemS = NowmS;

	adns3080apply_orientation_matrix();
} // adns3080update

boolean adns3080initxxx(void) {

	//    pinMode(_cs_pin, OUTPUT);
	//if (_reset_pin != 0) {
	//      pinMode(ADNS3080_RESET, OUTPUT);
	//}

	adns3080reset();

	return (SIORead(flowSel, ADNS3080_PRODUCT_ID) == 0x17);
} // adns3080initxxx

void adns3080disable_serial_pullup(void) {
	uint8 r;

	r = SIORead(flowSel, ADNS3080_EXTENDED_CONFIG);
	r = (r | ADNS3080_SERIALNPU_OFF);
	Delay1uS(50);
	SIOWrite(flowSel, ADNS3080_EXTENDED_CONFIG, r);
} // adns3080disable_serial_pullup

// get_led_always_on - returns true if LED is always on, false if only on when required
boolean adns3080get_led_always_on(void) {
	return ((SIORead(flowSel, ADNS3080_CONFIGURATION_BITS) & 0x40) > 0);
} // adns3080get_led_always_on

// set_led_always_on - set parameter to true if you want LED always on, otherwise false for only when required
void adns3080set_led_always_on(boolean alwaysOn) {
	uint8 r = SIORead(flowSel, ADNS3080_CONFIGURATION_BITS);
	r = (r & 0xbf) | (alwaysOn << 6);
	Delay1uS(50);
	SIOWrite(flowSel, ADNS3080_CONFIGURATION_BITS, r);
}

// returns resolution (either 400 or 1600 counts per inch)
int16 adns3080get_resolution(void) {

	return ((SIORead(flowSel, ADNS3080_CONFIGURATION_BITS) & 0x10)
			== 0 ? 400 : 1600);
} // adns3080get_resolution

// update conversion factors that are dependent upon field_of_view
void adns3080update_conversion_factors(void) {

	conv_factor = (1.0f / (real32) (num_pixels * scaler)) * 2.0f * tanf(
			field_of_view * 0.5f); // multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
	// 0.00615
	radians_to_pixels = (num_pixels * scaler) / field_of_view;
	// 162.99
} // adns3080update_conversion_factors


void adns3080set_resolution(uint16 resolution) {

	uint8 r = SIORead(flowSel, ADNS3080_CONFIGURATION_BITS);

	if (resolution == ADNS3080_RESOLUTION_400) {
		r &= ~0x10;
		scaler = ADNS3080_SCALER;
	} else { // default ADNS3080_RESOLUTION_1600
		r |= 0x10;
		scaler = ADNS3080_SCALER * 4;
	}

	Delay1uS(50);
	SIOWrite(flowSel, ADNS3080_CONFIGURATION_BITS, r);

	// this will affect conversion factors so update them
	adns3080update_conversion_factors();
} // adns3080set_resolution

boolean adns3080get_frame_rate_auto(void) {
	return ((SIORead(flowSel, ADNS3080_EXTENDED_CONFIG) & 0x01) == 0);
} // adns3080get_frame_rate_auto


void adns3080set_frame_rate_auto(boolean auto_frame_rate) {

	uint8 r = SIORead(flowSel, ADNS3080_EXTENDED_CONFIG);

	Delay1uS(50);
	if (auto_frame_rate) {
		SIOWrite(flowSel, ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER, 0xE0);
		Delay1uS(50);
		SIOWrite(flowSel, ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER, 0x1A);
		Delay1uS(50);
		r = (r & ~0x01);
	} else
	r = (r & ~0x01) | 0x01;

	SIOWrite(flowSel, ADNS3080_EXTENDED_CONFIG, r);
} // adns3080set_frame_rate_auto


uint16 adns3080get_frame_period(void) {

	numUnion val;
	val.b[1] = SIORead(flowSel, ADNS3080_FRAME_PERIOD_UPPER);
	Delay1uS(50);
	val.b[0] = SIORead(flowSel, ADNS3080_FRAME_PERIOD_LOWER);
	return (val.u16);

} // adns3080get_frame_period


void adns3080set_frame_period(uint16 period) {
	numUnion val;
	val.u16 = period;

	// set frame rate to manual
	adns3080set_frame_rate_auto(false);
	Delay1uS(50);

	// set specific frame period
	SIOWrite(flowSel, ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER, val.b[0]);
	Delay1uS(50);
	SIOWrite(flowSel, ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER, val.b[1]);

} // adns3080set_frame_period

uint16 adns3080get_frame_rate(void) {

	uint32 clockSpeed = ADNS3080_CLOCK_SPEED;
	uint16 rate = clockSpeed / adns3080get_frame_period();
	return rate;
} // adns3080get_frame_rate

void adns3080set_frame_rate(uint16 rate) {
	uint32 clockSpeed = ADNS3080_CLOCK_SPEED;
	uint16 period = (uint16) (clockSpeed / (uint32) rate);

	adns3080set_frame_period(period);
} // adns3080set_frame_rate


boolean adns3080get_shutter_speed_auto(void) { // true if auto
	return ((SIORead(flowSel, ADNS3080_EXTENDED_CONFIG) & 0x02) == 0);
} // adns3080get_shutter_speed_auto


void adns3080set_shutter_speed_auto(boolean auto_shutter_speed) {

	uint8 r = SIORead(flowSel, ADNS3080_EXTENDED_CONFIG);

	Delay1uS(50);
	if (auto_shutter_speed) {
		// return shutter speed max to default
		SIOWrite(flowSel, ADNS3080_SHUTTER_MAX_BOUND_LOWER, 0x8c);
		Delay1uS(50);
		SIOWrite(flowSel, ADNS3080_SHUTTER_MAX_BOUND_UPPER, 0x20);
		Delay1uS(50);

		r &= ~0x02;
	} else
	r |= 0x02;

	SIOWrite(flowSel, ADNS3080_EXTENDED_CONFIG, r);
	Delay1uS(50);
}

uint16 adns3080get_shutter_speed(void) {
	numUnion val;

	val.b[1] = SIORead(flowSel, ADNS3080_SHUTTER_UPPER);
	Delay1uS(50);
	val.b[0] = SIORead(flowSel, ADNS3080_SHUTTER_LOWER);

	return (val.u16);
} // adns3080get_shutter_speed


void adns3080set_shutter_speed(uint16 shutter_speed) {
	numUnion val;

	val.u16 = shutter_speed;

	adns3080set_shutter_speed_auto(false);
	Delay1uS(50);

	SIOWrite(flowSel, ADNS3080_SHUTTER_MAX_BOUND_LOWER, val.b[0]);
	Delay1uS(50);
	SIOWrite(flowSel, ADNS3080_SHUTTER_MAX_BOUND_UPPER, val.b[1]);
	Delay1uS(50);

	// larger delay
	Delay1mS(50);

	// need to update frame period to cause shutter value to take effect
	val.b[1] = SIORead(flowSel, ADNS3080_FRAME_PERIOD_UPPER);
	Delay1uS(50);
	val.b[0] = SIORead(flowSel, ADNS3080_FRAME_PERIOD_LOWER);
	Delay1uS(50);
	SIOWrite(flowSel, ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER, val.b[0]);
	Delay1uS(50);
	SIOWrite(flowSel, ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER, val.b[1]);
	Delay1uS(50);
} // adns3080set_shutter_speed


void adns3080clear_motion(void) {

	SIOWrite(flowSel, ADNS3080_MOTION_CLEAR, 0xff); // clear regs
	x = y = dx = dy = 0;
	motion = false;
} // adns3080clear_motion


void adns3080print_pixel_data(uint8 s) {
	int16 i, j;
	boolean isFirstPixel = true;
	uint8 r;
	uint8 pixelValue;

	// write to frame capture register to force capture of frame
	SIOWrite(flowSel, ADNS3080_FRAME_CAPTURE, 0x83);

	// wait 3 frame periods + 10 nanoseconds for frame to be captured
	Delay1uS(1510); // min frame speed is 2000 frames/second so 1 frame = 500 nano seconds.  so 500 x 3 + 10 = 1510

	// display the pixel data
	for (i = 0; i < ADNS3080_PIXELS_Y; i++) {
		for (j = 0; j < ADNS3080_PIXELS_X; j++) {
			r = SIORead(flowSel, ADNS3080_FRAME_CAPTURE);
			if (isFirstPixel && (r & 0x40) == 0)
			TxString(s, "failed to find first pixel\r\n");

			isFirstPixel = false;
			pixelValue = (r << 2);
			TxVal32(s, pixelValue, 0, 0);
			if (j != ADNS3080_PIXELS_X - 1)
			TxString(s, ",");
			Delay1uS(50);
		}
		TxString(s, "\r\n");
	}

	// hardware reset to restore sensor to normal operation
	adns3080reset();

} // adns3080print_pixel_data

boolean adns3080init(void) {

	_orientation = 0; // ROTATION_NONE;

	adns3080update_conversion_factors();

	return true; // just return true by default
} // adns3080init

// set_orientation - Rotation vector to transform sensor readings to the body frame.
void adns3080set_orientation(real32 rotation) {
	_orientation = rotation;
}

//________________________________________________________________________________________


// call at 20Hz
void adns3080update_position(real32 roll, real32 pitch, real32 cos_yaw_x,
		real32 sin_yaw_y, real32 altitude) {
	uint32 NowmS;
	real32 exp_change_x, exp_change_y;
	real32 change_x, change_y;
	real32 x_cm, y_cm;

	static real32 _last_roll = 0.0f;
	static real32 _last_pitch = 0.0f;
	static real32 _last_altitude = 0.0f;

	if (true) { // have optical
		NowmS = mSClock();
		if (NowmS >= mS[OpticalUpdatemS]) {
			mSTimer(OpticalUpdatemS, OPTICAL_TIME_MS);

			real32 diff_roll = roll - _last_roll;
			real32 diff_pitch = pitch - _last_pitch;

			adns3080update(NowmS);

			// only update position if surface quality is good and angle is not over 45 degrees
			if (surface_quality >= 10 && fabs(roll) <= DegreesToRadians(45)
					&& fabs(pitch) <= DegreesToRadians(45)) {

				altitude = Max(altitude, 0);
				// calculate expected x,y diff due to roll and pitch change
				exp_change_x = diff_roll * radians_to_pixels;
				exp_change_y = -diff_pitch * radians_to_pixels;

				// real estimated raw change from mouse
				change_x = dx - exp_change_x;
				change_y = dy - exp_change_y;

				real32 avg_altitude = (altitude + _last_altitude) * 0.5;

				// convert raw change to horizontal movement in cm
				x_cm = -change_x * avg_altitude * conv_factor; // perhaps this altitude should actually be the distance to the ground?  i.e. if we are very rolled over it should be longer?
				y_cm = -change_y * avg_altitude * conv_factor; // for example if you are leaned over at 45 deg the ground will appear farther away and motion from opt flow sensor will be less

				// convert x/y movements into lon/lat movement
				vlon = x_cm * sin_yaw_y + y_cm * cos_yaw_x;
				vlat = y_cm * sin_yaw_y - x_cm * cos_yaw_x;
			}

			_last_altitude = altitude;
			_last_roll = roll;
			_last_pitch = pitch;
		}
	}

} // adns3080update_position

#elif defined(USE_SYMA_FPV_CAM)

void SymaFPVCamreset(void) {

	if (PWMPins[Aux2Sel].Used) {
	DigitalWrite(&PWMPins[Aux2Sel].P, true); // reset sensor
	Delay1uS(1);
	DigitalWrite(&PWMPins[Aux2Sel].P, false); // return sensor to normal
	Delay1uS(4);
	DigitalWrite(&PWMPins[Aux2Sel].P, true); // return sensor to normal
	}
} // adns3080reset

#endif
