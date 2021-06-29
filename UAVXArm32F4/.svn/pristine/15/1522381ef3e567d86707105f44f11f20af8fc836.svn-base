#ifndef __ADNS3080_H__
#define __ADNS3080_H__

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


// orientations for ADNS3080 sensor
#define ADNS3080_PINS_FORWARD ROTATION_YAW_180
#define ADNS3080_PINS_FORWARD_RIGHT ROTATION_YAW_135
#define ADNS3080_PINS_RIGHT ROTATION_YAW_90
#define ADNS3080_PINS_BACK_RIGHT ROTATION_YAW_45
#define ADNS3080_PINS_BACK ROTATION_NONE
#define ADNS3080_PINS_BACK_LEFT ROTATION_YAW_315
#define ADNS3080_PINS_LEFT ROTATION_YAW_270
#define ADNS3080_PINS_FORWARD_LEFT ROTATION_YAW_225

// field of view of ADNS3080 sensor lenses
#define ADNS3080_08_FOV DegreesToRadians(11.6)

// scaler - value returned when sensor is moved equivalent of 1 pixel
#define ADNS3080_SCALER  1.1

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X                 30
#define ADNS3080_PIXELS_Y                 30
#define ADNS3080_CLOCK_SPEED              24000000

// Register Map for the ADNS3080 Optical OpticalFlow Sensor
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_REVISION_ID           0x01
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_PIXEL_SUM             0x06
#define ADNS3080_MAXIMUM_PIXEL         0x07
#define ADNS3080_CONFIGURATION_BITS    0x0a
#define ADNS3080_EXTENDED_CONFIG       0x0b
#define ADNS3080_DATA_OUT_LOWER        0x0c
#define ADNS3080_DATA_OUT_UPPER        0x0d
#define ADNS3080_SHUTTER_LOWER         0x0e
#define ADNS3080_SHUTTER_UPPER         0x0f
#define ADNS3080_FRAME_PERIOD_LOWER    0x10
#define ADNS3080_FRAME_PERIOD_UPPER    0x11
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_SROM_ENABLE           0x14
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER      0x19
#define ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER      0x1a
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_LOWER      0x1b
#define ADNS3080_FRAME_PERIOD_MIN_BOUND_UPPER      0x1c
#define ADNS3080_SHUTTER_MAX_BOUND_LOWER           0x1e
#define ADNS3080_SHUTTER_MAX_BOUND_UPPER           0x1e
#define ADNS3080_SROM_ID               0x1f
#define ADNS3080_OBSERVATION           0x3d
#define ADNS3080_INVERSE_PRODUCT_ID    0x3f
#define ADNS3080_PIXEL_BURST           0x40
#define ADNS3080_MOTION_BURST          0x50
#define ADNS3080_SROM_LOAD             0x60

// Configuration Bits
#define ADNS3080_LED_MODE_ALWAYS_ON        0x00
#define ADNS3080_LED_MODE_WHEN_REQUIRED    0x01

#define ADNS3080_RESOLUTION_400            400
#define ADNS3080_RESOLUTION_1600           1600

// Extended Configuration bits
#define ADNS3080_SERIALNPU_OFF  0x02

#define ADNS3080_FRAME_RATE_MAX         6469
#define ADNS3080_FRAME_RATE_MIN         2000

#define NUM_CALLS_FOR_10HZ     100         // timer process runs at 1khz.  100 iterations = 10hz
#define NUM_CALLS_FOR_20HZ     50          // timer process runs at 1khz.  50 iterations = 20hz
#define NUM_CALLS_FOR_50HZ     20          // timer process runs at 1khz.  20 iterations = 50hz

boolean adns3080init(void);
void adns3080update_position(real32 roll, real32 pitch, real32 cos_yaw_x,
		real32 sin_yaw_y, real32 altitude);
void adns3080print_pixel_data(uint8 s);

#endif
