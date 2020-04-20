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


#ifndef _uavx_h
#define _uavx_h

// Options

//#define USE_SPI_ESC  // place holder instead of stupid DSHOT disaster
//#define USE_MAG_DIRECT // bypass Madgwick yaw fusion - not recommended

//#define ALT_KF_TESTING  // different altitude filtering schemes
//#define TRACK_ACC_UP_BIAS

//#define USE_AUX3_PROBE_PIN // CAUTION DIAGNOSTIC ONLY: This disables use of Aux3 for WP Nav enable

//#define USE_CONSERVATIVE_DEF_PARAM_LOAD

#define USE_SENSOR_NOISE // for emulation adds noise to acc and baro sensors

//#define USE_THERMALS
#define HAVE_WIND_ESTIMATE

//#define USE_ATT_BATT_COMP

#define NAV_ENFORCE_ALTITUDE_CEILING		// limit all autonomous altitudes
#define CHECK_INVERTED // check multicopter upside down

#define MAX_BLHELI_ESCS 4
//#define USE_OLED
#define USE_WS2812 // shared with Aux1 pulse
//#define USE_WS2812B

//#define INC_STATS_TEL

#define INC_BARO_FULL_MATH
#define VOLT_MEASUREMENT_ONBOARD
//#define INC_TEMPERATURE // ambient temperature sensor

//________________________________________________________________________________________________

#include "UAVXRevision.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "mpu6050.h"

#define ADC_SCALE (1.0f/4096.0f)	// 12bit normalise to 1.0

#define STM32F4
#define TIMER_PS 168
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#include "misctypes.h"

#include "boards/harness.h"

#include "escprog/serial_4way_impl.h"
#include "escprog/serial_4way.h"
#include "escprog/serial_4way_stk.h"
#include "escprog/serial_4way_avr.h"
#include "oled/SSD1X06.h"

#include "main.h"
#include "filters.h"
#include "alarms.h"
#include "analog.h"
#include "armflash.h"
#include "as.h"
#include "altfilt.h"
#include "altitude.h"
#include "batt.h"
#include "bb.h"
#include "calib.h"
#include "clocks.h"
#include "control.h"
#include "auto.h"
#include "emu.h"
#include "frsky.h"
#include "gps.h"
#include "imu.h"
#include "inertial.h"
#include "mpu6xxx.h"
#include "isr.h"
#include "i2c.h"
#include "i2ceeprom.h"
#include "leds.h"
#include "mag.h"
#include "magvar.h"
#include "mission.h"
#include "nav.h"
#include "nvmem.h"
#include "mixer.h"
#include "outputs.h"
#include "params.h"
#include "serial.h"
#include "sio.h"
#include "spi.h"
#include "rc.h"
#include "spiflash.h"
#include "stats.h"
#include "telem.h"
#include "temp.h"
#include "tests.h"
#include "tune.h"
#include "wind.h"

#include "vcp/drivers/drv_usb.h"
#include "vcp/drivers/drv_system.h"
#include "vcp/vcpf4/usbd_cdc_vcp.h"

#include "soar/ekf.h"
#include "soar/soar.h"

#endif

