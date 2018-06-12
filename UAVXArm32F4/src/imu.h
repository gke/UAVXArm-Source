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


#ifndef _gyrosandaccelerometers_h
#define _gyrosandaccelerometers_h

enum GyroTypes {
	MLX90609Gyro,
	ADXRS150Gyro,
	LY530Gyro,
	ADXRS300Gyro,
	UAVXArm32IMU,
	FreeIMU,
	InfraRedAngle,
	GyroUnknown
};

#define DEF_ACC_SCALE (GRAVITY_MPS_S / MPU_1G)

void InertialTest(uint8 s);

void ErectGyros(uint8 imuSel, int32 d);

#define ACC_TRIM_STEP 5 // was 20 - too coarse

void InitIMU(uint8 imuSel);
void ScaleRateAndAcc(uint8 imuSel);

extern const uint8 MPUMap[];
extern const real32 MPUSign[];

extern real32 GyroBias[];
extern const real32 GyroScale[];
extern uint8 CurrAttSensorType;
extern real32 Acc[], Rate[];
extern real32 RateEnergySum;
extern uint32 RateEnergySamples;
extern boolean CaptureIMUBias;

#endif

