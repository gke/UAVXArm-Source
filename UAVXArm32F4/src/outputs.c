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

#define ONESHOT125_TIMER_MHZ  8
#define ONESHOT42_TIMER_MHZ   24
#define MULTISHOT_TIMER_MHZ   TIMER_PS // 72
#define PWM_BRUSHED_TIMER_MHZ 24

#define MULTISHOT_5US_PW    (TIMER_PS * 5)
#define MULTISHOT_20US_MULT (TIMER_PS * 20 / 1000.0f)

const uint8 DrivesUsed[AFUnknown + 1] = { 3, 6, 4, 4, 4, 8, 8, 6, 6, 8, 8, // TriAF, TriCoaxAF, VTailAF, QuadAF, QuadXAF, QuadCoaxAF, QuadCoaxXAF, HexAF, HexXAF, OctAF, OctXAF
		1, 2, // Heli90AF, BiAF,
		2, 2, 2, 2, 2, 1, // ElevonAF, DeltaAF, AileronAF, AileronSpoilerFlapsAF, AileronVTailAF, RudderElevatorAF,
		2, // DifferentailTwinAF,
		2, 2, // VTOLAF, VTOL2AF,
		2, 2, 2, // TrackedAF, FourWheelAF, TwoWheelAF,
		0, // GimbalAF,
		0, // Instrumentation,
		0 }; // AFUnknown,

const idx DM[10] = { 0, 1, 2, 3, // TIM4
		6, 7, 8, 9, // TIM3
		4, 5 }; // TIM1 V4 TIM8 camera servo channels always last

boolean UsingDCMotors = false;
boolean DrivesInitialised = false;

real32 FWStickScaleFrac = 0.33f;
real32 LPF1DriveK, LPF1ServoK;
uint8 CurrESCType = ESCUnknown;
real32 PWSum[MAX_PWM_OUTPUTS];
int8 PWDiagnostic[MAX_PWM_OUTPUTS];
real32 MultiPropSense = 1.0f;
uint32 PWSamples;
real32 PW[MAX_PWM_OUTPUTS];
real32 PWp[MAX_PWM_OUTPUTS];
idx NoOfDrives = 4;
real32 NoOfDrivesR;

real32 Rl, Pl, Yl, Sl;
idx CurrMaxPWMOutputs = 6;

CamStruct Cam;

real32 DFT[8];

const char * ESCName[] = { "PWM", "DC Motor", "OneShot", "MultiShot", "DShot",
		"Unknown" };

void ShowESCType(uint8 s) {
	TxString(s, ESCName[CurrESCType]);
} // ShowESCType

void driveWrite(idx channel, real32 v) {

	if (DM[channel] < CurrMaxPWMOutputs) {
		v = Limit((int16)((v + 1.0f) * 0.5f * PWM_MAX), PWM_MIN, PWM_MAX);
		*PWMPins[DM[channel]].Timer.CCR = v;
	}
} // driveWrite

void servoWrite(idx channel, real32 v) {

	if (DM[channel] < CurrMaxPWMOutputs) {
		v = Limit((int16 )((v + 1.0f) * 1000.0f), PWM_MIN_SERVO, PWM_MAX_SERVO);
		*PWMPins[DM[channel]].Timer.CCR = v;
	}
} // servoWrite

void driveSyncStart(uint8 drives) {

	for (idx m = 0; m < drives; m++)
		if (m < CurrMaxPWMOutputs)
			TIM_Cmd(PWMPins[DM[m]].Timer.Tim, ENABLE);

} // driveSyncStart

void driveDCWrite(idx channel, real32 v) {

	if (DM[channel] < CurrMaxPWMOutputs) {
		v = Limit((int16 )(v * 1000.0f), PWM_MIN_DC, PWM_MAX_DC);
		*PWMPins[DM[channel]].Timer.CCR = v;
	}
} // driveDCWrite

typedef void (*driveWriteFuncPtr)(idx channel, real32 value);
static driveWriteFuncPtr driveWritePtr = NULL;

// ESCPWM,  DCMotors, OneShot, MultiShot, DShot, ESCUnknown,

const struct {
	driveWriteFuncPtr driver;
	uint16 prescaler;
	uint32 period;
	uint32 min;
	uint32 max;
} Drive[] = { { driveWrite, PWM_PS, PWM_PERIOD, PWM_MIN, PWM_MAX }, // ESCPWM
		{ driveDCWrite, PWM_PS_DC, PWM_PERIOD_DC, PWM_MIN_DC, PWM_MAX_DC } // DCMotors
};

void UpdateDrives(void) {
	static idx m;

	if (DrivesInitialised) {

		if (F.PassThru && !IsMulticopter) {
			Rl = Limit1(-A[Roll].Stick * FWStickScaleFrac, OUT_NEUTRAL);
			Pl = Limit1(-A[Pitch].Stick * FWStickScaleFrac, OUT_NEUTRAL);
			Yl = Limit1(-A[Yaw].Stick * FWStickScaleFrac, OUT_NEUTRAL);
			Sl = 0.0f; //zzz
		} else {
			Rl = Limit1(A[Roll].Out, OUT_NEUTRAL);
			Pl = Limit1(A[Pitch].Out, OUT_NEUTRAL);
			Yl = Limit1(A[Yaw].Out, OUT_NEUTRAL);
		}

		if (IsMulticopter)
			DoMulticopterMix();
		else if (IsGroundVehicle)
			DoGroundVehicleMix();
		else
			DoMix();

		for (m = 0; m < NoOfDrives; m++) { // drives
			PWp[m] =
					(Armed() && !F.Emulation) ?
							LPF1(PWp[m], PW[m], LPF1DriveK) : 0.0f;
			driveWritePtr(m, PWp[m]);

			PWSum[m] += PWp[m];
		}

		PWSamples++;

		// servos
		MixAndLimitCam();

		for (m = NoOfDrives; m < MAX_PWM_OUTPUTS; m++) {
			PWp[m] = LPF1(PWp[m], PW[m], LPF1ServoK); // 0.25
			servoWrite(m, PWp[m]);
		}
	}

} // UpdateDrives

void InitDrives(void) {
	idx m, nd;

	F.DrivesArmed = false;

	UsingDCMotors = (CurrESCType == DCMotors);

	NoOfDrives = Limit(DrivesUsed[UAVXAirframe], 0, CurrMaxPWMOutputs);

	if (IsMulticopter) {

		NoOfDrivesR = 1.0f / NoOfDrives;
		nd = ((NoOfDrives + 3) / 4) * 4; // Timers share period 4 + 4 + 2

		driveWritePtr = Drive[CurrESCType].driver;

		for (m = 0; m < nd; m++) {
			PW[m] = PWp[m] = 0.0f;

			if (DM[m] < CurrMaxPWMOutputs)
				InitPWMPin(&PWMPins[DM[m]], Drive[CurrESCType].prescaler,
						Drive[CurrESCType].period, Drive[CurrESCType].min);
		}

		// servos
		if (!UsingDCMotors)
			for (m = nd; m < MAX_PWM_OUTPUTS; m++)
				if (DM[m] < CurrMaxPWMOutputs) {
					PW[m] = PWp[m] = OUT_NEUTRAL;
					InitPWMPin(&PWMPins[DM[m]], PWM_PS, PWM_PERIOD_SERVO,
					PWM_NEUTRAL);
				}
	} else {

		driveWritePtr = servoWrite;

		for (m = 0; m < NoOfDrives; m++)
			PW[m] = PWp[m] = 0.0f;

		for (m = NoOfDrives; m < MAX_PWM_OUTPUTS; m++)
			PW[m] = PWp[m] = OUT_NEUTRAL;

		for (m = 0; m < MAX_PWM_OUTPUTS; m++)
			if (DM[m] < CurrMaxPWMOutputs)
				InitPWMPin(&PWMPins[DM[m]], PWM_PS, PWM_PERIOD_SERVO,
				PWM_WIDTH_SERVO);

		for (m = 0; m < MAX_PWM_OUTPUTS; m++)
			servoWrite(m, PWp[m]);

	}

	for (m = 0; m < MAX_PWM_OUTPUTS; m++) {
		PWDiagnostic[m] = 0;
		PWSum[m] = 0.0f;
	}

	PWSamples = 1; // avoid div 0

	InitServoSense();

	DrivesInitialised = true;

} // InitDrives

