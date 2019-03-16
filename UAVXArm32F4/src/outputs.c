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
		1, 1, // Heli90AF, Heli120AF,
		1, 1, 1, 1, 1, 1, // ElevonAF, DeltaAF, AileronAF, AileronSpoilerFlapsAF, AileronVTailAF, RudderElevatorAF,
		1, 2, 0, // VTOLAF, VTOL2AF, GimbalAF,
		0, 4, // Instrumentation, IREmulation,
		0 }; // AFUnknown,

const uint8 PWMOutputsUsed[AFUnknown + 1] = { 5, 8, 6, 6, 6, 10, 10, 8, 8, 10,
		10, //
		6, 6, //
		3, 6, 7, 5, 6, 4, // inc rudder elev from 3->4 for spoilers
		4, 4, 2, //
		0, 0, //
		0 };

#if defined(UAVXF4V3) || defined(UAVXF4V4)
const idx DM[10] = {0, 1, 2, 3, // TIM4
	6, 7, 8, 9, // TIM3
	4, 5}; // TIM1 V4 TIM8  camera servo channels always last
#else
const idx DM[10] = { 3, 2, 1, 0, // TIM4 ROBERT needs reordering for MW convention
		6, 7, 8, 9, // TIM3
		4, 5 }; // TIM1 V4 TIM8  camera servo channels always last
#endif

typedef struct {
	unsigned int cmd :1;
	unsigned int c :3;
	unsigned int v :12;
}__attribute__((packed)) SPIESCChannelStruct_t;

boolean UsingPWMSync = false;
boolean UsingDCMotors = false;
boolean DrivesInitialised = false;

real32 LPF1DriveK, LPF1ServoK;
uint8 CurrESCType = ESCUnknown;
real32 PWSum[MAX_PWM_OUTPUTS];
int8 PWDiagnostic[MAX_PWM_OUTPUTS];
uint32 PWSamples;
real32 PW[MAX_PWM_OUTPUTS];
real32 PWp[MAX_PWM_OUTPUTS];
idx NoOfDrives = 4;
real32 NoOfDrivesR;
uint32 ESCI2CFail[256] = { 0 };
SPIESCChannelStruct_t SPIESCFrame[MAX_PWM_OUTPUTS];

real32 Rl, Pl, Yl, Sl;
real32 I2CESCMax;
idx CurrMaxPWMOutputs = 6;

CamStruct Cam;

real32 DFT[8];

const char * ESCName[] = { "PWM", "PWMSync", "PWMSyncDiv8 or OneShot", "I2C",
		"DC Motor", "DC Motor Slow Idle", "SPI", "ADC Angle", "Unknown" };

void ShowESCType(uint8 s) {
	TxString(s, ESCName[CurrESCType]);
} // ShowESCType

void driveWrite(idx channel, real32 v) {

	if (DM[channel] < CurrMaxPWMOutputs) {
		v = Limit((int16)((v + 1.0f) * 1000.0f), PWM_MIN, PWM_MAX);
		*PWMPins[DM[channel]].Timer.CCR = v;
	}
} // driveWrite

void servoWrite(idx channel, real32 v) {

	if (DM[channel] < CurrMaxPWMOutputs) {
		v = Limit((int16)((v + 1.0f) * 1000.0f),
				PWM_MIN_SERVO, PWM_MAX_SERVO);
		*PWMPins[DM[channel]].Timer.CCR = v;
	}
} // servoWrite

void driveSyncWrite(idx channel, real32 v) {
	const PinDef * u;

	if (DM[channel] < CurrMaxPWMOutputs) {
		u = &PWMPins[DM[channel]];
		// repetition for multiple channels on some timer
		TIM_Cmd(u->Timer.Tim, DISABLE);
		TIM_SetCounter(u->Timer.Tim, 0);

		*u->Timer.CCR = Limit((int16)((v + 1.0f) * 1000.0f), PWM_MIN, PWM_MAX);
	}
} // driveSyncWrite

void driveSyncDiv8Write(idx channel, real32 v) {
	const PinDef * u;

	if (DM[channel] < CurrMaxPWMOutputs) {
		u = &PWMPins[DM[channel]];
		// repetition for multiple channels on some timer
		TIM_Cmd(u->Timer.Tim, DISABLE);
		TIM_SetCounter(u->Timer.Tim, 0);

		*u->Timer.CCR = Limit((int16)((v + 1.0f) * 1500.0f),
				PWM_MIN_SYNC_DIV8, PWM_MAX_SYNC_DIV8);
	}
} // driveSyncDiv8Write


void driveSyncStart(uint8 drives) {

	for (idx m = 0; m < drives; m++)
		if (m < CurrMaxPWMOutputs)
			TIM_Cmd(PWMPins[DM[m]].Timer.Tim, ENABLE);

} // driveSyncStart


void driveI2CWrite(idx channel, real32 v) {
	/*
	 if (channel < CurrMaxPWMOutputs)
	 //ESCI2CFail[channel] =
	 SIOWrite(SIOESC, 0x52 + channel * 2, 0, Limit(
	 (uint16)(v * 225.0f),0, 225));
	 */
} // driveI2CWrite

void driveSPIWrite(idx channel, real32 v) {

	if (channel < CurrMaxPWMOutputs) {
		SPIESCFrame[channel].cmd = false;
		SPIESCFrame[channel].c = channel;
		SPIESCFrame[channel].v = Limit((int16)(v * 2048.0f), -2048, 2047);
	}

} // driveSPIWrite

void driveSPISyncStart(uint8 drives) {

	//SIOWrite(SIOESC, 0x52 + channel * 2, 0, Limit(
	//		(uint16)(v * 225.0f),0, 225));

} // driveSPISync


void driveDCWrite(idx channel, real32 v) {

	if (DM[channel] < CurrMaxPWMOutputs) {
		v = Limit((int16)(v * 1000.0f), PWM_MIN_DC, PWM_MAX_DC);
		*PWMPins[DM[channel]].Timer.CCR = v;
	}
} // driveDCWrite

typedef void (*driveWriteFuncPtr)(idx channel, real32 value);
static driveWriteFuncPtr driveWritePtr = NULL;

// ESCPWM, ESCSyncPWM, ESCSyncPWMDiv8, ESCI2C, DCMotors, DCMotorsWithIdle, SPI, IR, ESCUnknown,

const struct {
	driveWriteFuncPtr driver;
	uint16 prescaler;
	uint32 period;
	uint32 min;
	uint32 max;
} Drive[] = { { driveWrite, PWM_PS, PWM_PERIOD, PWM_MIN, PWM_MAX }, // ESCPWM
		{ driveSyncWrite, PWM_PS_SYNC, PWM_PERIOD_SYNC, PWM_MIN_SYNC,
				PWM_MAX_SYNC }, // ESCSyncPWM
		{ driveSyncDiv8Write, PWM_PS_SYNC_DIV8, PWM_PERIOD_SYNC_DIV8,
				PWM_MIN_SYNC_DIV8, PWM_MAX_SYNC_DIV8 }, // ESCSyncPWMDiv8
		{ driveI2CWrite, 0, 0, 0, 225 }, // ESCI2C
		{ driveDCWrite, PWM_PS_DC, PWM_PERIOD_DC, PWM_MIN_DC, PWM_MAX_DC }, // DCMotors
		{ driveDCWrite, PWM_PS_DC, PWM_PERIOD_DC, PWM_MIN_DC, PWM_MAX_DC }, // DCMotorsWithIdle
		{ driveSPIWrite, 0, 0, 0, 1023 }, // ESCSPI
		{ driveDCWrite, PWM_PS_DC, PWM_PERIOD_DC, PWM_MIN_DC, PWM_MAX_DC }, // ADC Angle
		};

void UpdateDrives(void) {
	static idx m;

	if (DrivesInitialised) {
		if (UAVXAirframe == IREmulation) {

			PW[IRRollC] = OUT_NEUTRAL + PWSense[IRRollC] * sinf(Angle[Roll])
					* OUT_NEUTRAL;
			PW[IRPitchC] = OUT_NEUTRAL + PWSense[IRPitchC] * sinf(
					Angle[Pitch]) * OUT_NEUTRAL;
			PW[IRZC] = OUT_NEUTRAL + PWSense[IRZC] * 0.0f; // TODO:

			for (m = 0; m < NoOfDrives; m++)
				driveWritePtr(m, PW[m]);

		} else if (IsMulticopter) {

			Rl = Limit1(A[Roll].Out, OUT_NEUTRAL);
			Pl = Limit1(A[Pitch].Out, OUT_NEUTRAL);
			Yl = Limit1(A[Yaw].Out, OUT_NEUTRAL);

			DoMulticopterMix();
			MixAndLimitCam();

			for (m = 0; m < NoOfDrives; m++) { // drives
				PWp[m] = (Armed() && !F.Emulation) ? LPF1(PWp[m], PW[m],
						LPF1DriveK) : 0.0f;

				driveWritePtr(m, PWp[m]);

				PWSum[m] += PWp[m];
			}

			PWSamples++;

			if (UsingPWMSync)
				driveSyncStart(NoOfDrives);
			else if (CurrESCType == ESCSPI)
				driveSPISyncStart(NoOfDrives);

			// servos
			if (!UsingDCMotors)
				for (m = NoOfDrives; m < MAX_PWM_OUTPUTS; m++) {
					PWp[m] = LPF1(PWp[m], PW[m], LPF1ServoK); // 0.25
					servoWrite(m, PWp[m]);
				}
		} else {

			if (F.PassThru) {
				Rl = -A[Roll].Stick * STICK_PASSTHRU_SCALE;
				Pl = -A[Pitch].Stick * STICK_PASSTHRU_SCALE;
				Yl = A[Yaw].Stick * STICK_PASSTHRU_SCALE;
				Sl = 0.0f; //zzz
			} else {
				Rl = Limit1(A[Roll].Out, OUT_NEUTRAL);
				Pl = Limit1(A[Pitch].Out, OUT_NEUTRAL);
				Yl = Limit1(A[Yaw].Out, OUT_NEUTRAL);
			}

			DoMix();

			for (m = 0; m < NoOfDrives; m++) { // drives
				PWp[m] = (Armed() && !F.Emulation) ? LPF1(PWp[m], PW[m],
						LPF1DriveK) : 0.0f;
				servoWrite(m, PWp[m]);
			}

			for (m = NoOfDrives; m < MAX_PWM_OUTPUTS; m++) { // servos
				PWp[m] = LPF1(PWp[m], PW[m], LPF1ServoK);
				servoWrite(m, PWp[m]);
			}
		}
	}

} // UpdateDrives


void InitDrives(void) {
	idx m, nd;

	F.DrivesArmed = false;

	UsingDCMotors = (CurrESCType == DCMotorsWithIdle) || (CurrESCType
			== DCMotors) || (CurrESCType == PWMDAC);
	UsingPWMSync = (CurrESCType == ESCSyncPWM) || (CurrESCType
			== ESCSyncPWMDiv8);

	NoOfDrives = Limit(DrivesUsed[UAVXAirframe], 0, CurrMaxPWMOutputs);

	if (IsMulticopter || (UAVXAirframe == IREmulation)) {

		NoOfDrivesR = 1.0f / NoOfDrives;
		nd = ((NoOfDrives + 3) / 4) * 4; // Timers share period 4 + 4 + 2

		driveWritePtr = Drive[CurrESCType].driver;

		for (m = 0; m < nd; m++) {
			PW[m] = PWp[m] = 0.0f;

			if ((CurrESCType != ESCI2C) && (CurrESCType != ESCSPI))
				if (DM[m] < CurrMaxPWMOutputs)
					InitPWMPin(&PWMPins[DM[m]], Drive[CurrESCType].prescaler,
							Drive[CurrESCType].period, Drive[CurrESCType].min,
							UsingPWMSync);
		}

		// servos
		if (!UsingDCMotors)
			for (m = nd; m < MAX_PWM_OUTPUTS; m++)
				if (DM[m] < CurrMaxPWMOutputs) {
					PW[m] = PWp[m] = OUT_NEUTRAL;
					InitPWMPin(&PWMPins[DM[m]], PWM_PS, PWM_PERIOD_SERVO,
							PWM_NEUTRAL, false);
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
						PWM_WIDTH_SERVO, false);

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


