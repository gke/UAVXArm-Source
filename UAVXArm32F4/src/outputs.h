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

#ifndef _outputs_h
#define _outputs_h

void InitPWM(void);
void pwmWrite(idx p, real32 pulseWidth, uint16 pulsemin, uint16 pulsemax);

void ShowESCType(uint8 s);

void UpdateDrives(void);
void StopDrives(void);
void InitDrives(void);

void ProgramSlaveAddress(idx s, uint8 addr);
void ConfigureESCs(uint8 s);

void driveWrite(idx channel, real32 v);

enum ESCTypes {
	ESCPWM, DCMotors, ESCUnknown
};

enum PWMTagsQuad {
	FrontC = 0, LeftC, RightC, BackC
};

// order is important for X3D & Holger ESCs
enum PWMTagsVT {
	FrontLeftC = 0, FrontRightC
};

enum PWMTagsTri {
	YawC = 4
};

enum PWMTagsY6 {
	FrontTC = 0, LeftTC, RightTC, FrontBC, LeftBC, RightBC
};

enum PWMTagsHexa {
	HFrontC = 0, HLeftFrontC, HRightFrontC, HLeftBackC, HRightBackC, HBackC
};

enum PWMTagsAileron {
	RightThrottleC,
	LeftThrottleC,
	RightAileronC,
	LeftAileronC,
	ElevatorC,
	RudderC,
	SpoilerC,
	Aux2C,
	CamRollC,
	Aux1CamPitchC
};
enum PWMTagsElevon {
	RightElevonC = RightAileronC, LeftElevonC = LeftAileronC
};

enum PWMTagsVTOL {
	RightPitchYawC = ElevatorC, LeftPitchYawC = RudderC
};

enum PWMTagsVTail {
	RightRudderElevatorC = ElevatorC, LeftRudderElevatorC = RudderC
};

enum PWMBiTags {
	RightTiltServoC = RightAileronC, LeftTiltServoC = LeftAileronC
};

extern uint8 CurrESCType;

extern real32 Rl, Pl, Yl, Sl;

typedef struct {
	real32 RollKp, PitchKp;
} CamStruct;

CamStruct Cam;

extern real32 LPF1DriveK, LPF1ServoK;
extern uint8 const DrivesUsed[];
extern idx NoOfDrives;
extern const idx DM[];
extern real32 PWSum[];
extern int8 PWDiagnostic[];
extern uint32 PWSamples;
extern real32 PW[], PWp[];
extern real32 PWSense[];
extern idx CurrMaxPWMOutputs;

extern real32 DFT[];
extern real32 CGOffset;

extern real32 I2CESCMax;

extern real32 FWStickScaleFrac;
extern boolean UsingPWMSync;
extern boolean UsingDCMotors;
extern boolean DrivesInitialised;
extern real32 NoOfDrivesR;
extern real32 MultiPropSense;

#endif

