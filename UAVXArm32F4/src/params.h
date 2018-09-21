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


#ifndef _params_h
#define _params_h

extern const real32 OKp, OIL, IKp, IKd;

void RegeneratePIDCoeffs(void);
void UpdateParameters(void);
void UseDefaultParameters(uint8 DefaultPS);
void DoStickProgramming(void);
void LoadParameters(void);
void CheckParameters(void);
uint8 P(uint8 i);
void SetP(uint8 i, uint8 v);
uint8 LimitP(uint8 i, uint8 l, uint8 h);

enum RCControls {
	ThrottleRC,
	RollRC,
	PitchRC,
	YawRC,
	NavModeRC,
	AttitudeModeRC,
	NavGainRC,
	BypassRC,
	CamPitchRC,
	Unused10RC,
	TransitionRC,
	ArmRC,
	NullRC
};

enum ArmingModes {
	YawStickArming, SwitchArming, RollStickArming, TxSwitchArming
};

enum RxTypes {
	CPPMRx,
	FutabaSBusRx,
	ParallelPPMRx,
	Deltang1024Rx,
	Spektrum1024Rx,
	Spektrum2048Rx,
	FrSkyFBusRx,
	UnknownRx
};

enum AFs {
	TriAF, TriCoaxAF, // aka Y6
	VTailAF,
	QuadAF,
	QuadXAF,
	QuadCoaxAF, // aka OctCoax
	QuadCoaxXAF,
	HexAF,
	HexXAF,
	OctAF,
	OctXAF,
	Heli90AF,
	Heli120AF,
	ElevonAF,
	DeltaAF,
	AileronAF,
	AileronSpoilerFlapsAF,
	AileronVTailAF,
	RudderElevatorAF,
	VTOLAF,
	VTOL2AF,
	GimbalAF,
	Instrumentation,
	IREmulation,
	AFUnknown,
};


enum Params { // MAX 128
	RollRateKp, // 01
	AltPosKi, // 02
	RollAngleKp, // 03
	ArmingMode, // 04
	RollAngleIntLimit, // 05
	PitchRateKp, // 06
	AltPosKp, // 07
	PitchAngleKp, // 08
	RFSensorType, // 09
	PitchAngleIntLimit, // 10

	YawRateKp, // 11
	RollRateKd, // 12
	IMUOption, // 13
	AltVelKd, // 14
	RxType, // 15
	Config1Bits, // 16
	RxThrottleCh, // 17
	LowVoltThres, // 18
	RollCamKp, // 19
	EstCruiseThr, // 20

	StickHysteresis, // 21
	FWClimbThrottle, //  22
	PercentIdleThr, // 23
	RollAngleKi, //  24
	PitchAngleKi, //  25
	PitchCamKp, // 26
	ServoLPFHz, // 27
	PitchRateKd, // 28
	NavVelKp, // 29
	AltVelKp, // 30

	Horizon, // 31
	MadgwickKpMag, //Acro, // 32
	NavRTHAlt, // 33
	NavMagVar, // 34
	SensorHint, // 35 UAVXPIC only
	ESCType, // 36
	RCChannels, // 37
	RxRollCh, // 38
	MadgwickKpAcc, // 39
	RollCamTrim, // 40

	NavPosIntLimit, // 41
	RxPitchCh, // 42
	RxYawCh, // 43
	AFType, // 44
	TelemetryType, // 45
	MaxDescentRateDmpS, // 46
	DescentDelayS, // 47
	GyroLPFSel, // 48
	NavCrossTrackKp, // 49
	RxGearCh, // 50

	RxAux1Ch, // 51
	ServoSense, // 52
	AccConfSD, // 53
	BatteryCapacity, // 0.1 AH 54
	RxAux2Ch, // 55
	RxAux3Ch, // 56
	NavPosKp, // 57
	AltLPF, // 58
	Balance, // 59
	RxAux4Ch, // 60

	NavPosKi, // 61
	GPSProtocol, // 62
	TiltThrottleFF, // 63
	MaxYawRate, // 64

	FWRollPitchFF, // 65
	FWPitchThrottleFF, // 66
	AltVelIntLimit, // 67
	FWMaxClimbAngle, // 68
	NavMaxAngle, // 69
	FWSpoilerDecayTime, // 70
	FWAileronDifferential, // 71
	ASSensorType, // 72,
	MaxROC, // 73,
	Config2Bits, // 74
	MaxPitchAngle, // 75
	Unused76, // 76
	MaxRollAngle, // 77
	YawLPFHz, // 78
	NavHeadingTurnout, // 79
	WS2812Leds, // 80
	MinhAcc, // 81
	FWBoardPitchAngle, // 82,
	MaxRollRate, // 83
	MaxPitchRate, // 84
	CurrentScale, // 0.01 85
	VoltScale, // 0.01 86
	FWAileronRudderMix, // 87
	FWAltSpoilerFF, // 88
	MaxCompassYawRate, // 89
	AccLPFSel, //P90,
	YawRateKd, // 91,
	GyroSlewRate, // 92
	ThrottleGainRate, // 93
	RxAux5Ch, // 94
	RxAux6Ch, // 95
	RxAux7Ch, // 96
	YawAngleKp, // 97
	YawAngleKi, // 98
	YawAngleIntLimit, // 99

	//
	AltPosIntLimit, // 100
	MotorStopSel, // 101
	AltVelKi, // 102
	AltHoldBand, // 103
	VRSDescentRate, // 104
	OSLPFType, // 105
	OSLPFHz, // 106
	Unused107, // 107
	Unused108, // 108
	Unused109, // 109
	Unused110, // 110

	Unused111, // 111
	Unused112, // 112
	Unused113, // 113
	Unused114, // 114
	Unused115, // 115
	Unused116, // 116
	Unused117, // 117
	Unused118, // 118
	Unused119, // 119
	Unused120, // 120
	Unused121, // 121
	Unused122, // 122
	Unused123, // 123
	Unused124, // 124
	Unused125, // 125
	Unused126, // 126
	Unused127, // 127
	Unused128
// 128
};

typedef struct {
	uint8 tag;
	uint8 min;
	uint8 max;
	uint8 p[NO_OF_PARAM_SETS];
} ParamStruct_t;

// Config1
#define UseInvertMagMask 		0x01   // bit01CheckBox 16_0
#define	UseRTHDescendMask		(1<<1) // bit11CheckBox 16_1
#define UseManualAltHoldMask 	(1<<2) // bit21CheckBox 16_2
#define EmulationEnableMask		(1<<3) // bit31CheckBox 16_3
#define GPSToLaunchRequiredMask (1<<4) // bit41CheckBox 16_4
#define	UseGPSAltMask			(1<<5) // bit51CheckBox 16_5
#define	UseRapidDescentMask		(1<<6) // bit61CheckBox 16_6

// Config2
#define UsePavelFilterMask		0x01   // bit02CheckBox 74_1
#define	UseFastStartMask		(1<<1) // bit12CheckBox 74_2
#define UseBLHeliMask 			(1<<2) // bit22CheckBox 74_3
#define UseGliderStrategyMask	(1<<3) // bit32CheckBox 74_4
#define UseGyroOSMask			(1<<4) // bit42CheckBox
#define	UseTurnToWPMask			(1<<5)
#define	UnusedUseHWLPFMask		(1<<6) // bit32CheckBox 74_7

// bit 7 unusable in UAVPSet

// In Servo Sense Byte
#define	UseConvYawSenseMask			(1<<6)

extern volatile boolean StickArmed, TxSwitchArmed;
extern const ParamStruct_t DefaultParams[];
extern const uint8 NoDefaultEntries;
extern const uint16 ESCLimits[];
extern int8 CP[];

extern const real32 AFOrientation[];
extern uint8 UAVXAirframe;
extern boolean IsMulticopter, UsingFastStart, UsingBLHeliPrograming,
		UsingGliderStrategy;
extern uint8 CurrMotorStopSel;

extern real32 AltCompDecayS;
extern boolean UseFastStart;

#endif

