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
void CheckParametersInitialised(void);
void InitParameters(void);
uint8 P(uint8 i);
void SetP(uint8 i, uint8 v);

enum RCControls {
	ThrottleRC,
	RollRC,
	PitchRC,
	YawRC,
	RTHRC,
	RateControlRC,
	NavGainRC,
	BypassRC,
	CamPitchRC,
	TuneRC,
	TransitionRC,
	ArmRC,
	NullRC
};

enum TuneParams {
	NoTuning,
	RollAngleP,
	RollAngleI,
	RollRateP,
	RollRateD,
	PitchAngleP,
	PitchAngleI,
	PitchRateP,
	PitchlRateD,
	YawAngleP,
	YawRateP,
	AltitudeErrorP,
	AltitudeErrorI,
	AltitudeROCP,
	AltitudeROCD,
	// TODO: should add all Nav PID params?
	NavPositionP,
	NavPositionI,
	NavVelP,
	NavCrossTrack
};

enum ArmingModes {
	YawStickArming, SwitchArming, RollStickArming, TxSwitchArming
};

enum ComboPort1Types {
	CPPM_GPS_M7to10,
	FutabaSBus_M7to10,
	ParallelPPM,
	Deltang1024_M7to10,
	Spektrum1024_M7to10,
	Spektrum2048_M7to10,
	BadDM9_M7to10,
	ComboPort1ConfigUnknown
};

enum ComboPort2Types {
	I2C_RF_BatV_V3, I2C_RF_V4, GPS_RF_V4, Unused_RF_V4, ComboPort2Unused
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
	RudderElevatorAF,
	VTOLAF,
	GimbalAF,
	Instrumentation,
	IREmulation,
	AFUnknown,
};

enum Params { // MAX 64
	RollRateKp, // 01
	AltPosKi, // 02
	RollAngleKp, // 03
	ArmingMode, // 04
	RollIntLimit, // 05
	PitchRateKp, // 06
	AltPosKp, // 07
	PitchAngleKp, // 08
	RFSensorType, // 09
	PitchIntLimit, // 10

	YawRateKp, // 11
	RollRateKd, // 12
	StateEst, // 13
	AltVelKd, // 14
	ComboPort1Config, // 15
	Config1Bits, // 16
	RxThrottleCh, // 17
	LowVoltThres, // 18
	RollCamKp, // 19
	EstCruiseThr, // 20

	StickHysteresis, // 21
	AltCompDecayTime, //  22
	PercentIdleThr, // 23
	RollAngleKi, //  24
	PitchAngleKi, //  25
	PitchCamKp, // 26
	Unused27, // 27
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
	MaxAltHoldComp, // 67
	FWMaxClimbAngle, // 68
	NavMaxAngle, // 69
	FWSpoilerDecayTime, // 70
	FWAileronDifferential, // 71
	ASSensorType, // 72,
	BestROC, // 73,
	Config2Bits, // 74
	MaxPitchAngle, // 75
	ComboPort2Config, // 76
	MaxRollAngle, // 77
	Unused78, // 78
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
	PIDTimeSel, // 92
	ThrottleGainRate, // 93
	RxAux5Ch,
	RxAux6Ch,
	RxAux7Ch
};

typedef struct {
	uint8 tag;
	uint8 p[NO_OF_PARAM_SETS];
} ParamStruct_t;

// Config1
#define UseInvertMagMask 		0x01
#define	UseRTHDescendMask		(1<<1)
#define UseManualAltHoldMask 	(1<<2)
#define EmulationEnableMask		(1<<3)
#define GPSToLaunchRequiredMask (1<<4)
#define	UseGPSAltMask			(1<<5)
#define	UseRapidDescentMask		(1<<6)

// Config2
#define UseSWFiltersMask			 0x01
#define	UseFastStartMask		(1<<1)
#define UseBLHeliMask 			(1<<2)
#define UseGliderStrategyMask	(1<<3)
#define UseConfigRebootMask		(1<<4)
#define	UseTurnToWPMask			(1<<5)
#define	UseSpecialMask			(1<<6)

// bit 7 unusable in UAVPSet

// In Servo Sense Byte
#define	UseConvPropSenseMask			(1<<6)

extern volatile boolean StickArmed, TxSwitchArmed;
extern const ParamStruct_t DefaultParams[];
extern const uint8 NoDefaultEntries;
extern const uint16 ESCLimits[];
extern int8 CP[];

extern const real32 AFOrientation[];
extern uint8 UAVXAirframe;
extern boolean IsMulticopter, IsFixedWing, UsingFastStart,
		UsingBLHeliPrograming, UsingGliderStrategy, UsingSpecial;

extern real32 AltCompDecayS;
extern boolean UseFastStart;

#endif

