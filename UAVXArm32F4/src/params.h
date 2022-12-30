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

// Aircraft Specific

#define BEST_ROC_MPS_FW 3.0
#define AS_MIN_MPS 8.0f
#define AS_MAX_MPS 14.0f
#define POLAR_CD0 0.027f
#define POLAR_B 0.031f
#define POLAR_K 25.6f

#define OUT_MAXIMUM			1.0f
#define OUT_NEUTRAL			(OUT_MAXIMUM*0.5f)

#define DEFAULT_HOME_LAT  (-352902889L) // Canberra
#define DEFAULT_HOME_LON  (1491109972L)

//#define DEFAULT_HOME_LAT  (-374397722L) // Hollowback
//#define DEFAULT_HOME_LON  (1438189666L)

#define DEFAULT_LON_CORR cosf(DegreesToRadians(DEFAULT_HOME_LAT*1e-7))

#define RC_MAXIMUM 1.0f
#define RC_NEUTRAL 0.5f

#define PID_CYCLE_1000US 1000
#define PID_CYCLE_2000US 2000
#define PID_SYNCPWM_CYCLE_2050US 2050 //2500 // for synchronised standard PWM

#define MAX_NOISE_BANDS 8


#define ARMED_TIMEOUT_MS 120000L // mS. automatic disarming if armed for this long and landed
#define NAV_LAND_TIMEOUT_MS 3000
#define NAV_ACTIVE_DELAY_MS 10000 // mS. after throttle exceeds idle that Nav becomes active
#define CRASHED_TIMEOUT_MS 2000 // aircraft is not falling and the attitude is greater than max bank

#define WARMUP_TIMEOUT_MS 2000 // 5000 // mS. time for Madgwick code to settle - empirical!

#define MAG_MIDDLE_STICK FromPercent(10) // yaw stick neutral dead zone
#define MAG_TIME_MS 15 // 75Hz
#define MAG_MAX_SLEW_RAD_S  (DegreesToRadians(720.0f))
#define COMPASS_TIME_MS		50			// 20Hz

#define MAG_CAL_BINS 8
#define MAG_CAL_BIN_SAMPLES 50
#define MAG_CAL_BIN_MAX (MAG_CAL_BIN_SAMPLES+5)
#define MAG_CAL_SAMPLES (MAG_CAL_BINS*MAG_CAL_BIN_SAMPLES)

#define ALT_UPDATE_HZ 100
#define ALT_UPDATE_US (1000000/ALT_UPDATE_HZ)
#define ALT_LPF_MAX_HZ		(3.0f)
#define ALT_MAX_SLEW_M			(20.0f) // TODO: zzz too high
#define ALT_SANITY_SLEW_M		(ALT_MAX_SLEW_M * 2.0f ) // 0.75f)
#define ALT_ROC_LPF_HZ 			(0.2f)
#define ALT_ROC_THRESHOLD_MPS 	(0.03f)
#define ALT_HOLD_DECAY_S		(3.0f)

#define DEFAULT_BARO_VARIANCE	(0.2f)
#define DEFAULT_ACCU_VARIANCE	(2.0f)

#define ALT_MIN_DESCENT_DMPS 	(4)
#define ALT_MAX_DESCENT_DMPS	(20)

#define ALT_MIN_DESCENT_MPS 	(ALT_MIN_DESCENT_DMPS * 0.1f)
#define ALT_MAX_DESCENT_MPS		(ALT_MAX_DESCENT_DMPS * 0.1f)

#define OPTICAL_TIME_MS			50

#define DESCENT_RADIUS_M  			7.0f
#define DESCENT_ORBIT_VELOCITY_MPS	1.5f
#define DEF_FW_LOITER_RADIUS_M 	60.0f

#define DESCENT_ALT_DIFF_M			(25.0f) // 10.0f
#define DESCENT_SAFETY_ALT_M		(25.0f) //(15.0f)

#define ACCU_LANDING_MPS_S			(0.5f * GRAVITY_MPS_S)

#define GYRO_MAX_SHAKE_RAW (DegreesToRadians(1.0)/GyroScale)

#define THR_START_PW FromPercent(5)

#define THR_MIN_ALT_HOLD_STICK FromPercent(20) // min throttle stick for altitude lock
#define THR_MAX_ALT_HOLD_STICK FromPercent(80)

#define BATTERY_SAG_VPS 0.01f

#define THRESHOLD_STICK FromPercent(1) // minimum for roll/pitch
#define THR_LOW_DELAY_MS 2500 // mS. that motor runs at idle after the throttle is closed

#define MAX_ANGLE_DEG 60
#define MAX_ANGLE_RAD DegreesToRadians(MAX_ANGLE_DEG)
#define CRASHED_ANGLE_RAD (MAX_ANGLE_RAD+DegreesToRadians(10))

#define ALT_MAX_ROC_MPS 5.0f

#define NAV_MAX_FENCE_SEGMENTS 10
#define NAV_CEILING_M 120.0f // 400 feet
#define NAV_DEFAULT_RTH_M 15.0f
#define NAV_DEFAULT_FENCE_M 400.0f
#define NAV_MIN_HOME_PROXIMITY_M 10.0f // don't report where information closer than this

#define NAV_MAX_ANGLE_RAD DegreesToRadians(35)

#define NAV_CORR_DECAY 2.0f	// decay to zero /S of nav corrections

#define NAV_SENS_THRESHOLD_STICK FromPercent(20)// No GPS Nav if Ch7 is less than this
#define NAV_ALT_THRESHOLD_STICK FromPercent(10)// No Alt Hold if Ch7 is less than this

#define NAV_LAND_M 5.0f // altitude below which motor shutoff armed for autoland
#define NAV_MIN_ALT_M 5.0f // minimum altitude after takeoff before going to next WP
#define GPS_TIMEOUT_MS 2000 // mS. TODO: too short for forced landing trigger?
#define GPS_MIN_SATELLITES 6 // preferably > 5 for 3D fix
#define GPS_MIN_HACC 3.0f // was 5
#define GPS_MIN_VACC 5.0f
#define GPS_MIN_SACC 1.0f
#define GPS_MIN_CACC 1.0f // deg
#define GPS_HDOP_TO_HACC 4.0f // crude approximation for NMEA GPS units

#define RC_MOVEMENT_STICK FromPercent(1) // minimum to be recognised as a stick input change without triggering failsafe

#define THR_UPDATE_MS 250
#define THR_UPDATE_S ((real32)THR_UPDATE_MS*0.001f)

#define NAV_SENS_ALT_THRESHOLD_STICK FromPercent(10)// Altitude hold disabled if Ch7 is less than this
#define ATTITUDE_HOLD_LIMIT_STICK FromPercent(20) // dead zone for roll/pitch stick for position hold
#define ATTITUDE_THRESHOLD_STICK FromPercent(2) // make sure neutral is 1500uS with no trims
#define ATTITUDE_HOLD_RESET_INTERVAL 25 // number of impulse cycles before GPS position is re-acquired

#define UAVX_TEL_INTERVAL_MS 500 // 200 // mS. emit an interleaved telemetry packet
#define FRSKY_TEL_INTERVAL_MS 125
#define FUSION_TEL_INTERVAL_MS 125

extern const real32 OKp, OIL, IKp, IKd;

void RegeneratePIDCoeffs(void);
void UpdateParameters(void);
void UseDefaultParameters(uint8 DefaultPS);
void DoStickProgramming(void);
void LoadParameters(void);
void ConditionParameters(void);

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
	NavQualificationRC,
	Aux1CamPitchRC,
	Aux2RC,
	TransitionRC,
	PassThruRC,
	Unused11RC,
	NullRC
};

enum ArmingModes {
	TxArming, SwitchArming, UnusedArming2, UnusedArming3
};

enum RxTypes {
	CPPMRx,
	FutabaSBusRx,
	Spektrum1024Rx,
	Spektrum2048Rx,
	CRSFRx,
	UnknownRx
};

enum AFs {
	TriAF,
	TriCoaxAF, // aka Y6
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
	BiAF,
	ElevonAF,
	DeltaAF,
	AileronAF,
	AileronSpoilerFlapsAF,
	AileronVTailAF,
	RudderElevatorAF,
	DifferentialTwinAF,

	VTOLAF,
	VTOL2AF,
	TrackedAF,
	FourWheelAF,
	TwoWheelAF,
	GimbalAF,
	Instrumentation,
	AFUnknown,
};

typedef const struct {
	char AFName[32];
	uint8 P[MAX_PARAMETERS];
} ParamStruct;

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
	IMUFiltType, // 13
	BBLogType, // 14
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
	UnusedSensorHint, // 35 UAVXPIC only
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
	FWSpoilerDecayPercentPS, // 70
	FWAileronDifferential, // 71
	ASSensorType, // 72,
	KFAccUBiasVar, // 73,
	Config2Bits, // 74
	MaxPitchAngle, // 75
	Unused76, // 76
	MaxRollAngle, // 77
	YawLPFHz, // 78
	NavHeadingTurnout, // 79
	AltHoldThrCompDecayPercentPS, // 80
	MinhAcc, // 81
	FWBoardPitchAngle, // 82,
	MaxRollRate, // 83
	MaxPitchRate, // 84
	CurrentScaleTrim, // 0.01 85
	VoltScaleTrim, // 0.01 86
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
	AltThrottleCompLimit, // 103
	VRSDescentRate, // 104
	Unused105, // 105
	AHROCWindowMPS, // 106
	NavProxAltM, // 107
	NavProxRadiusM, // 108
	CurrentSensorFS, // 109
	VoltageSensorFS, // 110

	KFBaroVar, // 111
	KFAccUVar, // 112
	FWStickScale, // 113
	FWRollControlPitchLimit, // 114
	AHThrottleMovingTrigger, // 115
	NavFenceRadiusM, // 116
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
	PowerResetCause
// 128
};


// Config1
#define UseInvertMagMask 		0x01   // bit01CheckBox 16_0
#define	UseRTHDescendMask		(1<<1) // bit11CheckBox 16_1
#define DisableLEDsInFlightMask (1<<2) // bit21CheckBox 16_2
#define EmulationEnableMask		(1<<3) // bit31CheckBox 16_3
#define UseAltHoldAlarmMask 	(1<<4) // bit41CheckBox 16_4
#define	UseOffsetHomeMask		(1<<5) // bit51CheckBox 16_5
#define	UseRapidDescentMask		(1<<6) // bit61CheckBox 16_6
// bit 7 unusable in UAVPSet

// Config2
#define UseBatteryCompMask		0x01   // bit02CheckBox 74_1
#define	UseFastStartMask		(1<<1) // bit12CheckBox 74_2
#define UseBLHeliMask 			(1<<2) // bit22CheckBox 74_3
#define UseGliderStrategyMask	(1<<3) // bit32CheckBox 74_4
#define UsePropSenseMask		(1<<4) // bit42CheckBox
#define	UseTurnToWPMask			(1<<5)
#define	UseNavBeepMask			(1<<6) // bit32CheckBox 74_7
// bit 7 unusable in UAVPSet

extern volatile boolean TxSwitchArmed;

extern ParamStruct DefaultParams[];
extern const uint8 NoOfDefParamSets;
extern boolean ConfigChanged;

extern const uint16 ESCLimits[];
extern int8 CP[];

extern const real32 AFOrientation[];
extern uint8 UAVXAirframe;
extern boolean IsMulticopter, IsGroundVehicle, UsingBatteryComp, UsingNavBeep, UsingFastStart, UsingBLHeliPrograming,
		UsingCruiseCentering, UsingGliderStrategy, DisablingLEDsInFlight;
extern uint8 CurrMotorStopSel;

extern real32 AltHoldThrCompDecayPS;
extern boolean UseFastStart;

extern ConfigStruct Config;

#endif

