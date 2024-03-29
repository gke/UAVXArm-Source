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


#ifndef _main_h
#define _main_h

#define ArmingSwitch (DigitalRead(&GPIOPins[ArmedSel].P))

//________________________________________________________________________________________

#define FLAG_BYTES  12

extern real32 dT, dTR, dTOn2, dTROn2;
extern timeuS CurrPIDCycleuS;
extern real32 CurrPIDCycleS;
extern int16 ArmUtilisationPercent;

enum uSTimes {
	CycleUpdateuS,
	MemReadyuS,
	LastCycleTime,
	BaroUpdateuS,
	AltUpdateuS,
	uSLastArrayEntry
};

enum mSTimes {
	StartTime,
	GeneralCountdown,
	UpdatemSTimeoutmS,
	RCSignalTimeoutmS,
	BeeperTimeoutmS,
	ThrottleIdleTimeoutmS,
	AbortTimeoutmS,
	NavStateTimeoutmS,
	DescentUpdatemS,
	GlidingTimemS,
	LastValidRx,
	LastGPSmS,
	AccTimeoutmS,
	GPSTimeoutmS,
	RxFailsafeTimeoutmS,
	StickChangeUpdatemS,
	StickTimeoutmS,
	LEDChaserUpdatemS,
	LastBatterymS,
	BatteryUpdatemS,
	TelemetryUpdatemS,
	BeeperUpdatemS,
	ArmedTimeoutmS,
	WarmupTimeoutmS,
	BaroUpdatemS,
	ThrottleUpdatemS,
	AltUpdatemS,
	EmuAltUpdatemS,
	NextASUpdatemS,
	FakeGPSUpdatemS,
	GyroTempUpdatemS,
	RangefinderUpdatemS,
	MagnetometerUpdatemS,
	LVCTimeoutmS,
	CrashedTimeoutmS,
	ThermalTimeoutmS,
	CruiseTimeoutmS,
	OpticalUpdatemS,
	WPTimeoutmS,
	CalibrationTimeoutmS,
	MotorStartmS,
	NavPulseUpdatemS,
	BLHeliTimeoutmS,
	oledDisplayUpdatemS,
	chaserTimeoutmS,
	ASUpdatemS,
	FrSkyTelemetryUpdatemS,
	mSLastArrayEntry
};

typedef volatile union {
	uint8 AllFlags[FLAG_BYTES];
	struct { // Order of these flags subject to change
		uint8 // 0
				AltControlEnabled :1,
				UsingGPSAltitude :1,
				RapidDescentHazard :1,
				LandingSwitch :1,
				NearLevel :1,
				LowBatt :1,
				GPSValid :1,
				OriginValid :1,

				// 1
				BaroFailure :1,
				IMUFailure :1,
				MagnetometerFailure :1,
				GPSFailure :1,
				AttitudeHold :1,
				ThrottleMoving :1,
				HoldingAlt :1,
				Navigate :1,

				// 2
				ReturnHome :1,
				WayPointAchieved :1,
				WayPointCentred :1,
				OrbitingWP :1,
				UsingRTHAutoDescend :1,
				BaroActive :1,
				RangefinderActive :1,
				UsingRangefinderAlt :1,

				// 3
				UsingPOI :1,
				PassThru :1,
				UsingAngleControl :1,
				Emulation :1,
				OffsetOriginValid :1,
				DrivesArmed :1,
				AccUBump :1,
				DCMotorsDetected :1,

				// 4
				Saturation :1,
				DumpingBlackBox :1,
				ParametersValid :1,
				Signal :1,
				UsingWPNavigation :1,
				IMUActive :1,
				MagnetometerActive :1,
				IsArmed :1,

				// 5
				IsFixedWing :1,
				ThrottleOpen :1,
				MagnetometerCalibrated :1,
				RCMapFail :1,
				NewAltitudeValue :1,
				IMUCalibrated :1,
				FenceAlarm :1,
				AccCalibrated :1,

				// 6
				NewBaroValue :1,
				BeeperInUse :1,
				HaveSerialRC :1,
				Soaring :1,
				ValidGPSVel :1,
				RCFrameReceived :1, // zzz
				Hovering :1,
				ASActive :1,

				// 7
				YawActive :1,
				HaveGPS :1,
				RCNewValues :1,
				NewNavUpdate :1,
				HaveNVMem :1,
				UsingRapidDescent :1, // was MPU6050
				UsingTurnToWP :1,
				Glide :1,

				// 8
				NewMagValues :1, UsingAltHoldAlarm :1, NewGPSPosition :1,
				sioFatal :1, GPSPacketReceived :1, WindEstValid :1,
				CrossTrackActive :1,
				NavigationEnabled :1,

				// 9
				ForcedLanding :1, unusedNewGPSPosition :1, EnforceDriveSymmetry :1,
				RCFrameOK :1, InvertMagnetometer :1, NewCommands :1,
				UsingOffsetHome :1, ValidGPSHeading:1,

				// 10
				BadBusDevConfig :1

		// 11
		; // MAXED OUT
	};
} Flags;

enum FlightStates {
	Starting,
	Warmup,
	Landing,
	Landed,
	Shutdown,
	InFlight,
	IREmulateUNUSED,
	Preflight,
	Ready,
	ThrottleOpenCheck,
	ErectingGyros,
	MonitorInstruments,
	InitialisingGPS,
	UnknownFlightState
};

enum reset_causes {
	UNKNOWN_RESET,
	LOW_POWER_RESET,
	WINDOW_WATCHDOG_RESET,
	INDEPENDENT_WATCHDOG_RESET,
	SOFTWARE_RESET,
	POWER_ON_POWER_DOWN_RESET,
	EXTERNAL_RESET_PIN_RESET,
	BROWNOUT_RESET
};

extern const char * ResetCauseNames[];

uint8 GetResetCause(void);
extern uint8 ResetCause;

extern uint8 State;
extern Flags F;
extern timeuS execTimeuS, execPeakTimeuS;

#endif

