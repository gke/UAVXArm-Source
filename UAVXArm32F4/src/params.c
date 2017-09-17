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

#include "defaults.h"

//uint8 NavStrength;

const real32 OKp = 0.25f;
const real32 OKi = 0.05f;
// derivative not used
const real32 OIL = 0.015f;

const real32 IKp = 0.005f;
// integral not used
#if defined(USE_PAVEL)
const real32 IKd = 0.000025f;
#else
const real32 IKd = 0.0001f;
#endif
//const real32 IIL = 0.01f;


volatile boolean StickArmed = false;
volatile boolean TxSwitchArmed = false;

uint8 UAVXAirframe = AFUnknown;
boolean IsMulticopter, IsFixedWing;
boolean UsingGliderStrategy, UsingFastStart, UsingBLHeliPrograming,
		UsingSpecial;

uint8 CurrConfig1, CurrConfig2;
uint8 CurrUAVXAirframe;
boolean CurrUsingUplink;

NVStruct NV;

inline uint8 P(uint8 i) {
	return (NV.P[NV.CurrPS][i]);
} // P

inline void SetP(uint8 i, uint8 v) {
	if (P(i) != v) {
		NV.P[NV.CurrPS][i] = v;
		NVChanged = true;
	}
} // SetP

void ClassifyAFType(void) {

	uint8 AF = P(AFType);

	IsFixedWing = (AF == ElevonAF) || (AF == DeltaAF) || (AF == AileronAF)
			|| (AF == AileronSpoilerFlapsAF) || (AF == RudderElevatorAF);
	IsMulticopter = !(IsFixedWing || (AF == VTOLAF) || (AF == Heli90AF) || (AF
			== Heli120AF));

	CruiseThrottle = IsFixedWing ? THR_DEFAULT_CRUISE_FW_STICK
			: THR_DEFAULT_CRUISE_STICK;

	OrientationRad = DegreesToRadians(AFOrientation[AF]);
	OrientS = sinf(OrientationRad);
	OrientC = cosf(OrientationRad);

	UAVXAirframe = AF;

}// ClassifyAFType


void DoConfigBits(void) {

	// Config1
	F.UsingRTHAutoDescend = (P(Config1Bits) & UseRTHDescendMask) != 0;
	F.GPSToLaunchRequired = (P(Config1Bits) & GPSToLaunchRequiredMask) != 0;
	F.InvertMagnetometer = (P(Config1Bits) & UseInvertMagMask) != 0;
	F.UsingRapidDescent = (P(Config1Bits) & UseRapidDescentMask) != 0;
	F.Emulation = (P(Config1Bits) & EmulationEnableMask) != 0;
	F.UseManualAltHold = (P(Config1Bits) & UseManualAltHoldMask) != 0;
	F.UsingGPSAltitude = (P(Config1Bits) & UseGPSAltMask) != 0;

	// Config2
	UsingSWFilters = (P(Config2Bits) & UseSWFiltersMask) != 0;
	UsingFastStart = (P(Config2Bits) & UseFastStartMask) != 0;
	UsingBLHeliPrograming = (P(Config2Bits) & UseBLHeliMask) != 0;
	UsingGliderStrategy = ((P(Config2Bits) & UseGliderStrategyMask) != 0)
			&& IsFixedWing;
	F.UsingTurnToWP = (P(Config2Bits) & UseTurnToWPMask) != 0;
	UsingSpecial = (P(Config2Bits) & UseSpecialMask) != 0;

	//... currentl unused

} // DoConfigBits

void RegeneratePIDCoeffs(void) {
	// retains familiar historical UAVP values
	AxisStruct * C;
	//real32 Temp;

	ThrottleGain = (real32) P(ThrottleGainRate);

	// Roll
	C = &A[Roll];

	C->AngleKp = (real32) P(RollAngleKp) * OKp;
	C->AngleKi = (real32) P(RollAngleKi) * OKi;

	C->AngleIL = DegreesToRadians(P(RollIntLimit)) * OIL;

	C->AngleMax = DegreesToRadians(P(MaxRollAngle));

	C->RateKp = (real32) P(RollRateKp) * IKp;
	// no integral
	C->RateKd = (real32) P(RollRateKd) * IKd;

	C->RateMax = C->AngleMax * C->AngleKp;
	SetP(MaxRollRate, RadiansToDegrees(C->RateMax) * 0.1f);

	// Pitch
	C = &A[Pitch];

	C->AngleKp = (real32) P(PitchAngleKp) * OKp;
	C->AngleKi = (real32) P(PitchAngleKi) * OKi;

	C->AngleIL = DegreesToRadians(P(PitchIntLimit)) * OIL;

	C->AngleMax = DegreesToRadians(P(MaxPitchAngle));

	C->RateKp = (real32) P(PitchRateKp) * IKp;
	C->RateKd = (real32) P(PitchRateKd) * IKd;

	C->RateMax = C->AngleMax * C->AngleKp;
	SetP(MaxPitchRate, RadiansToDegrees(C->RateMax) * 0.1f);

	// Nav

	Nav.LPFCutOffHz = 1.0f; // GPS_UPDATE_HZ * 0.2f;

	Nav.PosKp = (real32) P(NavPosKp) * 0.0165f; //20 -> 0.33f;
	Nav.PosKi = (real32) P(NavPosKi) * 0.004f; // 5 -> 0.02f;

	Nav.MaxAngle = DegreesToRadians(Limit(P(NavMaxAngle), 2, MAX_ANGLE_DEG));
	if (Nav.MaxAngle > A[Roll].AngleMax) {
		Nav.MaxAngle = A[Roll].AngleMax;
		SetP(NavMaxAngle, RadiansToDegrees(Nav.MaxAngle));
	}

	Nav.VelKp = (real32) P(NavVelKp) * 0.06f; // 20 -> 1.2f; // @45deg max
	Nav.VelKi = 0.0f;
	Nav.VelIL = 0.0f;

	Nav.MaxVelocity = P(NavPosIntLimit);

	// Yaw
	C = &A[Yaw];

	C->RateMax = DegreesToRadians(P(MaxYawRate) * 10.0f);

	C->CompassRateMax = DegreesToRadians(P(MaxCompassYawRate) * 10.0f);

	C->RateKp = (real32) P(YawRateKp) * IKp;
	C->RateKd = (real32) P(YawRateKd) * IKd;

	C->AngleMax = Nav.HeadingTurnoutRad;
	C->AngleKp = C->CompassRateMax / C->AngleMax;

	if (UsingSpecial) {
		// Altitude
		Alt.PosKp = (real32) P(AltPosKp) * 0.18f;
		Alt.PosKi = (real32) P(AltPosKi) * 0.074f;

		Alt.PosIL = 0.35f; // 0.15f;

		Alt.VelKp = (real32) P(AltVelKp) * 0.026f;
		Alt.VelKd = (real32) P(AltVelKd) * 0.0016f;

	} else {

		// Altitude
		Alt.PosKp = (real32) P(AltPosKp) * 0.018f;
		Alt.PosKi = (real32) P(AltPosKi) * 0.0074f;

		Alt.PosIL = 0.35f; // 0.15f;

		Alt.VelKp = (real32) P(AltVelKp) * 0.0026f;
		Alt.VelKd = (real32) P(AltVelKd) * 0.00016f;

	}

	// Camera
	Cam.RollKp = P(RollCamKp) * 0.1f;
	Cam.PitchKp = P(PitchCamKp) * 0.1f;

} // RegeneratePIDCoeffs

void SetPIDPeriod(void) {

	if (CurrESCType == ESCSyncPWM) {
		if (P(PIDTimeSel) > 0)
			SetP(PIDTimeSel, 0);
		CurrPIDTimeSel = P(PIDTimeSel);
		CurrPIDCycleuS = PID_SYNCPWM_CYCLE_2050US;
	} else {
#if defined(V4_BOARD)
		if (P(PIDTimeSel) > 1)
		SetP(PIDTimeSel, 1);
#else
		if (P(PIDTimeSel) > 1)
			SetP(PIDTimeSel, 1);
#endif
		CurrPIDTimeSel = P(PIDTimeSel);
		CurrPIDCycleuS = PID_CYCLE_2000US >> CurrPIDTimeSel;
	}

	CurrPIDCycleS = CurrPIDCycleuS * 1.0e-6;

} // SetPIDPeriod


void UpdateParameters(void) {
	// overkill if only a single parameter has changed but not in flight loop
	real32 Temp;

	if (F.ParametersChanged) {

		// Misc

		F.UsingConvPropSense = (P(ServoSense) & UseConvPropSenseMask) != 0;

		UpdateRCMap(); // for channel re-assignment

		// Change to physical configuration or attached devices  - NEEDS POWER CYCLE

		if ((State == Preflight) || (State == Ready)) { // NOT IN FLIGHT
			if ((CurrStateEst != P(StateEst))
					|| (ArmingMethod != P(ArmingMode)) //
					|| (CurrAttSensorType != P(SensorHint)) //
					|| (CurrComboPort1Config != P(ComboPort1Config)) //
					|| (CurrComboPort2Config != P(ComboPort2Config)) //
					|| (CurrConfig1 != P(Config1Bits)) //
					|| (CurrConfig2 != P(Config2Bits)) //
					|| (CurrESCType != P(ESCType)) //
					|| (UAVXAirframe != P(AFType)) //
					|| (CurrRFSensorType != P(RFSensorType)) //
					|| (CurrASSensorType != P(ASSensorType)) //
					|| (CurrGyroLPFSel != P(GyroLPFSel)) //
					|| (CurrAccLPFSel != P(AccLPFSel)) //
					|| (CurrPIDTimeSel != P(PIDTimeSel)) //
					|| (CurrGPSType != P(GPSProtocol)) //
					|| (CurrwsNoOfLeds != P(WS2812Leds))) {
				if ((P(Config2Bits) & UseConfigRebootMask) != 0)
					NVIC_SystemReset();
				else
					Catastrophe();
			}

			memset(&A, 0, sizeof(AxisStruct));
			memset(&Rate, 0, sizeof(Rate[3]));
			memset(&Acc, 0, sizeof(Acc[3]));
			Acc[Z] = -GRAVITY_MPS_S;
		}

		DoConfigBits();

		InitTune();

		// Throttle

		Temp = Limit((int16)P(PercentIdleThr) ,0 ,20);
		SetP(PercentIdleThr, Temp);
		IdleThrottle = FromPercent(Temp);
		IdleThrottlePW = FromPercent(Temp);

		BestROCMPS = Limit(P(BestROC), 0.0f, 5.0f);
		MaxAltHoldCompFrac = FromPercent(P(MaxAltHoldComp));

		FWPitchThrottleFFFrac = FromPercent(P(FWPitchThrottleFF));

		TiltThrFFFrac = FromPercent(P(TiltThrottleFF));

		CGOffset = FromPercent(Limit1(P(Balance), 100));

		if (P(EstCruiseThr) > 0)
			CruiseThrottle
					= Limit(FromPercent(P(EstCruiseThr)), THR_MIN_ALT_HOLD_STICK, THR_MAX_ALT_HOLD_STICK);
		else
			CruiseThrottle = (IsFixedWing) ? THR_DEFAULT_CRUISE_FW_STICK
					: THR_DEFAULT_CRUISE_STICK;
		SetP(EstCruiseThr, CruiseThrottle * 100.0f);

		// Attitude

		StickDeadZone = FromPercent(Limit(P(StickHysteresis), 1, 10));

		KpAccBase = P(MadgwickKpAcc) * 0.1f;
		BetaBase = KpAccBase * 0.2f;
		KpMagBase = P(MadgwickKpMag) * 0.1f;

		AccConfidenceSDevR = 1.0f / (Limit(P(AccConfSD), 0, 100) * 0.01f);

		FWAileronDifferentialFrac = FromPercent(P(FWAileronDifferential));
		FWRollPitchFFFrac = -FromPercent(P(FWRollPitchFF));

		HorizonTransPoint = FromPercent(Limit(P(Horizon), 1, 100));

		FWMaxClimbAngleRad = DegreesToRadians(P(FWMaxClimbAngle));
		FWBoardPitchAngleRad = DegreesToRadians(P(FWBoardPitchAngle));

		// Altitude

		InitRangefinder();

		MinROCMPS = -(real32) P(MaxDescentRateDmpS) * 0.1f;

		F.UsingGPSAltitude = F.UsingGPSAltitude & IsFixedWing && ((CurrGPSType
				== UBXBinGPS) || (CurrGPSType == UBXBinGPSInit));

		FWAileronRudderFFFrac = P(FWAileronRudderMix) * 0.01f;
		FWAltSpoilerFFFrac = P(FWAltSpoilerFF) * 0.01f;

		FWSpoilerDecayS = P(FWSpoilerDecayTime);

		AltCompDecayS = P(AltCompDecayTime) * 0.1f;
		FWSpoilerDecayS = P(FWSpoilerDecayTime) * 0.1f;

		AltLPFHz = Limit((real32)P(AltLPF) * 0.1f, 0.1f, 5.0f); // apply to Baro and Zacc

		// Nav

		Nav.CrossTrackKp = P(NavCrossTrackKp) * 0.01f;
		Nav.FenceRadius = NAV_DEFAULT_FENCE_M; // TODO: from Default Mission

		if (P(NavHeadingTurnout) <= 0)
			SetP(NavHeadingTurnout, RadiansToDegrees(DEFAULT_TURNOUT_RAD));
		Nav.HeadingTurnoutRad
				= DegreesToRadians(Limit(P(NavHeadingTurnout), 10, 90));

		if (P(MinhAcc) <= 0) {
			GPSMinhAcc = GPS_MIN_HACC;
			SetP(MinhAcc, GPS_MIN_HACC * 10.0f);
		} else
			GPSMinhAcc = P(MinhAcc) * 0.1f;

		InitServoSense();
		InitBattery();

		CurrTelType = P(TelemetryType);
		F.UsingMAVLink = (CurrTelType == MAVLinkTelemetry) || (CurrTelType
				== MAVLinkMinTelemetry);

		RegeneratePIDCoeffs();

		F.ParametersChanged = false;
	}

} // UpdateParameters

#define THR_LO  (1<<(ThrottleRC<<1)) // 0b00000001
#define THR_CE  (3<<(ThrottleRC<<1)) // 0b00000011
#define THR_HI  (2<<(ThrottleRC<<1)) // 0b00000010
#define ROL_LO  (1<<(RollRC<<1)) // 0b00000100
#define ROL_CE  (3<<(RollRC<<1)) // 0b00001100
#define ROL_HI  (2<<(RollRC<<1)) // 0b00001000
#define PIT_LO  (1<<(PitchRC<<1)) // 0b00010000
#define PIT_CE  (3<<(PitchRC<<1)) // 0b00110000
#define PIT_HI  (2<<(PitchRC<<1)) // 0b00001000
#define YAW_LO  (1<<(YawRC<<1)) // 0b01000000
#define YAW_CE  (3<<(YawRC<<1)) // 0b11000000
#define YAW_HI  (2<<(YawRC<<1)) // 0b10000000
enum StickStates {
	MonitorSticks, SticksChanging, SticksChanged
};
static uint8 SticksState = MonitorSticks;
static uint8 StickPattern = 0;

void AccTrimStickAdjust(real32 BFTrim, real32 LRTrim) {

	NV.AccCal.Bias[Y] += BFTrim;
	NV.AccCal.Bias[X] += LRTrim;

} // AccTrimStickAdjust

void UpdateSticksState(void) {
	// pattern scheme from MultiWii
	uint32 NowmS;
	uint8 pattern = 0;
	idx i;

	real32 RCPattern[4];
	real32 Swap;

	for (i = ThrottleRC; i <= YawRC; i++)
		RCPattern[i] = RC[i];

	if (P(ArmingMode) == YawStickArming) {
		Swap = RCPattern[RollRC];
		RCPattern[RollRC] = RCPattern[YawRC];
		RCPattern[YawRC] = Swap;
	}

	for (i = ThrottleRC; i <= YawRC; i++) {
		pattern >>= 2;
		if (RCPattern[i] > FromPercent(30))
			pattern |= 0x80; // check for MIN
		if (RCPattern[i] < FromPercent(70))
			pattern |= 0x40; // check for MAX
	}

	NowmS = mSClock();
	switch (SticksState) {
	case MonitorSticks:
		if (StickPattern != pattern) {
			StickPattern = pattern;
			mSTimer(NowmS, StickTimeout, 2000);
			SticksState = SticksChanging;
		}
		break;
	case SticksChanging:
		if (StickPattern == pattern) {
			if (NowmS > mS[StickTimeout])
				SticksState = SticksChanged;
		} else
			SticksState = MonitorSticks;
		break;
	default:
		break;
	}// switch

} // UpdateSticksState


void DoStickProgramming(void) {
	uint32 NowmS;
	int8 NewCurrPS;
	real32 BFTrim, LRTrim;

	if (!Armed()) {

		UpdateSticksState();

		if (SticksState == SticksChanged) {
			if ((P(ArmingMode) != SwitchArming) && (StickPattern == (THR_LO
					| YAW_CE | PIT_CE | ROL_HI)) && ArmingSwitch && !StickArmed) {
				LEDOn(LEDBlueSel);
				//DoBeep(3, 0);
				StickArmed = true;
				SticksState = MonitorSticks;
				LEDOff(LEDBlueSel);
			} else {
				NewCurrPS = NV.CurrPS;
				switch (StickPattern) {
				case THR_LO | YAW_HI | PIT_LO | ROL_CE: // TopRight
					if (++NewCurrPS >= NO_OF_PARAM_SETS)
						NewCurrPS = 0;
					break;
				case THR_LO | YAW_LO | PIT_LO | ROL_CE: // TopLeft
					if (--NewCurrPS < 0)
						NewCurrPS = NO_OF_PARAM_SETS - 1;
					break;
				default:
					break;
				} // switch

				if (NewCurrPS != NV.CurrPS) {
					LEDOn(LEDBlueSel);
					// IGNORE	NV.CurrPS = NewCurrPS;
					DoBeeps(NV.CurrPS + 1);
					F.ParametersChanged = true;
					LEDOff(LEDBlueSel);
				}
				SticksState = MonitorSticks;
			}
		}
	} else if (State == Landed) {

		UpdateSticksState();

		if (SticksState == SticksChanged) {
			if ((P(ArmingMode) != SwitchArming) && (StickPattern == (THR_LO
					| YAW_CE | PIT_CE | ROL_LO)) && StickArmed) {
				LEDOn(LEDBlueSel);
				//DoBeep(1, 0);
				StickArmed = false;
				SticksState = MonitorSticks;
				LEDOff(LEDBlueSel);
			} else {
				BFTrim = LRTrim = 0.0f;

				switch (StickPattern) {
				case THR_LO | YAW_CE | PIT_HI | ROL_CE:
					BFTrim = +ACC_TRIM_STEP;
					NVChanged = true;
					break;
				case THR_LO | YAW_CE | PIT_LO | ROL_CE:
					BFTrim = -ACC_TRIM_STEP;
					NVChanged = true;
					break;
				case THR_LO | YAW_LO | PIT_CE | ROL_CE:
					LRTrim = -ACC_TRIM_STEP;
					NVChanged = true;
					break;
				case THR_LO | YAW_HI | PIT_CE | ROL_CE:
					LRTrim = +ACC_TRIM_STEP;
					NVChanged = true;
					break;
				default:
					break;
				} // switch

				if (NVChanged) {
					AccTrimStickAdjust(BFTrim, LRTrim);
					// updated in Landing or disarm UpdateNV();

					UpdateGyroTempComp();
					DoBeep(1, 0);
					SticksState = SticksChanging;
					NowmS = mSClock();
					mSTimer(NowmS, StickTimeout, 100);
					mSTimer(NowmS, ArmedTimeout, ARMED_TIMEOUT_MS);
				} else
					SticksState = MonitorSticks;
			}
		}
	}

	UpdateParameters();

} // DoStickProgramming


void UseDefaultParameters(uint8 DefaultPS) { // loads a representative set of initial parameters as a base for tuning
	idx i;

	NV.CurrPS = 0;

	for (i = 0; i < MAX_PARAMETERS; i++)
		SetP(DefaultParams[i].tag, DefaultParams[i].p[DefaultPS]);

	ClassifyAFType();

	if (IsFixedWing) {
		NV.Mission.ProximityAltitude = WING_PROXIMITY_ALTITUDE_M;
		NV.Mission.ProximityRadius = WING_PROXIMITY_RADIUS_M;
	} else {
		NV.Mission.ProximityAltitude = NAV_PROXIMITY_ALTITUDE_M;
		NV.Mission.ProximityRadius = NAV_PROXIMITY_RADIUS_M;
	}

	NV.Mission.NoOfWayPoints = 0;
	NV.Mission.RTHAltHold = NAV_DEFAULT_RTH_M;

	UpdateNV();

	F.ParametersChanged = true;

} // UseDefaultParameters

void CheckParametersInitialised(void) {
	uint8 v;
	idx i;

	ReadBlockNV(0, sizeof(NV), (int8 *) (&NV));

	NV.CurrPS = 0;

	v = P(0);
	NVChanged = true;
	for (i = 1; i < MAX_PARAMETERS; i++)
		NVChanged = NVChanged && (v == P(i));

	if (NVChanged) {
		memset(&NV, 0, sizeof(NV));
		NV.CurrRevisionNo = RevisionNo;
		UseDefaultParameters(0);
		InitMagnetometerBias();

		UpdateNV();
	}

} // CheckParametersInitialised

void InitParameters(void) {

	F.UsingUplink = F.ParametersValid = true; //unused

	ReadBlockNV(0, sizeof(NV), (int8 *) (&NV)); // redundant?

	NV.CurrPS = 0; // zzz force it

	ClassifyAFType();

	// must have these
	CurrStateEst = P(StateEst);
	if ((CurrStateEst == MadgwickMARG) && IsFixedWing) {
		SetP(StateEst, MadgwickIMU);
		CurrStateEst = MadgwickIMU;
	}

	ArmingMethod = P(ArmingMode);
	CurrAttSensorType = P(SensorHint);
	CurrComboPort1Config = P(ComboPort1Config);
	CurrComboPort2Config = P(ComboPort2Config);
	CurrConfig1 = P(Config1Bits);
	CurrConfig2 = P(Config2Bits);
	UAVXAirframe = Limit(P(AFType), 0, AFUnknown);
	CurrESCType = Limit(P(ESCType), 0, ESCUnknown);
	CurrGPSType = P(GPSProtocol);
	CurrRFSensorType = P(RFSensorType);
	CurrASSensorType = P(ASSensorType);

	SetPIDPeriod();

	if (UAVXAirframe == IREmulation) { // force PWM for DAC function
		SetP(ESCType, PWMDAC);
		CurrESCType = PWMDAC;
	}

	DoConfigBits();

#if defined(V4_BOARD) // give ComboPort1 priority for GPS
	if ((CurrComboPort1Config == CPPM_GPS_M7to10) && (CurrComboPort2Config
					== GPS_RF_V4)) {
		CurrComboPort2Config = Unused_RF_V4;
		SetP(ComboPort2Config, CurrComboPort2Config);
	}
#else
	if (CurrComboPort2Config != I2C_RF_BatV_V3) {
		CurrComboPort2Config = ComboPort2Unused;
		SetP(ComboPort2Config, CurrComboPort2Config);
	}
#endif

	if (CurrComboPort1Config != ComboPort1ConfigUnknown) {
		InitRCComboPort();
		InitRC();

		if ((CurrESCType != ESCUnknown) && (UAVXAirframe != AFUnknown))
			InitDrives();
		else
			DrivesInitialised = false;
	}

	F.UsingAnalogGyros = (CurrAttSensorType != UAVXArm32IMU)
			&& (CurrAttSensorType != FreeIMU);

	CurrGyroLPFSel = P(GyroLPFSel);
	CurrAccLPFSel = P(AccLPFSel);

	CurrwsNoOfLeds = Limit(P(WS2812Leds), 0, MAX_WS2812_LEDS);
	wsInit();

	if (GPSRxSerial != TelemetrySerial)
		InitGPS();

	InitMAVLink();

	F.ParametersChanged = true;
	UpdateParameters();

} // InitParameters


