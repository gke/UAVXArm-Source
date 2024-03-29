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

const uint8 NoOfDefParamSets = sizeof(DefaultParams) / sizeof(ParamStruct);

volatile boolean TxSwitchArmed = false;

uint8 UAVXAirframe = AFUnknown;
boolean IsMulticopter, IsGroundVehicle;
boolean UsingBatteryComp, UsingGliderStrategy, UsingFastStart, UsingBLHeliPrograming, UsingHWLPF, UsingCruiseCentering,
		UsingNavBeep, DisablingLEDsInFlight;

uint8 CurrConfig1, CurrConfig2;
uint8 CurrUAVXAirframe;
uint8 CurrMotorStopSel;

ConfigStruct Config;
boolean ConfigChanged = false;

boolean ConfigUninitialised(void) {
	uint8 p;
	boolean same = true;

	for (p = 1; p<= MAX_PARAMETERS; p++)
		same = same && (Config.P[0][p-1] == Config.P[0][p]);

	return (same);
} // ConfigUninitialised

void RefreshConfig(void) {
	/*
	 boolean changed = false;
	 uint32 i;

	 i = 0;
	 do {
	 changed |= *((uint8 *) &Config + i)
	 != *(int8 *) (CONFIG_FLASH_ADDR + i);
	 } while ((++i < sizeof(Config)) && !changed);
	 */
	if (ConfigChanged && (State != InFlight)) {
		WriteBlockArmFlash(true, CONFIG_FLASH_SECTOR, CONFIG_FLASH_ADDR + 0,
				sizeof(Config), (uint8 *) &Config);
		ConfigChanged = false;
	}

} // RefreshConfig

uint8 P(uint8 i) {
	return (Config.P[Config.CurrPS][i]);
} // P

void SetP(uint8 i, uint8 v) {
	if (Config.P[Config.CurrPS][i] != v) {
		Config.P[Config.CurrPS][i] = v;
		ConfigChanged = true;
	}
} // SetP

// TODO: really should have a limits array for all params as a prepass
uint8 LimitP(uint8 i, uint8 l, uint8 h) {

	if (P(i) < l)
		SetP(i, l);
	else if (P(i) > h)
		SetP(i, h);

	return (P(i));
} // LimitP

void ClassifyAFType(void) {

	uint8 AF = P(AFType);

	F.IsFixedWing = (AF == ElevonAF) || (AF == DeltaAF) || (AF == AileronAF)
			|| (AF == AileronSpoilerFlapsAF) || (AF == AileronVTailAF)
			|| (AF == RudderElevatorAF);
	IsMulticopter = !(F.IsFixedWing || (AF == VTOLAF) || (AF == VTOL2AF)
			|| (AF == Heli90AF) || (AF == BiAF) || (AF == DifferentialTwinAF));
	IsGroundVehicle = (AF == TrackedAF) || (AF == TwoWheelAF)
			|| (AF == FourWheelAF);

	OrientationRad = DegreesToRadians(AFOrientation[AF]);

	OrientS = sinf(OrientationRad);
	OrientC = cosf(OrientationRad);

	UAVXAirframe = AF;

} // ClassifyAFType

void DoConfigBits(void) {

	// Config1
	F.UsingRTHAutoDescend = (P(Config1Bits) & UseRTHDescendMask) != 0;
	F.UsingAltHoldAlarm = (P(Config1Bits) & UseAltHoldAlarmMask) != 0;
	F.InvertMagnetometer = (P(Config1Bits) & UseInvertMagMask) != 0;

	F.UsingRapidDescent = (P(Config1Bits) & UseRapidDescentMask) == 0; // change to flag used to disable VRS

	F.Emulation = (P(Config1Bits) & EmulationEnableMask) != 0;
	F.UsingOffsetHome = (P(Config1Bits) & UseOffsetHomeMask) != 0;
	DisablingLEDsInFlight = (P(Config1Bits) & DisableLEDsInFlightMask) != 0;
	// Config2
	UsingBatteryComp = (P(Config2Bits) & UseBatteryCompMask) != 0;
	UsingFastStart = (P(Config2Bits) & UseFastStartMask) != 0;
	UsingBLHeliPrograming = (P(Config2Bits) & UseBLHeliMask) != 0;
	UsingGliderStrategy = ((P(Config2Bits) & UseGliderStrategyMask) != 0)
			&& F.IsFixedWing;
	UsingNavBeep = (P(Config2Bits) & UseNavBeepMask) != 0;

	F.UsingTurnToWP = (P(Config2Bits) & UseTurnToWPMask) != 0;

} // DoConfigBits

void InitPIDStructs(void) {
	// retains familiar historical UAVP values
	AxisStruct * C;

	// Nav

	Nav.PosKp = (real32) P(NavPosKp) * 0.0165f; //20 -> 0.33f;
	Nav.PosKi = (real32) P(NavPosKi) * 0.004f; // 5 -> 0.02f;

	Nav.CrossTrackKp = P(NavCrossTrackKp) * 0.01f;
	Nav.MaxCompassRate = DegreesToRadians(P(MaxCompassYawRate) * 10.0f);
	Nav.MaxBankAngle = DegreesToRadians(
			LimitP(NavMaxAngle, 10, P(MaxRollAngle)));
	Nav.HeadingTurnout = DegreesToRadians(LimitP(NavHeadingTurnout, 10, 90));

	Nav.VelKp = (real32) P(NavVelKp) * 0.06f; // 20 -> 1.2f; // @45deg max
	Nav.MaxVelocity = P(NavPosIntLimit);

	Nav.ProximityAlt = LimitP(NavProxAltM, 1, 5);
	Nav.ProximityRadius = LimitP(NavProxRadiusM, 1, 100);
	Nav.FenceRadius = LimitP(NavFenceRadiusM, 20, 250) * 10.0f;

	// Attitude

	const real32 ScaleAngleKp = 0.25f;
	const real32 ScaleAngleKi = 0.05f;
	const real32 ScaleAngleIL = 0.015f;
	// no derivative

	const real32 ScaleRateKp = 0.005f;
	const real32 ScaleRateKd = 0.0001f; // 0.0045

	// Pitch
	C = &A[Pitch];

	C->P.Kp = (real32) P(PitchAngleKp) * ScaleAngleKp;
	C->P.Ki = (real32) P(PitchAngleKi) * ScaleAngleKi;
	C->P.IntLim = DegreesToRadians(P(PitchAngleIntLimit)) * ScaleAngleIL;
	C->P.Max = DegreesToRadians(P(MaxPitchAngle));

	C->R.Kp = (real32) P(PitchRateKp) * ScaleRateKp;
	C->R.Kd = (real32) P(PitchRateKd) * ScaleRateKd;

	C->R.Max = C->P.Max * C->P.Kp;
	SetP(MaxPitchRate, RadiansToDegrees(C->R.Max) * 0.1f);

	// Roll
	C = &A[Roll];

	C->P.Kp = (real32) P(RollAngleKp) * ScaleAngleKp;
	C->P.Ki = (real32) P(RollAngleKi) * ScaleAngleKi;
	C->P.IntLim = DegreesToRadians(P(RollAngleIntLimit)) * ScaleAngleIL;
	C->P.Max = DegreesToRadians(P(MaxRollAngle));

	C->R.Kp = (real32) P(RollRateKp) * ScaleRateKp;
	C->R.Kd = (real32) P(RollRateKd) * ScaleRateKd;

	C->R.Max = C->P.Max * C->P.Kp;
	SetP(MaxRollRate, RadiansToDegrees(C->R.Max) * 0.1f);

	// Yaw

	const real32 ScaleAngleYawKp = 0.25f;
	const real32 ScaleAngleYawKi = 0.05f; // 1/5 of Kp
	const real32 ScaleAngleYawIL = 0.05f;

	const real32 ScaleRateYawKp = 0.005f;
	const real32 ScaleRateYawKd = 0.000025f;

	C = &A[Yaw];

	C->P.Max = Nav.HeadingTurnout;
	C->P.Kp = Nav.MaxCompassRate / C->P.Max;

	SetP(YawAngleKp, C->P.Kp / ScaleAngleYawKp);

	C->P.Ki = (real32) P(YawAngleKi) * ScaleAngleYawKi; // sign shift at 180 deg problem
	C->P.IntLim = DegreesToRadians(P(YawAngleIntLimit) * ScaleAngleYawIL);

	C->R.Kp = (real32) P(YawRateKp) * ScaleRateYawKp;
	C->R.Kd = (real32) P(YawRateKd) * ScaleRateYawKd;
	C->R.Max = DegreesToRadians(P(MaxYawRate) * 10.0f);

	// Altitude

	//memset(&Alt, 0, sizeof(Alt));

	MinROCMPS = -(real32) P(MaxDescentRateDmpS) * 0.1f;
	VRSDescentRateMPS = -LimitP(VRSDescentRate, P(MaxDescentRateDmpS), 60)
			* 0.1f;

	Alt.P.Kp = (real32) P(AltPosKp) * 0.0183f; // 0.515->28
	Alt.P.Ki = (real32) P(AltPosKi) * 0.00046f; // 0.0046->10
	Alt.P.IntLim = (real32) P(AltPosIntLimit) * 0.05f;
	Alt.P.Max = Abs(VRSDescentRateMPS);

	Alt.R.Kp = (real32) P(AltVelKp) * 0.0026f; // MatLab 0.085->32
	Alt.R.Ki = (real32) P(AltVelKi) * 0.00027f; // MatLab 0.0027->10
	Alt.R.IntLim = (real32) P(AltVelIntLimit) * 0.05f;
	Alt.R.Max = ALT_MAX_ROC_MPS;
	MaxAltHoldThrComp = FromPercent((real32 ) P(AltThrottleCompLimit));

	Alt.R.Kd = 0.0f;

	// Camera
	Cam.RollKp = P(RollCamKp) * 0.1f;
	Cam.PitchKp = P(PitchCamKp) * 0.1f;

	// Gain Scheduling

	MaxAttitudeGainReduction = FromPercent(LimitP(ThrottleGainRate, 0, 70));

	//ROCGainAltBand = (real32) LimitP(AltHoldBand, 2, 10);
	//ROCGainAltBandR = 1.0f / ROCGainAltBand;
	//MaxROCGainReduction = FromPercent(LimitP(ROCGainRate, 0, 70));

} // InitPIDStructs

void SetPIDPeriod(void) {

	CurrPIDCycleuS =
			busDev[imuSel].useSPI ? PID_CYCLE_1000US : PID_CYCLE_2000US;

	CurrPIDCycleS = CurrPIDCycleuS * 1.0e-6;

} // SetPIDPeriod

boolean CurrAltFiltersChanged(void) {

	return (CurrAltLPFHz != P(AltLPF));

} // CurrAltFiltersChanged

boolean CurrInertialFiltersChanged(void) {

	return
			(CurrIMUFilterType != P(IMUFiltType))
			|| (CurrAccLPFSel != P(AccLPFSel)) //
			|| (CurrGyroLPFSel != P(GyroLPFSel)) //
			|| (CurrYawLPFHz != P(YawLPFHz)) //
			|| (CurrServoLPFHz != P(ServoLPFHz));

} // CurrInertialFiltersChanged

void UpdateParameters(void) {
	// overkill if only a single parameter has changed but not in flight loop

	// Misc

	UpdateRCMap(); // for channel re-assignment

	// The following if statement identifies a change to the physical
	// configuration or attached devices and forces a reboot to restore
	// Arm registers to a known state and to initialise new devices.

	if ((State == Preflight) || (State == Ready)
			|| (State == MonitorInstruments)) { // PARANOID
		if ( (ArmingMethod != P(ArmingMode)) //
				|| (CurrRxType != P(RxType)) //
				|| (CurrConfig1 != P(Config1Bits)) //
				|| (CurrConfig2 != P(Config2Bits)) //
				|| (CurrESCType != P(ESCType)) //
				|| (UAVXAirframe != P(AFType)) //
				|| (CurrTelType != P(TelemetryType)) //
				|| (CurrBBLogType != P(BBLogType)) //
				|| (CurrRFSensorType != P(RFSensorType)) //
				|| (CurrASSensorType != P(ASSensorType)) //
				|| CurrInertialFiltersChanged() //
				|| CurrAltFiltersChanged() //
				|| (CurrGPSType != P(GPSProtocol)) //
				|| (CurrMotorStopSel != P(MotorStopSel)) //
				)
			systemReset(false);

		InitPIDStructs();

		// Throttle
		IdleThrottlePW = IdleThrottle = FromPercent(
				LimitP(PercentIdleThr, RC_THRES_START + 1, 20));

		FWPitchThrottleFFFrac = FromPercent(P(FWPitchThrottleFF));
		TiltThrFFFrac = FromPercent(P(TiltThrottleFF));

		CGOffset = FromPercent(Limit1(P(Balance), 100));

		if (F.Emulation)
			CruiseThrottle =
					F.IsFixedWing ?
							EM_THR_CRUISE_FW_STICK : EM_THR_CRUISE_STICK;
		else
			CruiseThrottle =
					FromPercent(
							LimitP(EstCruiseThr, ToPercent(THR_MIN_ALT_HOLD_STICK), ToPercent(THR_MAX_ALT_HOLD_STICK) ));

		//CruiseThrottleTrackingRate
		//		= FromPercent(LimitP(CruiseTrackingRate, 1, 100) * 0.01f);

		// Attitude

		FWStickScaleFrac = FromPercent(
				LimitP(FWStickScale, 15, 100)) * OUT_NEUTRAL;

		FWRollControlPitchLimitRad = DegreesToRadians(
				LimitP(FWRollControlPitchLimit, 45, 60));

		StickDeadZone = FromPercent(LimitP(StickHysteresis, 1, 5));

		AccConfidenceSDevR = 1.0f / (LimitP(AccConfSD, 1, 127) * 0.01f);

		TwoKpAccBase = P(MadgwickKpAcc) * 0.01f * 2.0f;
		KpMag = P(MadgwickKpMag) * 0.01f;

		FWAileronDifferentialFrac = FromPercent(P(FWAileronDifferential));
		FWRollPitchFFFrac = -FromPercent(P(FWRollPitchFF));

		HorizonTransScale = 1.0f / FromPercent(LimitP(Horizon, 1, 100));

		FWMaxClimbAngleRad = DegreesToRadians(P(FWMaxClimbAngle));
		FWBoardPitchAngleRad = DegreesToRadians(P(FWBoardPitchAngle));

		FWClimbThrottleFrac = FromPercent(P(FWClimbThrottle));

		// Altitude
		InitRangefinder();

		TrackBaroVariance = BaroVariance = (real32) (LimitP(KFBaroVar, 2, 200))
				* 0.01f;
		TrackAccUVariance = AccUVariance = (real32) (LimitP(KFAccUVar, 2, 200))
				* 0.1f;
		AccUBiasVariance = (real32) (LimitP(KFAccUBiasVar, 1, 200)) * 0.000001f;

		FWAileronRudderFFFrac = FromPercent(P(FWAileronRudderMix));
		FWAltSpoilerFFFrac = FromPercent(P(FWAltSpoilerFF));

		FWSpoilerDecayPS = FromPercent((real32)LimitP(FWSpoilerDecayPercentPS, 10, 50) * 0.1f);

		AltHoldThrCompDecayPS = FromPercent((real32)LimitP(AltHoldThrCompDecayPercentPS, 10, 200) * 0.1f );

		ThrottleMovingWindow = FromPercent(P(AHThrottleMovingTrigger)) * THR_UPDATE_S;

		AltitudeHoldROCWindow = (timemS) LimitP(AHROCWindowMPS, 15, 200) * 0.01f;

		// Nav
		NavGPSTimeoutmS = 3000.0f; // (timemS) LimitP(NavGPSTimeoutS, 2, 30) * 1000;
		MagVariation = DegreesToRadians((real32 )P(NavMagVar) * 0.1f);

		GPSMinhAcc = GPS_MIN_HACC; // not set in GUI LimitP(MinhAcc, 10, GPS_MIN_HACC * 10) * 0.1f;
		GPSMinvAcc = GPS_MIN_VACC; // not set in GUI LimitP(MinhAcc, 10, GPS_MIN_HACC * 10) * 0.1f;

		// Misc
		InitServoSense();

		InitTune();

		RefreshConfig();
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

	Config.AccCal.Bias[Y] += BFTrim;
	Config.AccCal.Bias[X] += LRTrim;

	ConfigChanged = true;

} // AccTrimStickAdjust

void UpdateSticksState(void) {
	// pattern scheme from MultiWii
	timemS NowmS;
	uint8 pattern = 0;
	idx i;

	real32 RCPattern[4];
	real32 Swap;

	for (i = ThrottleRC; i <= YawRC; i++)
		RCPattern[i] = RC[i];

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
			mSTimer(StickTimeoutmS, 2000);
			SticksState = SticksChanging;
		}
		break;
	case SticksChanging:
		if (StickPattern == pattern) {
			if (mSTimeout(StickTimeoutmS))
				SticksState = SticksChanged;
		} else
			SticksState = MonitorSticks;
		break;
	default:
		break;
	} // switch

} // UpdateSticksState

void DoStickProgramming(void) {
	timemS NowmS;
	int8 NewCurrPS;
	real32 BFTrim, LRTrim;
	boolean Changed = false;

	// Yaw & Roll swapped in pattern gen to make Mode 0/1 pattern equivalent

	if (State == Landed) {

		UpdateSticksState();

		if (SticksState == SticksChanged) {

			BFTrim = LRTrim = 0.0f;
			Changed = false;

			switch (StickPattern) {
			case THR_LO | YAW_CE | PIT_HI | ROL_CE:
				BFTrim = +ACC_TRIM_STEP;
				Changed = true;
				break;
			case THR_LO | YAW_CE | PIT_LO | ROL_CE:
				BFTrim = -ACC_TRIM_STEP;
				Changed = true;
				break;
			case THR_LO | YAW_LO | PIT_CE | ROL_CE:
				LRTrim = +ACC_TRIM_STEP;
				Changed = true;
				break;
			case THR_LO | YAW_HI | PIT_CE | ROL_CE:
				LRTrim = -ACC_TRIM_STEP;
				Changed = true;
				break;
			default:
				break;
			} // switch

			if (Changed) {

				AccTrimStickAdjust(BFTrim, LRTrim);
				// updated in Landing or disarm UpdateConfig();

				UpdateGyroTempComp();
				LEDToggle(ledYellowSel);
				DoBeep(1, 0);

				NowmS = mSClock();
				mSTimer(StickTimeoutmS, 100);
				mSTimer(ArmedTimeoutmS, ARMED_TIMEOUT_MS);

				SticksState = SticksChanging;
			} else {
				SticksState = MonitorSticks;
				LEDOff(ledYellowSel);
			}
		}
	}

	LEDOff(ledYellowSel);

} // DoStickProgramming

void UseDefaultParameters(uint8 DefaultPS) {
	uint16 i;

	ConfigChanged = false;

#if defined(USE_CONSERVATIVE_DEF_PARAM_LOAD)

	if (Config.CurrRevisionNo != RevisionNo)
	memset(&Config, 0, sizeof(Config)); // forces acc/mag recalibration

#else
	memset(&Config.Mission, 0, sizeof(Config.Mission));
	memset(&Config.History, 0, sizeof(Config.History));
#endif

	Config.CurrPS = 0;
	Config.CurrRevisionNo = RevisionNo;

	for (i = 0; i < MAX_PARAMETERS; i++)
		SetP(i, DefaultParams[DefaultPS].P[i]);

	SetP(KFBaroVar, DEFAULT_BARO_VARIANCE * 100.0f);
	SetP(KFAccUVar, DEFAULT_ACCU_VARIANCE * 10.0f);

	ConfigChanged = true;

	RefreshConfig();

} // UseDefaultParameters

void ConditionParameters(void) {

	F.ParametersValid = true; //unused

	ClassifyAFType();

	CurrIMUFilterType = LimitP(IMUFiltType, LPFilt, F4);

	ArmingMethod = P(ArmingMode);

	CurrTelType = P(TelemetryType);
	CurrBBLogType = P(BBLogType); //LimitP(BBLogType, logUAVX, logYaw);

#if (defined(UAVXF4V3) || defined(UAVXF4V4) || defined(UAVXF4V3BBFLASH))
	CurrRxType = P(RxType);
#else
	CurrRxType = FutabaSBusRx;
	SetP(RxType, CurrRxType);
#endif

	CurrConfig1 = P(Config1Bits);
	CurrConfig2 = P(Config2Bits);
	UAVXAirframe = LimitP(AFType, 0, AFUnknown);
	CurrESCType = LimitP(ESCType, 0, ESCUnknown);

	CurrGPSType = P(GPSProtocol);

	CurrMotorStopSel = P(MotorStopSel);
	CurrRFSensorType = P(RFSensorType);
	CurrASSensorType = P(ASSensorType);

	CurrAccLPFSel = LimitP(AccLPFSel, 1, 5);
	CurrGyroLPFSel = LimitP(GyroLPFSel, 1, 4);
	CurrYawLPFHz = LimitP(YawLPFHz, 25, 255);
	CurrServoLPFHz = LimitP(ServoLPFHz, 10, 100);

	CurrAltLPFHz = LimitP(AltLPF, 5, 20);

	DoConfigBits();

	SetPIDPeriod();

	MultiPropSense = ((P(Config2Bits) & UsePropSenseMask) != 0) ? -1.0f : 1.0f;

	UpdateParameters();

} // ConditionParameters

void LoadParameters(void) {

	ReadBlockArmFlash(CONFIG_FLASH_ADDR + 0, sizeof(Config),
			(int8 *) (&Config));

	//if ((Config.CurrRevisionNo != RevisionNo))
	if (ConfigUninitialised())
		UseDefaultParameters(0);
	else if ((ResetCause != SOFTWARE_RESET)
			|| (ResetCause != POWER_ON_POWER_DOWN_RESET)
			|| (ResetCause != UNKNOWN_RESET))
		SetP(PowerResetCause, ResetCause);

	ConditionParameters();

} // LoadParameters

