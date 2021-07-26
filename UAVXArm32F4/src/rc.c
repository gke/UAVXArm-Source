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

// Spektrum

boolean SpekHiRes = false;
uint8 SpekChannelCount;

uint8 SpekChanShift;
uint8 SpekChanMask;
real32 SpekScale = 1.0f;
int32 SpekOffset = 988;
uint8 SpekFrameSize = 16;
uint16 LostFrameCount = 0;
uint8 SpekFrameNo = 0;

uint32 RCNavFrames = 0;
uint32 RC1CaptureFrames = 0;
uint32 RCGlitches = 0;

// Futaba SBus

// The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
// See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
// and https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023

boolean SBusFailsafe = false;
boolean SBusSignalLost = false;

uint8 RSSI;

// Common

RCInpDefStruct_t RCInp[RC_MAX_CHANNELS];
RCInpDefStruct_t RCCaptureInp;

timeuS RCLastFrameuS = 0;
timeuS RCSyncWidthuS = 0;
timeuS RCFrameIntervaluS = 0;
uint8 Channel = 0;

int8 SignalCount = RC_GOOD_BUCKET_MAX;

uint8 Map[RC_MAX_CHANNELS], RMap[RC_MAX_CHANNELS];
real32 RC[RC_MAX_CHANNELS], RCp[RC_MAX_CHANNELS];

uint8 DiscoveredRCChannels = 4; // used by PPM/CPPM

real32 StickThrottle;
real32 CamPitchTrim;
real32 ThrLow, ThrHigh, ThrNeutral;
real32 ThrottleMovingWindow;
real32 CurrMaxRollPitchStick;
int8 RCStart;
timemS NextNavSwUpdatemS = 0;
real32 AHThrottle, AHThrottleWindow;

uint8 CurrRxType = UnknownRx;

void EnableRC(void) {

	if (CurrRxType != CPPMRx)
		RxEnabled[RCSerial] = true;

} // EnableRC

void RCSerialISR(timeuS TimerVal) {
	int32 Temp;
	timeuS NowuS;
	int16 Width;

	NowuS = uSClock();
	Temp = RCInp[0].PrevEdge;
	if (TimerVal < Temp)
		Temp -= (int32) 0x0000ffff;
	Width = (TimerVal - Temp);
	RCInp[0].PrevEdge = TimerVal;

	if (Width > (int32) MIN_PPM_SYNC_PAUSE_US) { // A pause  > 5ms
		DiscoveredRCChannels = Channel;

		Channel = 0; // Sync pulse detected - next CH is CH1
		RCSyncWidthuS = Width;
		RCFrameIntervaluS = NowuS - RCLastFrameuS;
		RCLastFrameuS = NowuS;

		F.RCFrameOK = true;
		F.RCNewValues = false;
	} else {

		if (RCWidthOK(Width))
			RCInp[Channel].Raw = Width;
		else {
			RCGlitches++;
			F.RCFrameOK = false;
		}

		// MUST demand rock solid RC frames for autonomous functions not
		// to be cancelled by noise-generated partially correct frames
		if (++Channel >= DiscoveredRCChannels) {
			F.RCNewValues = F.RCFrameOK;
			if (F.RCNewValues)
				SignalCount++;
			else
				SignalCount -= RC_GOOD_RATIO;
			SignalCount = Limit1(SignalCount, RC_GOOD_BUCKET_MAX);
			F.Signal = SignalCount > 0;
		}
	}

} // RCSerialISR

// Futaba SBus

void sbusDecode(void) {
	idx i;

	RCInp[0].Raw = RCFrame.u.c.c1;
	RCInp[1].Raw = RCFrame.u.c.c2;
	RCInp[2].Raw = RCFrame.u.c.c3; // Futaba Throttle
	RCInp[3].Raw = RCFrame.u.c.c4;
	RCInp[4].Raw = RCFrame.u.c.c5;
	RCInp[5].Raw = RCFrame.u.c.c6;
	RCInp[6].Raw = RCFrame.u.c.c7;
	RCInp[7].Raw = RCFrame.u.c.c8;
	RCInp[8].Raw = RCFrame.u.c.c9;
	RCInp[9].Raw = RCFrame.u.c.c10;
	RCInp[10].Raw = RCFrame.u.c.c11;
	RCInp[11].Raw = RCFrame.u.c.c12;
	RCInp[12].Raw = RCFrame.u.c.c13;
	RCInp[13].Raw = RCFrame.u.c.c14;
	RCInp[14].Raw = RCFrame.u.c.c15;
	RCInp[15].Raw = RCFrame.u.c.c16;

	for (i = 0; i < 16; i++)
		RCInp[i].Raw = (real32) RCInp[i].Raw * 0.625f + 880;
	//RCInp[i].Raw = Limit((real32)RCInp[i].Raw * 0.625f + 880, RC_MIN_WIDTH_US, RC_MAX_WIDTH_US);

	RCInp[16].Raw = RCFrame.u.b[22] & 0b0001 ? 2000 : 1000;
	RCInp[17].Raw = RCFrame.u.b[22] & 0b0010 ? 2000 : 1000;

	SBusSignalLost = (RCFrame.u.b[22] & SBUS_SIGNALLOST_MASK) != 0;
	SBusFailsafe = (RCFrame.u.b[22] & SBUS_FAILSAFE_MASK) != 0;

	F.Signal = !(SBusSignalLost || SBusFailsafe);

	if (CurrRxType == FrSkyFBusRx)
		FrSkyDLinkuS = RCLastFrameuS;

	F.RCNewValues = true;

} // sbusDecode

void RCUSARTISR(uint8 v) { // based on MultiWii

	enum {
		SBusWaitSentinel, SBusWaitData, SBusWaitEnd
	};

	timeuS IntervaluS, NowuS;

	NowuS = uSClock();
	IntervaluS = NowuS - RCFrame.lastByteReceiveduS;
	RCFrame.lastByteReceiveduS = NowuS;

	switch (CurrRxType) {

	case FutabaSBusRx:

		if (IntervaluS > SBUS_MIN_SYNC_PAUSE_US) {
			RCSyncWidthuS = IntervaluS;
			RCFrame.index = 0;
			RCFrame.state = SBusWaitSentinel;
		}

		switch (RCFrame.state) {
		case SBusWaitSentinel:
			if ((RCFrame.index == 0) && (v == SBUS_START_BYTE)) {
				RCFrame.state = SBusWaitData;
				RCFrame.index = 0;
			}
			break;
		case SBusWaitData:
			RCFrame.u.b[RCFrame.index++] = v;
			if (RCFrame.index == 23)
				RCFrame.state = SBusWaitEnd;
			break;
		case SBusWaitEnd:
			F.RCFrameReceived = true;
			RCFrameIntervaluS = NowuS - RCLastFrameuS;
			RCLastFrameuS = NowuS;
			RCFrame.index = 0;
			RCFrame.state = SBusWaitSentinel;
			break;
		}
		break;
	case Spektrum1024Rx:
	case Spektrum2048Rx:
		if (IntervaluS > (timeuS) MIN_SPEK_SYNC_PAUSE_US) {
			RCSyncWidthuS = IntervaluS;
			RCFrame.index = 0;
		}

		RCFrame.u.b[RCFrame.index++] = v;
		if (RCFrame.index >= SPEK_FRAME_SIZE) {
			RCFrameIntervaluS = NowuS - RCLastFrameuS;
			RCLastFrameuS = NowuS;
			F.RCFrameReceived = F.Signal = true;
		}
		break;

	case FrSkyFBusRx:
		/*
		 RxFrSkySPort(v);
		 if (FrSkyPacketReceived) {
		 if (FrSkyPacketTag == 0) {

		 memcpy(&RCFrame.u.b[0], &FrSkyPacket[0], 24);

		 RSSI = RCFrame.u.b[23];

		 CheckSBusFrame(NowuS);

		 } else {
		 if (FrSkyPacketTag == 1) {
		 //	TODO: RxCTS[RCSerial] = true;
		 //	TxFrSkySPort(RCSerial);
		 //	RxCTS[RCSerial] = false;
		 }
		 }
		 }
		 break;
		 */
	default:
		break;
	} // switch

} // RCUSARTISR

// Code-based Spektrum satellite receiver binding for the HobbyKing Pocket Quad
// Spektrum binding code due to Andrew L.
// navigation07@gmail.com

// Merge idea due to davidea using standard bind link between GND and THR at startup

// Bind Mode Table:
// 2 low pulses: DSM2 1024/22ms
// 3 low pulses: no result
// 4 low pulses: DSM2 2048/11ms
// 5 low pulses: no result
// 6 low pulses: DSMX 22ms
// 7 low pulses: no result
// 8 low pulses: DSMX 11ms

#if (SPEKTRUM == 1024)
#define SPEK_BIND_PULSES 2
#else
#define SPEK_BIND_PULSES 4
#endif

#if defined(SPEK_BIND)

void doSpektrumBinding(void) {
	uint8 pulse;

	pinMode(7, INPUT); // THR pin as input
	DigitalWrite(7, HIGH);// turn on pullup resistors

	if (!DigitalRead(7)) {

		pinMode(0, OUTPUT); // Tx pin for satellite
		DigitalWrite(0, HIGH);

		pinMode(0, OUTPUT);

		DigitalWrite(0, HIGH);
		delayMicroseconds(116);

		for (pulse = 0; pulse < SPEK_BIND_PULSES; pulse++) {
			DigitalWrite(0, LOW);
			delayMicroseconds(116);
			DigitalWrite(0, HIGH);
			delayMicroseconds(116);
		}

		pinMode(0, INPUT);
	}
} // checkSpektrumBinding

#endif // SPEK_BIND
// Number of low pulses sent to satellite receivers for binding
#define MASTER_RX_PULSES 		5
#define SLAVE_RX_PULSES 		6

void DoSpektrumBind(void) {
	idx i;
	PinDef p;
	/*
	 p.Port = SerialPorts[RCSerial].Port;
	 p.Pin = SerialPorts[RCSerial].RxPin;
	 p.Mode = GPIO_Mode_OUT;
	 p.OType = GPIO_OType_PP;
	 p.PuPd = GPIO_PuPd_UP;

	 InitPin(&p);
	 // need to power the Rx off one of the pins so power up can be controlled.
	 DigitalWrite(&p, true);

	 Delay1mS(61); // let satellites settle after power up

	 for (i = 0; i < MASTER_RX_PULSES; i++) {
	 DigitalWrite(&p, false);
	 Delay1uS(120);
	 DigitalWrite(&p, true);
	 Delay1uS(120);
	 }

	 InitSerialPort(RCSerial, true, false);

	 while (!F.RCFrameReceived)
	 CheckSerialRx();
	 */
} // DoSpektrumBind

void spektrumDecode(void) {
	idx i;
	int16 v;

	for (i = 2; i < SPEK_FRAME_SIZE; i += 2)
		if ((RCFrame.u.b[i] & RCFrame.u.b[i + 1]) != 0xff) {

			SpekFrameNo = (i == 2) && ((RCFrame.u.b[i] >> 7) == 1);
			Channel = (RCFrame.u.b[i] >> SpekChanShift) & 0x0f;
			if ((Channel + 1) > DiscoveredRCChannels)
				DiscoveredRCChannels = Channel + 1;
			v = ((uint32) (RCFrame.u.b[i] & SpekChanMask) << 8)
					| RCFrame.u.b[i + 1];

			RCInp[Channel].Raw = (v - SpekOffset) * SpekScale + 1500;
		}

	LostFrameCount = ((uint16) RCFrame.u.b[0] << 8) //TODO:???
	| RCFrame.u.b[1];

	F.RCNewValues = true;

} // spektrumDecode

void UpdateRCMap(void) {
	uint8 c;
	uint8 Count[RC_MAX_CHANNELS];
	uint8 NewMap[RC_MAX_CHANNELS];

	for (c = 0; c < RC_MAX_CHANNELS; c++) {
		NewMap[c] = c;
		Count[c] = 0;
	}

	NewMap[ThrottleRC] = P(RxThrottleCh);
	NewMap[RollRC] = P(RxRollCh);
	NewMap[PitchRC] = P(RxPitchCh);
	NewMap[YawRC] = P(RxYawCh);

	NewMap[NavModeRC] = P(RxGearCh);
	NewMap[AttitudeModeRC] = P(RxAux1Ch);
	NewMap[NavQualificationRC] = P(RxAux2Ch);
	NewMap[Aux1CamPitchRC] = P(RxAux3Ch);
	NewMap[Aux2RC] = P(RxAux4Ch);
	NewMap[TransitionRC] = P(RxAux5Ch);
	NewMap[PassThruRC] = P(RxAux6Ch);
	NewMap[Unused11RC] = P(RxAux7Ch);

	for (c = 0; c < RC_MAX_GUI_CHANNELS; c++)
		++Count[NewMap[c]];

	F.RCMapFail = false;
	for (c = 0; c < RC_MAX_GUI_CHANNELS; c++)
		F.RCMapFail = F.RCMapFail || (Count[c] != 1);

	if (!F.RCMapFail) {
		for (c = 0; c < RC_MAX_CHANNELS; c++)
			Map[c] = NewMap[c];
		for (c = 0; c < RC_MAX_CHANNELS; c++)
			RMap[Map[c]] = c;
	}

} // UpdateRCMap

void InitRC(void) {
	timemS NowmS;
	uint8 c;
	RCInpDefStruct_t * R;

	DiscoveredRCChannels = 1;

	RCLastFrameuS = uSClock();
	RCStart = RC_INIT_FRAMES;
	NowmS = mSClock();

	memset((void *) &RCFrame, 0, sizeof(RCFrame));

	switch (CurrRxType) {
	case FutabaSBusRx:
		DiscoveredRCChannels = SBUS_CHANNELS;
		RxEnabled[RCSerial] = true;
		break;
	case Spektrum1024Rx:
		SpekChanShift = 2;
		SpekChanMask = 0x03;
		SpekScale = 1.2f;
		SpekOffset = 1500 - 988;
		RxEnabled[RCSerial] = true;
		break;
	case Spektrum2048Rx:
		SpekChanShift = 3;
		SpekChanMask = 0x07;
		SpekScale = 0.6f;
		SpekOffset = (1500 - 988) * 2;
		RxEnabled[RCSerial] = true;
		break;
	case FrSkyFBusRx:
		DiscoveredRCChannels = SBUS_CHANNELS;
		RxCTS[RCSerial] = false;
		SetBaudRate(RCSerial, 115200);
		RxEnabled[RCSerial] = true;
		break;
	default:
		RxEnabled[RCSerial] = false;
		break;
	} // switch

	//DoSpektrumBind();

	UpdateRCMap();

	R = &RCCaptureInp;
	R->State = true;
	R->PrevEdge = R->Raw = R->FallingEdge = R->RisingEdge = 0;

	for (c = 0; c < RC_MAX_CHANNELS; c++) {
		R = &RCInp[c];

		R->State = true;
		R->PrevEdge = R->Raw = R->FallingEdge = R->RisingEdge = 0;

		RC[c] = RCp[c] = 0;
	}

	for (c = RollRC; c <= YawRC; c++)
		RC[c] = RCp[c] = RC_NEUTRAL;
	RC[Aux1CamPitchRC] = RCp[Aux1CamPitchRC] = RC_NEUTRAL;

	CamPitchTrim = 0;
	F.ReturnHome = F.Navigate = F.AltControlEnabled = false;

	mS[StickChangeUpdatemS] = NowmS;
	mSTimer(RxFailsafeTimeoutmS, RC_NO_CHANGE_TIMEOUT_MS);

	StickThrottle = 0.0f;

	Channel = 0;

	SignalCount = -RC_GOOD_BUCKET_MAX;
	F.Signal = F.RCNewValues = false;

	NavSwState = 0;

} // InitRC

void MapRC(void) { // re-maps captured PPM to Rx channel sequence
	uint8 c, cc;
	real32 Temp;

	for (c = 0; c < DiscoveredRCChannels; c++) {
		cc = RMap[c];
		RCp[cc] = RC[cc];
		Temp = (RCInp[c].Raw - 1000) * 0.001f;
		RC[cc] = LPF1(RCp[cc], Temp, 0.75f);
	}
} // MapRC

void CheckRC(void) {

	if (CurrRxType != CPPMRx)
		if (F.RCFrameReceived) {
			F.RCFrameReceived = false;
			switch (CurrRxType) {
			case FrSkyFBusRx:
			case FutabaSBusRx:
				sbusDecode();
				break;
			case Spektrum1024Rx:
			case Spektrum2048Rx:
				spektrumDecode();
				break;
			default:
				F.RCNewValues = F.Signal = false;
				break;
			} // switch
		}

	if (uSClock() > (RCLastFrameuS + RC_SIGNAL_TIMEOUT_US)) {
		F.Signal = false;
		SignalCount = -RC_GOOD_BUCKET_MAX;
	}

} // CheckRC

inline boolean ActiveCh(uint8 c) {
	return DiscoveredRCChannels > Map[c];
} // ActiveCh

void CheckThrottleMoved(void) {
	static real32 StickThrottleP = 0.0f;
	real32 t;

	if (mSTimeout(ThrottleUpdatemS)) {
		mSTimer(ThrottleUpdatemS, AH_THR_UPDATE_MS);
		t = StickThrottle - StickThrottleP;
		F.ThrottleMoving = (Abs(t) > ThrottleMovingWindow);
		StickThrottleP = StickThrottle;
	}

} // CheckThrottleMoved

inline boolean ActiveAndTriggered(real32 t, uint8 r) {
	return ActiveCh(r) && (RC[r] > t);
} // ActiveAndTriggered

void UpdateControls(void) {

	CheckRC();

	if (F.RCNewValues) {
		F.RCNewValues = false;

		MapRC(); // re-map channel order for specific Tx

		//_________________________________________________________________________________________

		// Attitude

		// normalise from 0-1.0 -> -1.0-1.0
		A[Roll].Stick = (RC[RollRC] - RC_NEUTRAL) * 2.0f;
		A[Pitch].Stick = (RC[PitchRC] - RC_NEUTRAL) * 2.0f;
		A[Yaw].Stick = (RC[YawRC] - RC_NEUTRAL) * 2.0f;

		CurrMaxRollPitchStick = Max(Abs(A[Roll].Stick), Abs(A[Pitch].Stick));

		F.AttitudeHold = CurrMaxRollPitchStick < ATTITUDE_HOLD_LIMIT_STICK;

		//_________________________________________________________________________________________

		// Switch Processing

		StickThrottle = RC[ThrottleRC];
		F.ThrottleOpen = StickThrottle >= RC_THRES_START_STICK;

		CheckThrottleMoved();

		F.PassThru = ActiveAndTriggered(0.7f, PassThruRC);

		if (ActiveCh(NavModeRC)) {
			NavSwState = Limit((uint8 )(RC[NavModeRC] * 3.0f), SwLow, SwHigh);
			if (NavSwState >= SwMiddle)
				RCNavFrames++;
		} else {
			NavSwState = SwLow;
			F.ReturnHome = F.Navigate = F.NavigationEnabled = false;
		}

		CamPitchTrim =
				ActiveCh(Aux1CamPitchRC) ? RC[Aux1CamPitchRC] - RC_NEUTRAL : 0;

		// COMPLICATED switching for arming, AH WP nav etc ****************

		F.AltControlEnabled = State == InFlight;

		if (ActiveCh(NavQualificationRC))
			if (ArmingMethod == SwitchArming) {
				TxSwitchArmed = false;
				Nav.Sensitivity = Limit(RC[NavQualificationRC], 0, 1.0f);
				F.AltControlEnabled = F.AltControlEnabled
						&& ActiveAndTriggered(NAV_ALT_THRESHOLD_STICK,
								NavQualificationRC);
				EnableWPNav();
			} else {
				Nav.Sensitivity = F.Emulation ? 0.5f : 1.0f;
				TxSwitchArmed = ActiveAndTriggered(0.2f, NavQualificationRC);
				F.AltControlEnabled = F.AltControlEnabled
						&& ActiveAndTriggered(0.45f, NavQualificationRC);
				WPNavEnabled = ActiveAndTriggered(0.7f, NavQualificationRC);
			}
		else {
			TxSwitchArmed = false;
			Nav.Sensitivity = F.Emulation ? 0.5f : 1.0f;
			F.AltControlEnabled = true;
			EnableWPNav();
		}

		PW[Aux1CamPitchC] =
				(ActiveCh(Aux1CamPitchRC)) ? RC[Aux1CamPitchRC] : 0.5f;
		PW[Aux2C] = (ActiveCh(Aux2RC)) ? RC[Aux2RC] : 0.0f;

		UpdateRTHSwState();

		if (NavState != PIC)
			AttitudeMode = AngleMode;
		else if ((NavSwState == SwMiddle) || (NavSwState == SwHigh))
			AttitudeMode = AngleMode;
		else if (ActiveCh(AttitudeModeRC))
			AttitudeMode = Limit((uint8 )(RC[AttitudeModeRC] * 3.0f), AngleMode,
					RateMode); // captures horizon mode
		else
			AttitudeMode = AngleMode;

		F.UsingAngleControl = AttitudeMode == AngleMode;

		// END COMPLICATED switching for arming, AH WP nav etc ****************

		VTOLMode = ActiveAndTriggered(0.5f, TransitionRC)
				&& (UAVXAirframe == VTOLAF) && !F.PassThru;

		//_________________________________________________________________________________________

		// Rx has gone to failsafe

		if (RCStart == 0)
			F.NewCommands = true;
		else
			RCStart--;

		Tune();
	}

} // UpdateControls

