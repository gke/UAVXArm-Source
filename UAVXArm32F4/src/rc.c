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
uint32 RCFailsafes = 0;
uint32 RCSignalLosses = 0;

// Futaba SBus

// The endByte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
// See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
// and https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023

boolean RCFailsafe = false;
boolean RCSignalLost = false;
boolean FailsafePacketGenerated = false;

uint8 RSSI;

// CRSF

#define RSSI_MAX_VALUE 1023

#define CRSF_TIME_NEEDED_PER_FRAME_US   1100 // 700 ms + 400 ms for potential ad-hoc request
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811
#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef, type)

#define LQ_SIGNAL_LOST_LIMIT 25
#define LQ_FAILSAFE_LIMIT 50

TrackerStruct lqTracker, rssiTracker, snrTracker;
rcLinkStatsStruct rcLinkStats;

// Common

RCInpDefStruct RCInp[RC_MAX_CHANNELS], FS[RC_MAX_CHANNELS];
RCInpDefStruct RCCaptureInp;

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

uint8 crc8_dvb_s2(uint8 crc, unsigned char a) {
	idx i;

	crc ^= a;
	for (i = 0; i < 8; ++i)
		crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : crc << 1;

	return crc;
} // scaleRange

void GenerateFailsafePacket(void) {
	idx c;

	if (!FailsafePacketGenerated) {
		RCFailsafes++;
		F.Signal = false;
		RCSignalLost = RCFailsafe = FailsafePacketGenerated = true;

		for (c = 0; c < RC_MAX_CHANNELS; c++)
			RCInp[c].Raw = FS[c].Raw;

		SignalCount = -RC_GOOD_BUCKET_MAX;
		F.RCNewValues = true;
	}

} // GenerateFailsafePacket

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
			if (F.RCFrameOK)
				SignalCount++;
			else
				SignalCount -= RC_GOOD_RATIO;
			SignalCount = Limit1(SignalCount, RC_GOOD_BUCKET_MAX);
			F.Signal = F.RCFrameReceived = SignalCount > 0;
		}
	}

} // RCSerialISR

void ExtractAndScaleRCInp(void) {

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

} // ExtractAndScaleRCInp

// Futaba SBus

void sbusDecode(void) {

	ExtractAndScaleRCInp();

	RCInp[16].Raw = RCFrame.u.b[22] & 0b0001 ? 2000 : 1000;
	RCInp[17].Raw = RCFrame.u.b[22] & 0b0010 ? 2000 : 1000;

	RCSignalLost = (RCFrame.u.b[22] & SBUS_SIGNALLOST_MASK) != 0;
	RCFailsafe = (RCFrame.u.b[22] & SBUS_FAILSAFE_MASK) != 0;

	if (RCSignalLost)
		RCSignalLosses++;

	F.Signal = !(RCSignalLost || RCFailsafe);

	F.RCNewValues = true;

} // sbusDecode

//  CRSF/ExpressLRS rewritten from Cleanflight/iNav.

static uint8 telemetryBuf[CRSF_FRAME_SIZE_MAX];
static uint8 telemetryBufLen = 0;

// The power levels represented by uplinkTXPower above in mW (250mW added to full TX in v4.00 firmware)
const uint16 crsfPowerStates[] = { 0, 10, 25, 100, 500, 1000, 2000, 250 };

// CRSF protocol
//
// CRSF protocol uses a single wire half duplex uart connection.
// The master sends one frame every 4ms and the slave replies between two frames from the master.
//
// 420000 baud
// not inverted
// 8 Bit
// 1 Stop bit
// Big endian
// 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
// Max frame size is 64 bytes
// A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
//
// CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
//
// Every frame has the structure:
// <Device address> <Frame length> < Type> <Payload> < CRC>
//
// Device address: (uint8)
// Frame length:   length in  bytes including Type (uint8)
// Type:           (uint8)
//  <Payload>
// CRC:            (uint8)

uint8 crsfFrameCRC(void) {
	idx i;
	uint8 crc;

	// CRC includes type and payload
	crc = crc8_dvb_s2(0, RCFrame.type);
	for (i = 0; i < RCFrame.length; i++)
		crc = crc8_dvb_s2(crc, RCFrame.u.b[i]);

	return crc;
} // crsfFrameCRC

uint8 crsfDecode(void) {
	idx i;
	uint8 crc;

	switch (RCFrame.type) {
	case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:

		ExtractAndScaleRCInp();

		F.RCNewValues = true;

		if (SoftSerialTxPin.Used) // soft serial baud rate too slow? TODO:
			SendFrSkyTelemetry(FrSkySerial);

		break;
	case CRSF_FRAMETYPE_LINK_STATISTICS:

		memcpy(&rcLinkStats, &RCFrame.u.ls, sizeof(rcLinkStatsStruct)); // assume infrequent updates so save

		RCInp[17].Raw = Limit(rcLinkStats.uplinkLQ, 0, 100);

		TrackerSet(&lqTracker, rcLinkStats.uplinkLQ);
		TrackerSet(&rssiTracker,
				-1
						* (rcLinkStats.activeAntenna ?
								rcLinkStats.uplinkRSSIAnt2 :
								rcLinkStats.uplinkRSSIAnt1));
		TrackerSet(&snrTracker, rcLinkStats.uplinkSNR);

		RCSignalLost = TrackerGet(&lqTracker) < LQ_SIGNAL_LOST_LIMIT;
		RCFailsafe = TrackerGet(&lqTracker) < LQ_FAILSAFE_LIMIT;

		if (RCFailsafe)
			RCFailsafes++;
		if (RCSignalLost)
			RCSignalLosses++;

		break;
	default:
		// bad frame
		RCGlitches++;
		break;
	} // switch

} // crsfDecode

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RCUSARTISR(uint8 v) { // based on MultiWii
	static uint32 sample = 0;
	idx i;

	uint8 crc;
	static uint16 length;

	enum {
		SBusWaitSentinel,
		SBusWaitData,
		SBusWaitEnd,
		CRSFWaitLength,
		CRSFWaitType,
		CRSFWaitData,
		CRSFWaitNext
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
	case CRSFRx:
		if ((NowuS - RCFrame.frameStartuS) > CRSF_TIME_NEEDED_PER_FRAME_US) { // TODO:
			// We've received a character after max time needed to complete a frame,
			// so this must be the start of a new frame.
			RCFrame.busy = true;
			RCFrame.device = v;
			RCFrame.frameStartuS = NowuS;
			RCFrame.state = CRSFWaitLength;
		} else
			switch (RCFrame.state) {
			case CRSFWaitLength:
				RCFrame.length = v - 2;
				RCFrame.state = CRSFWaitType;
				break;
			case CRSFWaitType:
				RCFrame.type = v;
				RCFrame.index = 0;
				RCFrame.state = CRSFWaitData;
				break;
			case CRSFWaitData:
				if (RCFrame.index < RCFrame.length)
					RCFrame.u.b[RCFrame.index++] = v;
				else {
					RCFrameIntervaluS = NowuS - RCLastFrameuS;
					RCLastFrameuS = NowuS;
					crc = crsfFrameCRC();
					F.RCFrameReceived = F.Signal = (v == crc);
					if (!F.RCFrameReceived)
						RCGlitches++;

					RCFrame.busy = false;
					RCFrame.state = CRSFWaitNext;
				}
				break;
			case CRSFWaitNext:

				break;
			} // switch

		break;
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

	for (c = 0; c < RC_MAX_CHANNELS; c++)
		++Count[NewMap[c]];

	F.RCMapFail = false;
	for (c = 0; c < RC_MAX_CHANNELS; c++)
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
	RCInpDefStruct * R;

	DiscoveredRCChannels = 1;

	RCLastFrameuS = uSClock();
	RCStart = RC_INIT_FRAMES;
	NowmS = mSClock();

	memset((void *) &RCFrame, 0, sizeof(RCFrame));

	switch (CurrRxType) {
	case FutabaSBusRx:
		DiscoveredRCChannels = SBUS_CHANNELS;
		SetBaudRate(RCSerial, 100000);
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
	case CRSFRx:
		memset(&rcLinkStats, 0, sizeof(rcLinkStatsStruct));
		TrackerReset(&lqTracker);
		TrackerReset(&rssiTracker);
		TrackerReset(&snrTracker);
		DiscoveredRCChannels = CRSF_CHANNELS;
		SetBaudRate(RCSerial, 420000);
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
		FS[c].Raw = 1000;
		RCInp[c].Raw = 0;
		RC[c] = RCp[c] = 0.0f;
	}

	for (c = RollRC; c <= YawRC; c++) {
		FS[Map[c]].Raw = 1500;
		RC[c] = RCp[c] = RC_NEUTRAL;
	}

	RC[Aux1CamPitchRC] = RCp[Aux1CamPitchRC] = RC_NEUTRAL;

	FS[Map[ThrottleRC]].Raw = 1050;
	//FS[Map[RollRC]].Raw = 1500;
	//FS[Map[PitchRC]].Raw = 1500;
	//FS[Map[YawRC]].Raw = 1500;
	//FS[Map[Aux1CamPitchRC]].Raw = 1500;
	FS[Map[NavQualificationRC]].Raw = 1250; // ~50% sensitivity
	FS[Map[NavModeRC]].Raw = 2000; // force RTH

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
	idx c, cc;
	real32 Temp;

	for (c = 0; c < DiscoveredRCChannels; c++) {
		cc = RMap[c];
		RCp[cc] = RC[cc];
		Temp = (RCInp[c].Raw - 1000) * 0.001f;
		RC[cc] = RCFailsafe ? Temp : LPF1(RCp[cc], Temp, 0.75f);
	}
} // MapRC

void CheckRC(void) {

	// most modern receivers continue to emit packets even if there is o signal
	// being received. Signal can be lost if the receiver actually fails or as in
	// some cases the receiver stops producing packets after loss of signal.
	// ExpressLRS and presumably Crossbow where the link status packet gives
	// LQ, RSSI and SNR. The LQ and RSSI values got to Zero for a few packets
	// after which it seems packets are no longer emitted.
	// There also seems to be an issue of transmission being re-established if
	// the Tx is turned off then on again.

	if (uSClock() > (RCLastFrameuS + RC_SIGNAL_TIMEOUT_US))
		GenerateFailsafePacket(); // Rx stopped emitting packets
	else {
		if (F.RCFrameReceived) {
			F.RCFrameReceived = FailsafePacketGenerated = false;
			switch (CurrRxType) {
			case CPPMRx:
				F.RCNewValues = true; // no decoding needed
				break;
			case FutabaSBusRx:
				sbusDecode();
				break;
			case Spektrum1024Rx:
			case Spektrum2048Rx:
				spektrumDecode();
				break;
			case CRSFRx:
				crsfDecode();
				break;
			default:
				F.RCNewValues = F.Signal = false;
				break;
			} // switch
		}
	}

} // CheckRC

boolean ActiveCh(uint8 c) {
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

boolean ActiveAndTriggered(real32 t, uint8 r) {
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
			F.ReturnHome = F.Navigate = F.NewNavUpdate = false;
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

#if defined(PRE_20210828)
				TxSwitchArmed = ActiveAndTriggered(0.2f, NavQualificationRC);
				Nav.Sensitivity = 0.5f;
				F.AltControlEnabled = F.AltControlEnabled
				&& ActiveAndTriggered(0.45f, NavQualificationRC);
				WPNavEnabled = ActiveAndTriggered(0.7f, NavQualificationRC);
#else
				TxSwitchArmed = ActiveAndTriggered(0.1f, NavQualificationRC);
				WPNavEnabled = true; //ActiveAndTriggered(0.2f, NavQualificationRC);
				Nav.Sensitivity = Limit(RC[NavQualificationRC] - 0.1f, 0.0f,
						1.0f);
#endif

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

