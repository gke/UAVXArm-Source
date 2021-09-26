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


#ifndef _rc_h
#define _rc_h

#define RC_MAX_CHANNELS 20
#define RC_MAX_GUI_CHANNELS 12

#define RC_NO_CHANGE_TIMEOUT_MS 20000 // mS.
#define RC_INIT_FRAMES 60 // number of initial RC frames to allow filters to settle
#define RC_MARGIN_US	100 // could set zero or less for FrSky CPPM
#define RC_MIN_WIDTH_US (1000-RC_MARGIN_US) // temporarily to prevent wraparound 900
#define RC_MAX_WIDTH_US (2000+RC_MARGIN_US)
#define RCWidthOK(n) ((n > RC_MIN_WIDTH_US)&&(n < RC_MAX_WIDTH_US))

#define RC_GOOD_BUCKET_MAX 20
#define RC_GOOD_RATIO 4

#define RC_THRES_START	4 // zzz was 3
#define RC_THRES_START_STICK FromPercent(RC_THRES_START)
#define THR_MAXIMUM FromPercent(90)
#define RC_FRAME_TIMEOUT_US 25000
#define RC_SIGNAL_TIMEOUT_US  (RC_FRAME_TIMEOUT_US * 5)

#define RXBUF_SIZE	64

enum SwStates {
	SwLow, SwMiddle, SwHigh, SwUnknown
};

typedef struct  {
	uint8 uplinkRSSIAnt1;
	uint8 uplinkRSSIAnt2;
	uint8 uplinkLQ;
	int8 uplinkSNR;
	uint8 activeAntenna;
	uint8 rfMode;
	uint8 uplinkTXPower;
	uint8 downlinkRSSI;
	uint8 downlinkLQ;
	int8 downlinkSNR;
	uint8 activeAnt;
}__attribute__ ((__packed__)) rcLinkStatsStruct;

typedef struct {
    unsigned int c1  : 11;
    unsigned int c2  : 11;
    unsigned int c3  : 11;
    unsigned int c4  : 11;
    unsigned int c5  : 11;
    unsigned int c6  : 11;
    unsigned int c7  : 11;
    unsigned int c8  : 11;
    unsigned int c9  : 11;
    unsigned int c10 : 11;
    unsigned int c11 : 11;
    unsigned int c12 : 11;
    unsigned int c13 : 11;
    unsigned int c14 : 11;
    unsigned int c15 : 11;
    unsigned int c16 : 11;
    }  __attribute__((packed)) SBUSChannelStruct;

typedef struct {
    union {
    rcLinkStatsStruct ls;
	uint8 b[RXBUF_SIZE];
	SBUSChannelStruct c;
    } u;
    volatile boolean busy;
	uint8 state;
	uint8 index;
	uint8 channel;
	uint8 device;
	uint8 length;
	uint8 type;
	timeuS lastByteReceiveduS;
	timeuS frameStartuS;
} RCFrameStruct;

RCFrameStruct RCFrame;

// PPM
#define MIN_PPM_SYNC_PAUSE_US 2800 // 8x2+2=18 FrSky broken

typedef struct {
	uint8 Pin;
	timeuS PrevEdge;
	boolean PrevState;
	boolean State;
	int32 RisingEdge;
	int32 FallingEdge;
	int32 Raw;
	int32 SpekRaw;
} RCInpDefStruct;

// Spektrum

// 115200b

#define MIN_SPEK_SYNC_PAUSE_US 7500

#define SPEK_CHANNELS_PER_FRAME 7
#define SPEK_FRAME_SIZE 16

#define SPEKTRUM_CHANNEL1	0x04
#define SPEKTRUM_CHANNEL2	0x05

void CheckSerialRx(void);
void DoRCSerial(timeuS NowuS);
void DoSpektrum(void);

// Futaba SBus

// 100000 baud, 2 stop bits, even parity

#define SBUS_MIN_SYNC_PAUSE_US 4000 // 3000

#define SBUS_CHANNELS 18
#define SBUS_FRAME_SIZE 25

#define SBUS_START_BYTE	0x0f
#define SBUS_END_BYTE 0x00

#define SBUS_CHVAL_NEUTRAL 1024

#define SBUS_CH17_MASK (1<0)
#define SBUS_CH18_MASK (1<1)
#define SBUS_SIGNALLOST_MASK (1<2)
#define SBUS_FAILSAFE_MASK (1<3)

// CRSF

#define MSP_RSSI_TIMEOUT_US     1500000   // 1.5 sec
#define RX_LQ_INTERVAL_MS       200
#define RX_LQ_TIMEOUT_MS        1000

#define CRSF_CHANNELS SBUS_CHANNELS

// General
void InitRC(void);
void EnableRC(void);
void CheckRC(void);
void MapRC(void);
void CheckSticksHaveChanged(void);
void UpdateControls(void);
void CheckThrottleMoved(void);
void ReceiverTest(uint8 s);
void UpdateRCMap(void);

// ISR

void RCSerialISR(timeuS NowuS);
void RCUSARTISR(uint8 ch);

extern RCInpDefStruct RCInp[];
extern timeuS RCLastFrameuS;
extern timeuS RCSyncWidthuS;
extern timeuS RCFrameIntervaluS;
extern uint32 RCGlitches;
extern uint8 Channel;
extern int8 SignalCount;
extern uint32 RCNavFrames;
extern uint8 Map[], RMap[];
extern real32 RC[], RCp[];

extern uint8 DiscoveredRCChannels;
extern real32 StickThrottle;
extern real32 CurrMaxRollPitchStick;
extern real32 CamPitchTrim;
extern real32 ThrLow, ThrHigh, ThrNeutral;
extern real32 ThrottleMovingWindow;
extern uint8 NoOfControls;
extern int8 RCStart;
extern timemS NextNavSwUpdatemS;
extern real32 AHThrottle, AHThrottleWindow;

extern uint8 CurrRxType;
extern uint16 LostFrameCount;
extern uint8 RSSI;

extern boolean SBusFailsafe;
extern boolean SBusSignalLost;
extern boolean SBusFutabaValidFrame;

#define CRSF_BAUDRATE           420000
//#define CRSF_PORT_OPTIONS       (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define CRSF_PORT_MODE          MODE_RXTX

#define CRSF_MAX_CHANNEL        17

enum { CRSF_SYNC_BYTE = 0xC8 };

enum { CRSF_FRAME_SIZE_MAX = 64 }; // 62 bytes frame plus 2 bytes frame header(<length><type>)
enum { CRSF_PAYLOAD_SIZE_MAX = CRSF_FRAME_SIZE_MAX - 6 };

enum {
    CRSF_DISPLAYPORT_SUBCMD_UPDATE = 0x01, // transmit displayport buffer to remote
    CRSF_DISPLAYPORT_SUBCMD_CLEAR = 0X02, // clear client screen
    CRSF_DISPLAYPORT_SUBCMD_OPEN = 0x03,  // client request to open cms menu
    CRSF_DISPLAYPORT_SUBCMD_CLOSE = 0x04,  // client request to close cms menu
    CRSF_DISPLAYPORT_SUBCMD_POLL = 0x05,  // client request to poll/refresh cms menu
};

enum {
    CRSF_DISPLAYPORT_OPEN_ROWS_OFFSET = 1,
    CRSF_DISPLAYPORT_OPEN_COLS_OFFSET = 2,
};

enum {
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE = 2,
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
    CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
    CRSF_FRAME_LENGTH_ADDRESS = 1, // length of ADDRESS field
    CRSF_FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
    CRSF_FRAME_LENGTH_TYPE = 1, // length of TYPE field
    CRSF_FRAME_LENGTH_CRC = 1, // length of CRC field
    CRSF_FRAME_LENGTH_TYPE_CRC = 2, // length of TYPE and CRC fields combined
    CRSF_FRAME_LENGTH_EXT_TYPE_CRC = 4, // length of Extended Dest/Origin, TYPE and CRC fields combined
    CRSF_FRAME_LENGTH_NON_PAYLOAD = 4, // combined length of all fields except payload
};

enum {
    CRSF_FRAME_TX_MSP_FRAME_SIZE = 58,
    CRSF_FRAME_RX_MSP_FRAME_SIZE = 8,
    CRSF_FRAME_ORIGIN_DEST_SIZE = 2,
};

// Clashes with CRSF_ADDRESS_FLIGHT_CONTROLLER
#define CRSF_TELEMETRY_SYNC_BYTE  0XC8

typedef enum {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
} crsfAddress_e;

typedef enum {
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO_SENSOR = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAMETYPE_DISPLAYPORT_CMD = 0x7D, // displayport control command
} crsfFrameType_e;

typedef struct crsfFrameDef_s {
    uint8 deviceAddress;
    uint8 frameLength;
    uint8 type;
    uint8 payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
} crsfFrameDef;

typedef struct rcLinkQualityTracker_s {
    timemS lastUpdatedmS;
    uint32_t lqAccumulator;
    uint32_t lqCount;
    uint32_t lqValue;
} rcLinkQualityTrackerStruct;

typedef struct rcLinkStatistics_s {
    int16_t     uplinkRSSI;     // RSSI value in dBm
    uint8_t     uplinkLQ;       // A protocol specific measure of the link quality in [0..100]
    int8_t      uplinkSNR;      // The SNR of the uplink in dB
    uint8_t     rfMode;         // A protocol specific measure of the transmission bandwidth [2 = 150Hz, 1 = 50Hz, 0 = 4Hz]
    uint16_t    uplinkTXPower;  // power in mW
    uint8_t     activeAnt;
} rcLinkStatisticsStruct;

typedef union  {
    uint8 bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrameDef frame;
} crsfFrame_u;

extern rcLinkStatisticsStruct rcLinkStatistics;
extern rcLinkStatsStruct rcLinkStats;

void lqTrackerReset(void);
void lqTrackerAccumulate(uint16_t rawValue);
void lqTrackerSet(uint16 rawValue);
uint16_t lqTrackerGet(void);

extern rcLinkQualityTrackerStruct lqTracker;

void crsfRxWriteTelemetryData(const void *data, int len);
void crsfRxSendTelemetryData(void);

struct rxConfig_s;
struct rxRuntimeConfig_s;

#endif


