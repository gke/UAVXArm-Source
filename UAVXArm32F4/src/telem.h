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


#ifndef _telemetry_h
#define _telemetry_h

enum inflightLogs {logUAVX, logAltitude, logPitch, logRoll, logYaw };

enum MiscComms {
	miscCalIMU,
	miscCalMag,
	miscLB,
	miscUnused,
	miscBBDump,
	miscGPSPassThru,
	miscCalAcc,
	miscCalGyro,
	miscBootLoader
};


enum PacketTags {
	//Original NavLev Autopilot
	UnknownPacketTag = 0,
	LevPacketTag,
	NavPacketTag,
	MicropilotPacketTag,
	WayPacketTag,
	AirframePacketTag,
	NavUpdatePacketTag,
	BasicPacketTag,
	RestartPacketTag,
	TrimblePacketTag,
	MessagePacketTag,
	EnvironmentPacketTag,
	BeaconPacketTag,

	// UAVX
	UAVXFlightPacketTag,
	UAVXNavPacketTag,
	UAVXStatsPacketTag,
	UAVXControlPacketTag,
	UAVXParamPacketTag,
	UAVXMinPacketTag,
	UAVXOriginPacketTag,
	UAVXWPPacketTag,
	UAVXMissionPacketTag,
	UAVXRCChannelsPacketTag,

	UAVXRequestPacketTag = 50,
	UAVXAckPacketTag = 51,
	UAVXMiscPacketTag = 52,
	UAVXNoisePacketTag = 53,
	UAVXBBPacketTag = 54,
	unusedUAVXInertialPacketTag = 55,
	UAVXMinimOSDPacketTag = 56,
	UAVXTuningPacketTag = 57,
	UAVXUKFPacketTag = 58,
	UAVXGuidancePacketTag = 59,
	UAVXAltitudeControlPacketTag = 60,
	UAVXSoaringPacketTag = 61,
	UAVXCalibrationPacketTag = 62,
	UAVXAFNamePacketTag = 63,
	UAVXWindPacketTag = 64,
	UAVXTrackPacketTag = 65,
	UAVXSerialPortPacketTag = 66,
	UAVXExecutionTimePacketTag = 67,
	UAVXAttitudeControlPacketTag = 68,

	FrSkyPacketTag = 99
};

enum TelemetryTypes {
	UAVXDJTTelemetry,
	iNavLUATelemetry,
	NoTelemetry,
	u4Telemetry,
	u5Telemetry,
	u6Telemetry,
	u7Telemetry,
	u8Telemetry
};

void TxString(uint8 s, const char *);
void TxNextLine(uint8 s);
void TxValH(uint8 s, uint8);
void TxValH32(uint8 s, int32 v);
void TxVal32(uint8 s, int32, int8, uint8);

void TxBin8(uint8 s, int8 v);
void TxBin16(uint8 s, int16 v);
void TxBin32(uint8 s, int32 v);

void InitPollRxPacket(void);
void SendBBPacket(uint8 s, int32 seqNo, uint8 l, int8 * b);

void SendDefAFNames(uint8 s);
void SendFlightPacket(uint8 s);
void SendCalibrationPacket(uint8 s);
void SendAckPacket(uint8 s, uint8 Tag, uint8 Reason);
void SendMinPacket(uint8 s);
void SendAltitudeControlPacket(uint8 s);
void SendAttitudeControlPacket(uint8 s, idx a);
void CheckTelemetry(uint8 s);

void SetTelemetryBaudRate(uint8 s, uint32 b);

extern uint8 CurrTelType;
extern uint32 TrackGPSInvalid;
extern uint8 CurrBBLogType;

#endif

