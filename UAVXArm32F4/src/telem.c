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

#include <ctype.h>

#define TELEMETRY_FLAG_BYTES  6

enum RxPacketStates {
	WaitRxSentinel,
	WaitRxBody,
	WaitRxESC,
	WaitRxCheck,
	WaitRxCopy,
	WaitUPTag,
	WaitUPLength,
	WaitUPBody,
	WaitRxTag
};

uint16 PacketsReceived[128] = { 0, };
uint8 ReceivedPacketTag, RxPacketTag, PacketRxState, RxPacketLength,
		RxPacketByteCount;
boolean PacketReceived, RxPacketError, CheckSumError;
uint8 UAVXPacket[256];
uint16 RxLengthErrors = 0;
uint16 RxCheckSumErrors = 0;
uint32 TrackGPSInvalid = 0;

uint8 TxPacketTag;
uint8 CurrTelType = UAVXDJTTelemetry;
uint8 CurrBBLogType = logUAVX;
boolean EnableGPSPassThru = false;

void TxNextLine(uint8 s) {
	TxChar(s, ASCII_CR);
	TxChar(s, ASCII_LF);
} // TxNextLine

void TxString(uint8 s, const char *pch) {
	while (*pch != (char) 0)
		TxChar(s, *pch++);
} // TxString

void TxNibble(uint8 s, uint8 v) {
	if (v > 9)
		TxChar(s, 'a' + v - 10);
	else
		TxChar(s, '0' + v);
} // TxNibble

void TxValH(uint8 s, uint8 v) {
	TxNibble(s, v >> 4);
	TxNibble(s, v & 0x0f);
} // TxValH

void TxValH16(uint8 s, int16 v) {
	TxValH(s, v >> 8);
	TxValH(s, v);
} // TxValH16

void TxValH32(uint8 s, int32 v) {
	TxValH16(s, v >> 16);
	TxValH16(s, v);
} // TxValH16

void TxBin8(uint8 s, int8 v) {
	TxChar(s, v);
} // TxBin8

void TxBin16(uint8 s, int16 v) {
	TxChar(s, v);
	TxChar(s, v >> 8);
} // TxBin16

void TxBin32(uint8 s, int32 v) {
	TxBin16(s, v);
	TxBin16(s, v >> 16);
} // TxBin16

void TxVal32(uint8 s, int32 V, int8 dp, uint8 Separator) {
	uint8 S[16];
	int8 c, Rem, zeros, i;
	int32 NewV;

	if (V < 0) {
		TxChar(s, '-');
		V = -V;
	}
	//	else
	//		TxChar(s, ' ');

	c = 0;
	do {
		NewV = V / 10;
		Rem = V - (NewV * 10);
		S[c++] = Rem + '0';
		V = NewV;
	} while (V > 0);

	if ((c < (dp + 1)) && (dp > 0)) {
		TxChar(s, '0');
		TxChar(s, '.');
	}

	zeros = (int8) dp - c - 1;
	if (zeros >= 0)
		for (i = zeros; i >= 0; i--)
			TxChar(s, '0');

	do {
		c--;
		TxChar(s, S[c]);
		if ((c == dp) && (c > 0))
			TxChar(s, '.');
	} while (c > 0);

	if (Separator != ASCII_NUL)
		TxChar(s, Separator);
} // TxVal32

void TxESCu8(uint8 s, uint8 ch) {
	if ((ch == ASCII_SOH) || (ch == ASCII_EOT) || (ch == ASCII_ESC))
		TxChar(s, ASCII_ESC);
	TxChar(s, ch);
} // TxESCu8

void TxESCi8(uint8 s, int8 b) {
	if ((b == ASCII_SOH) || (b == ASCII_EOT) || (b == ASCII_ESC))
		TxChar(s, ASCII_ESC);
	TxChar(s, b);
} // TxESCu8

void TxESCi16(uint8 s, int16 v) {
	TxESCu8(s, v & 0xff);
	TxESCu8(s, (v >> 8) & 0xff);
} // TxESCi16

void TxESCi24(uint8 s, int32 v) {
	TxESCu8(s, v & 0xff);
	TxESCu8(s, (v >> 8) & 0xff);
	TxESCu8(s, (v >> 16) & 0xff);
} // TxESCi24

void TxESCi32(uint8 s, int32 v) {
	TxESCi16(s, v & 0xffff);
	TxESCi16(s, (v >> 16) & 0xffff);
} // TxESCi32

uint8 UAVXPacketu8(uint8 p) {
	return UAVXPacket[p];
} // UAVXPacketu8

int16 UAVXPacketi8(uint8 p) {
	int16 temp;

	temp = (int8) UAVXPacket[p];
	//if (temp > 127)
	//	temp -= 256;

	return temp;
} // UAVXPacketi8

int16 UAVXPacketi16(uint8 p) {
	int16 temp;

	temp = (int16) (UAVXPacket[p + 1] << 8);
	temp |= (int16) UAVXPacket[p];

	return temp;
} // UAVXPacketi16

int32 UAVXPacketi24(uint8 p) {
	int32 temp;

	temp = ((int32) UAVXPacket[p + 2] << 24);
	temp |= ((int32) UAVXPacket[p + 1] << 16);
	temp |= (int32) UAVXPacket[p] << 8;
	temp /= 256;
	return temp;
} // UAVXPacketi24

int32 UAVXPacketi32(uint8 p) {
	int32 temp;

	temp = (int32) (UAVXPacket[p + 3] << 24);
	temp |= ((int32) UAVXPacket[p + 2] << 16);
	temp |= ((int32) UAVXPacket[p + 1] << 8);
	temp |= (int32) UAVXPacket[p];
	return temp;
} // UAVXPacketi32

void SendPacketHeader(uint8 s) {
	TxChar(s, 0xff); // synchronisation to "jolt" USART
	TxChar(s, ASCII_SOH);
	TxCheckSum[s] = 0;
} // SendPacketHeader

void SendPacketTrailer(uint8 s) {

	TxESCu8(s, TxCheckSum[s]);
	TxChar(s, ASCII_EOT);

	TxChar(s, ASCII_CR);
	TxChar(s, ASCII_LF);
} // SendPacketTrailer

void SendAckPacket(uint8 s, uint8 Tag, uint8 Reason) {

	SendPacketHeader(s);

	TxESCu8(s, UAVXAckPacketTag);
	TxESCu8(s, 2);

	TxESCu8(s, Tag); // 2
	TxESCu8(s, Reason); // 3

	SendPacketTrailer(s);
} // SendAckPacket

//______________________________________________________________________________________________

// Tx UAVX Packets

#define NAV_STATS_INTERLEAVE	10
int8 StatsNavAlternate = 0;

void ShowDrives(uint8 s) {
	real32 AvSum;
	real32 PWSamplesR;
	idx b;

	PWSamplesR = 1.0 / PWSamples;

	AvSum = 0.0f;
	for (b = 0; b < NoOfDrives; b++)
		AvSum += PWSum[b];
	AvSum *= NoOfDrivesR;

	for (b = 0; b < NoOfDrives; b++)
		PWDiagnostic[b] = Limit1((PWSum[b] - AvSum) * PWSamplesR * 100, 100);

	TxESCi8(s, CurrMaxPWMOutputs);
	for (b = 0; b < CurrMaxPWMOutputs; b++) // motor/servo channels
		TxESCi16(s, PW[b] * 1000);
	for (b = 0; b < CurrMaxPWMOutputs; b++)
		TxESCi8(s, PWDiagnostic[b]);

} // ShowDrives

void ShowAttitude(uint8 s) {
	idx a;

	for (a = Pitch; a <= Yaw; a++) {
		TxESCi16(s, A[a].P.Desired * 1000.0f);
		TxESCi16(s, Angle[a] * 1000.0f);
		TxESCi16(s, Acc[a] * 1000.0f * GRAVITY_MPS_S_R);
		TxESCi16(s, A[a].R.Desired * 1000.0f);
		TxESCi16(s, Rate[a] * 1000.0f);
	}

} // ShowAttitude

void SendNavState(uint8 s) {
	TxESCu8(s, NavState);
} // SendNavState

void SendFlightPacket(uint8 s) {
	uint8 b;

	SendPacketHeader(s);

	TxESCu8(s, UAVXFlightPacketTag);
	TxESCu8(s,
			TELEMETRY_FLAG_BYTES + 11 + 3 * 10 + 36 + 4 + 4 + 1
					+ CurrMaxPWMOutputs * 3 + 3);
	for (b = 0; b < TELEMETRY_FLAG_BYTES; b++)
		TxESCu8(s, F.AllFlags[b]);

	TxESCu8(s, State);

	TxESCi16(s, BatteryVolts * 100.0f);
	TxESCi16(s, BatteryCurrent * 100.0f);

	TxESCi16(s, Limit(BatteryChargeUsedmAH, 0, 32000));

	TxESCi16(s, RCGlitches);
	TxESCi16(s, DesiredThrottle * 1000.0f);

	ShowAttitude(s);

	TxESCi16(s, ROC * 100.0f); // cm/S
	TxESCi24(s, Altitude * 100.0f); // cm

	TxESCi16(s, CruiseThrottle * 1000.0f);

	TxESCi16(s, RangefinderAltitude * 100.0f); // cm
	TxESCi24(s, DesiredAlt * 100.0f);

	TxESCi16(s, Heading * 1000.0f);
	TxESCi16(s, DesiredHeading * 1000.0f);

	TxESCi16(s, TiltThrFFComp * 1000.0f);
	TxESCi16(s, BattThrFFComp * 1000.0f);
	TxESCi16(s, AltHoldThrComp * 1000.0f);

	TxESCi8(s, Limit(AccConfidence * 100.0f, 0, 100));
	TxESCi16(s, BaroTemperature * 100.0f);
	TxESCi24(s, BaroPressure * 10.0f);

	TxESCi24(s, KFDensityAltitude * 100.0f);

	TxESCi16(s, TrackBaroVariance * 1000.0f);
	TxESCi16(s, TrackAccUVariance * 1000.0f);

	TxESCi16(s, Make2Pi(MagHeading) * 1000.0f);

	TxESCi16(s, MPU6XXXTemperature * 10.0f); // 0.1C

	TxESCi16(s,
			Limit((100.0f * RateEnergySum) / (real32 ) RateEnergySamples, 0,
					32000));
	TxESCi16(s, RadiansToDegrees(FWGlideAngleOffsetRad) * 10.0f);

	SendNavState(s);

	ShowDrives(s);

	TxESCi24(s, mSClock());

	SendPacketTrailer(s);
} // SendFlightPacket

void SendControlPacket(uint8 s) {

	SendPacketHeader(s);

	TxESCu8(s, UAVXControlPacketTag);
	TxESCu8(s, 31 + 6 + CurrMaxPWMOutputs * 3);

	TxESCi16(s, DesiredThrottle * 1000.0f);

	ShowAttitude(s);

	TxESCu8(s, UAVXAirframe); // unused

	ShowDrives(s);

	TxESCi24(s, mSClock());

	SendPacketTrailer(s);

} // SendControlPacket

void SendNavPacket(uint8 s) {

	SendPacketHeader(s);

	TxESCu8(s, UAVXNavPacketTag);
	TxESCu8(s, 57);

	SendNavState(s);
	TxESCu8(s, AlarmState);
	TxESCu8(s, GPS.noofsats);
	TxESCu8(s, GPS.fix);

	TxESCu8(s, CurrWPNo);

	TxESCi16(s, Limit(GPS.hAcc, 0.0f, 32.0f) * 100.0f);
	TxESCi16(s, Limit(GPS.vAcc, 0.0f, 32.0f) * 100.0f);
	TxESCi16(s, Limit(GPS.sAcc, 0.0f, 32.0f) * 100.0f);
	TxESCi16(s, Limit(GPS.cAcc, 0.0f, 32.0f) * 100.0f);

	TxESCi16(s, Nav.WPBearing * 1000.0f);
	TxESCi16(s, Nav.CrossTrackE * 10.0f);

	TxESCi16(s, GPS.gspeed * 10.0f); // dM/S
	TxESCi16(s, Make2Pi(GPS.heading) * 1000.0f); // milliRadians

	TxESCi24(s, GPS.altitude * 100.0f);
	TxESCi32(s, GPS.C[NorthC].Raw); // 1.0-7 degrees
	TxESCi32(s, GPS.C[EastC].Raw);

	TxESCi32(s, Nav.C[NorthC].PosE * 10.0f); // dM
	TxESCi32(s, Nav.C[EastC].PosE * 10.0f);

	TxESCi24(s, mS[NavStateTimeoutmS] - mSClock()); // mS

	TxESCi16(s, Limit(GPSdTmS, 0, 32000)); // was MPU Temp

	TxESCi32(s, GPS.hwVersion);

	TxESCi16(s, 0); // Nav.Sensitivity * 1000.0f);

	TxESCi16(s, A[Pitch].NavCorr * 1000.0f);
	TxESCi16(s, A[Roll].NavCorr * 1000.0f);

	TxESCi16(s, RadiansToDegrees(A[Yaw].P.Error));

	SendPacketTrailer(s);

} // SendNavPacket

void SendSoaringPacket(uint8 s) {
	/*
	 if (F.Glide) {
	 SendPacketHeader(s);

	 TxESCu8(s, UAVXSoaringPacketTag);
	 TxESCu8(s, 56);

	 timemS SoaringTune.mS;
	 real32 SoaringTune.vario;
	 real32 SoaringTune.thermalstrength;
	 real32 SoaringTune.dx;
	 real32 SoaringTune.dy;
	 real32 SoaringTune.x0;
	 real32 SoaringTune.x1;
	 real32 SoaringTune.x2;
	 real32 SoaringTune.x3;
	 uint32 SoaringTune.lat;
	 uint32 SoaringTune.lon;
	 real32 SoaringTune.alt;
	 real32 SoaringTune.dx_w;
	 real32 SoaringTune.dy_w;

	 SendPacketTrailer(s);
	 }
	 */
} // SendSoaringPacket

void SendAltitudeControlPacket(uint8 s) {

	// 65536 / (50Hz x 27 bytes) -> 1 minute only if logging

	if ((State == InFlight) || (State == MonitorInstruments)) {

		BlackBoxEnabled = true;

		SendPacketHeader(s);

		TxESCu8(s, UAVXAltitudeControlPacketTag);

		TxESCu8(s, 25);

		TxESCi16(s, AccU * 1000.0f);
		TxESCi16(s, AccUBias * 1000.0f);

		TxESCi16(s, (RawDensityAltitude - OriginAltitude) * 100.0f); // raw sensor value
		TxESCi16(s, (KFDensityAltitude - OriginAltitude) * 100.0f);

		TxESCi16(s, DesiredAlt * 100.0f);

		TxESCi16(s, BaroROC * 1000.0f);
		TxESCi16(s, KFROC * 1000.0f);

		TxESCu8(s, BatteryVolts * 10.0f);
		TxESCu8(s, CruiseThrottle * 200.0f);
		TxESCu8(s, DesiredThrottle * 200.0f);

		TxESCi16(s, TiltThrFFComp * 10000.0f);
		TxESCi16(s, BattThrFFComp * 10000.0f);
		TxESCi16(s, AltHoldThrComp * 1000.00f);
		TxESCi16(s,
				(DesiredThrottle + AltHoldThrComp) * TiltThrFFComp
						* BattThrFFComp * 10000.0f);

		SendPacketTrailer(s);

		BlackBoxEnabled = false;
	}

} // SendAltitudeControlPacket

void SendAttitudeControlPacket(uint8 s, idx a) {

	// 65536 / ( 6 bytes * 250 Hz) -> 45 seconds

	static boolean TickTock = false;
	static uint32 Tick = 0;

	if ((State == InFlight) || (State == MonitorInstruments)) {

		//TickTock = !TickTock;
		//if (TickTock) {
		Tick++;
		if (Tick > 20) {

			BlackBoxEnabled = true;

			SendPacketHeader(s);

			TxESCu8(s, UAVXAttitudeControlPacketTag);

			TxESCu8(s, 3);

			TxESCi16(s, Rate[a] * 1000.0f);
			TxESCi8(s, A[a].Out * 200.0f);

			SendPacketTrailer(s);

			Tick = 0;

			BlackBoxEnabled = false;
		}
	}

} // SendAttitudeControlPacket

void SendCalibrationPacket(uint8 s) {
	idx a, b, x, y, z;

	SendPacketHeader(s);

	TxESCu8(s, UAVXCalibrationPacketTag);
	TxESCu8(s, TELEMETRY_FLAG_BYTES + 2 + 64);

	for (b = 0; b < TELEMETRY_FLAG_BYTES; b++)
		TxESCu8(s, F.AllFlags[b]);

	TxESCi16(s, Config.GyroCal.ReferenceTemp * 10.0f);

	for (a = X; a <= Z; a++) {
		TxESCi16(s,
		RadiansToDegrees(Config.GyroCal.TempGradient[a]) * GyroScale * 1000.0f);
		TxESCi16(s,
		RadiansToDegrees(Config.GyroCal.Bias[a]) * GyroScale * 1000.0f);

		TxESCi16(s,
				Config.AccCal.Scale[a] * MPU_1G * GRAVITY_MPS_S_R * 1000.0f);
		TxESCi16(s,
				Config.AccCal.Bias[a] * Config.AccCal.Scale[a] * GRAVITY_MPS_S_R
						* 1000.0f);

		TxESCi16(s, Mag[a]);
		TxESCi16(s, Config.MagCal.Bias[a] * 1000.0f);
	}

	TxESCi16(s, 1.0f / CurrPIDCycleS);
	TxESCi16(s, CurrAccLPFHz);
	TxESCi16(s, CurrGyroLPFHz);
	TxESCi16(s, CurrYawLPFHz);
	TxESCi16(s, CurrServoLPFHz);

	for (z = 0; z <= 1; z++)
		for (y = 0; y <= 1; y++)
			for (x = 0; x <= 1; x++)
				TxESCi16(s, Population[x][y][z]);
	TxESCi16(s, mm);

	SendPacketTrailer(s);
} // SendCalibrationPacket

#define MAX_GUI_DEF_PARAM_SETS 20

void SendDefAFNames(uint8 s) {
	idx p, a, len;

	for (p = 0; p < Limit(NoOfDefParamSets, 0, MAX_GUI_DEF_PARAM_SETS); p++) {

		SendPacketHeader(s);

		len = strlen(DefaultParams[p].AFName);

		TxESCu8(s, UAVXAFNamePacketTag);
		TxESCu8(s, 2 + len);

		TxESCu8(s, p);
		TxESCu8(s, len);
		for (a = 0; a < len; a++)
			TxESCi8(s, DefaultParams[p].AFName[a]);

		SendPacketTrailer(s);

		SendAckPacket(s, UAVXAFNamePacketTag, true);
	}

} // SendDefAFNames

void SendParamsPacket(uint8 s, uint8 GUIPS) {
	idx p;

	if ((State == Preflight) || (State == Ready)
			|| (State == MonitorInstruments)) {

		SendDefAFNames(s); // refresh

		if (GUIPS < NoOfDefParamSets)
			UseDefaultParameters(GUIPS);

		Config.CurrPS = 0;
		SendPacketHeader(s);

		uint16 len = strlen(Revision);

		TxESCu8(s, UAVXParamPacketTag);
		TxESCu8(s, 1 + MAX_PARAMETERS + len + 1);

		TxESCu8(s, Config.CurrPS);

		for (p = 0; p < MAX_PARAMETERS; p++)
			TxESCi8(s, Config.P[Config.CurrPS][p]);
		TxESCu8(s, len);
		for (p = 0; p < len; p++)
			TxESCu8(s, Revision[p]);

		SendPacketTrailer(s);

		SendAckPacket(s, UAVXParamPacketTag, true);

		if (GUIPS < NoOfDefParamSets) {
			Delay1mS(100);
			systemReset(false);
		}

	} else
		SendAckPacket(s, UAVXParamPacketTag, false);

} // SendParamsPacket

void SendRCChannelsPacket(uint8 s) {
	uint8 c;

	SendPacketHeader(s);

	TxESCu8(s, UAVXRCChannelsPacketTag);
	TxESCu8(s, 12 * 2 + 2 + 1);
	TxESCi16(s, Limit(RCFrameIntervaluS, 0, 32767));
	TxESCu8(s, DiscoveredRCChannels);
	for (c = 0; c < RC_MAX_GUI_CHANNELS; c++)
		TxESCi16(s, RC[c] * 1000.0f + 1000.0f);

	SendPacketTrailer(s);

} // SendRCChannelsPacket

void SendExecutionTimeStatus(uint8 s) {

	idx e;

	SendPacketHeader(s);

	TxESCu8(s, UAVXExecutionTimePacketTag);
	TxESCu8(s, 4);

	TxESCi16(s, (100.0f * execTimeuS) / CurrPIDCycleuS);
	TxESCi16(s, (100.0f * execPeakTimeuS) / CurrPIDCycleuS);

	SendPacketTrailer(s);

} // SendExecutionTimeStatus

void SendRCLinkStats(uint8 s) {

	SendPacketHeader(s);

	TxESCu8(s, UAVXLinkStatsPacketTag);
	TxESCu8(s, 8);

	TxESCi8(s, TrackerGet(&lqTracker));
	TxESCi8(s, TrackerGet(&snrTracker));
	TxESCi16(s, TrackerGet(&rssiTracker));

	TxESCi16(s, Limit(RCSignalLosses, 0, 32767));
	TxESCi16(s, Limit(RCFailsafes, 0, 32767));

	SendPacketTrailer(s);

} // SendRCLinkStats

void SendSerialPortStatus(uint8 s) {
	int16 Entries;

	SendPacketHeader(s);

	TxESCu8(s, UAVXSerialPortPacketTag);
	TxESCu8(s, 13); // 18

	TxESCu8(s, MAX_SERIAL_PORTS);
	TxESCi16(s, SERIAL_BUFFER_SIZE);

	TxESCu8(s,
			(TxOverflow[TelemetrySerial] << 1) | RxOverflow[TelemetrySerial]);
	TxESCi16(s, TxQEntries[TelemetrySerial]);
	TxESCi16(s, RxQEntries[TelemetrySerial]);
	TxQEntries[TelemetrySerial] = RxQEntries[TelemetrySerial] = 0;

	/*
	 if (F.HaveGPS) {
	 TxESCu8(s, (TxOverflow[GPSSerial] << 1) | RxOverflow[GPSSerial]);
	 TxESCi16(s, TxQEntries[GPSSerial]);
	 TxESCi16(s, RxQEntries[GPSSerial]);
	 TxQEntries[GPSSerial] = RxQEntries[GPSSerial] = 0;
	 } else {
	 TxESCu8(s, 0);
	 TxESCi16(s, 0);
	 TxESCi16(s, 0);
	 }
	 */

	TxESCu8(s, (TxOverflow[SoftSerial] << 1) | RxOverflow[SoftSerial]);
	TxESCi16(s, TxQEntries[SoftSerial]);
	TxESCi16(s, RxQEntries[SoftSerial]);
	TxQEntries[SoftSerial] = RxQEntries[SoftSerial] = 0;

	SendPacketTrailer(s);

} // SendSerialPortStatus

void SendBBPacket(uint8 s, int32 seqNo, uint8 l, int8 * B) {
	idx i;

	SendPacketHeader(s);

	TxESCu8(s, UAVXBBPacketTag);
	TxESCu8(s, l + 4 + 2);
	TxESCi32(s, seqNo);
	TxESCi16(s, l);
	for (i = 0; i < l; i++)
		TxESCu8(s, B[i]);

	SendPacketTrailer(s);

} // SendBBPacket

void SendMinPacket(uint8 s) {
	idx b;

	SendPacketHeader(s);

	TxESCu8(s, UAVXMinPacketTag);
	TxESCu8(s, 32 + TELEMETRY_FLAG_BYTES);
	for (b = 0; b < TELEMETRY_FLAG_BYTES; b++)
		TxESCu8(s, F.AllFlags[b]);

	TxESCu8(s, State);
	SendNavState(s);
	TxESCu8(s, AlarmState);

	TxESCi16(s, BatteryVolts * 100.0f);
	TxESCi16(s, BatteryCurrent * 100.0f);
	TxESCi16(s, BatteryChargeUsedmAH);

	TxESCi16(s, Angle[Roll] * 1000.0f);
	TxESCi16(s, Angle[Pitch] * 1000.0f);

	TxESCi24(s, Altitude * 100.0f);
	TxESCi16(s, ROC * 100.0f);

	TxESCi16(s, Make2Pi(Heading) * 1000.0f);

	TxESCi32(s, GPS.C[NorthC].Raw);
	TxESCi32(s, GPS.C[EastC].Raw);

	TxESCu8(s, UAVXAirframe); // unused

	TxESCi24(s, mSClock());

	SendPacketTrailer(s);

} // SendMinPacket

void SendMinimOSDPacket(uint8 s) {

	SendPacketHeader(s);

	TxESCu8(s, UAVXMinimOSDPacketTag);
	TxESCu8(s, 48);

	TxESCi16(s, BatteryVolts * 1000.0f); //2
	TxESCi16(s, BatteryCurrent * 10.0f); // ??
	TxESCi16(s, (BatteryChargeUsedmAH * 100.0) / BatteryCapacitymAH);

	TxESCi16(s, RadiansToDegrees(Angle[Roll]));
	TxESCi16(s, RadiansToDegrees(Angle[Pitch]));

	TxESCi24(s, Altitude * 100.0f);
	TxESCi24(s, DesiredAlt * 100.0f);
	TxESCi16(s, ROC * 100.0f);

	TxESCi16(s, GPS.gspeed * 10.0f);

	TxESCi16(s, RadiansToDegrees(Make2Pi(Heading)));
	TxESCi16(s, RadiansToDegrees(Make2Pi(GPS.heading)));

	TxESCi32(s, GPS.C[NorthC].Raw);
	TxESCi32(s, GPS.C[EastC].Raw);

	TxESCu8(s, GPS.noofsats);
	TxESCu8(s, GPS.fix);
	TxESCi16(s, GPS.hAcc * 100.0f);

	TxESCu8(s, CurrWPNo);
	TxESCi16(s, RadiansToDegrees(Nav.WPBearing));
	TxESCi16(s, Nav.WPDistance * 10.0f);
	TxESCi16(s, Nav.CrossTrackE * 10.0f);

	TxESCi16(s, DesiredThrottle * 100.0f);
	TxESCu8(s, Armed());
	SendNavState(s);
	TxESCu8(s, UAVXAirframe); // unused

	SendPacketTrailer(s);

} // SendMinimOSD

void SendOriginPacket(uint8 s) {
	MissionStruct * M;

	M = &Config.Mission;

	SendPacketHeader(s);

	TxESCu8(s, UAVXOriginPacketTag);
	TxESCu8(s, 14);

	TxESCu8(s, M->NoOfWayPoints); // 0

	TxESCu8(s, Limit(Nav.MaxVelocity, 1.0f, 25.0f) * 10); // 1
	TxESCi16(s, M->FenceRadius); // 2

	if (F.OriginValid) {
		TxESCi16(s, GPS.originAltitude); // 4
		TxESCi32(s, GPS.C[NorthC].OriginRaw); // 6
		TxESCi32(s, GPS.C[EastC].OriginRaw); // 10
	} else {
		TxESCi16(s, 0); // 4
		TxESCi32(s, 0); // 6
		TxESCi32(s, 0); // 10
	}

	SendPacketTrailer(s);
} // SendOriginPacket

/*
 void SendWindPacket(uint8 s) {
 idx a;

 if ((State == InFlight) && F.WindEstValid) {
 SendPacketHeader(s);

 TxESCu8(s, UAVXWindPacketTag);
 TxESCu8(s, 10);

 TxESCi16(s, Wind.Speed * 100.0f);
 TxESCi16(s, Wind.Direction * 1000.0f);

 for (a = X; a <= Z; a++)
 TxESCi16(s, Wind.Est[a] * 100.0f);

 SendPacketTrailer(s);
 }

 } // SendWindPacket
 */

void SendGuidancePacket(uint8 s) {

	if ((State == InFlight) && F.OriginValid) {

		SendPacketHeader(s);

		TxESCu8(s, UAVXGuidancePacketTag);
		TxESCu8(s, 6);

		TxESCi16(s, Nav.Distance);
		TxESCi16(s, RadiansToDegrees(Nav.Bearing));

		TxESCi8(s, RadiansToDegrees(Nav.Elevation));
		TxESCi8(s, RadiansToDegrees(Nav.Hint));

		SendPacketTrailer(s);
	}

} // SendGuidance

void SendWPPacket(uint8 s, uint8 wp) {
	MissionStruct * M;

	M = &Config.Mission;

	SendPacketHeader(s);

	TxESCu8(s, UAVXWPPacketTag);
	TxESCu8(s, 30);

	TxESCu8(s, wp);
	TxESCi32(s, M->WP[wp].LatitudeRaw); // 1e7/degree
	TxESCi32(s, M->WP[wp].LongitudeRaw);
	TxESCi16(s, M->WP[wp].Altitude);
	TxESCi16(s, M->WP[wp].VelocitydMpS); // dM/S
	TxESCi16(s, M->WP[wp].Loiter); // S
	TxESCi16(s, M->WP[wp].OrbitRadius);
	TxESCi16(s, M->WP[wp].OrbitAltitude); // M relative to Origin
	TxESCi16(s, M->WP[wp].OrbitVelocitydMpS); // dM/S

	TxESCi32(s, M->WP[wp].PulseWidthmS); // mS
	TxESCi32(s, M->WP[wp].PulsePeriodmS); // mS

	TxESCu8(s, M->WP[wp].Action);

	SendPacketTrailer(s);
} // SendMissionWPPacket

void SendMission(uint8 s) {
	uint8 wp;

	SendNavPacket(s);
	for (wp = 1; wp <= Config.Mission.NoOfWayPoints; wp++)
		SendWPPacket(s, wp);

	SendOriginPacket(s);
} // SendMission

//______________________________________________________________________________________________

//UAVX Telemetry

void SetTelemetryBaudRate(uint8 s, uint32 b) {
	static uint32 CurrTelemetryBaudRate = 42;

	if (b != CurrTelemetryBaudRate) {
		SetBaudRate(s, b);
		CurrTelemetryBaudRate = b;
	}

} // SetTelemetryBaudRate

//______________________________________________________________________________________________

// Rx UAVX Packets

void ProcessParamsPacket(uint8 s) {
	uint8 p;

	if ((State == Preflight) || (State == Ready)
			|| (State == MonitorInstruments)) { // not inflight

		for (p = 0; p < MAX_PARAMETERS; p++)
			SetP(p, UAVXPacketi8(p + 3));

		ConditionParameters();

		State = Preflight;

		SendAckPacket(s, UAVXParamPacketTag, true);
		SendParamsPacket(s, 255);

	} else
		SendAckPacket(s, UAVXParamPacketTag, false);

} // ProcessParamsPacket

void ProcessWPPacket(uint8 s) {
	uint8 wp;
	WPStructNV * W;

	wp = UAVXPacket[2];
	W = &NewNavMission.WP[wp];

	W->LatitudeRaw = UAVXPacketi32(3);
	W->LongitudeRaw = UAVXPacketi32(7);
	W->Altitude = UAVXPacketi16(11);
	W->VelocitydMpS = UAVXPacketi16(13);
	W->Loiter = UAVXPacketi16(15);

	W->OrbitRadius = UAVXPacketi16(17);
	W->OrbitAltitude = UAVXPacketi16(19);
	W->OrbitVelocitydMpS = UAVXPacketi16(21);
	W->PulseWidthmS = UAVXPacketi32(23);
	W->PulsePeriodmS = UAVXPacketi32(27);
	W->Action = UAVXPacket[31];

} // ReceiveWPPacket

void ProcessOriginPacket(uint8 s) {

	NewNavMission.NoOfWayPoints = UAVXPacket[2];
	NewNavMission.FenceRadius = UAVXPacketi16(3);

	UpdateNavMission();

} // ProcessOriginPacket

void ProcessGPSPassThru(void) {

	LEDsOff();

	Delay1mS(1000);
	EnableGPSPassThru = true;

	while (true) {
		if (SerialAvailable(TelemetrySerial)) {
			TxChar(GPSSerial, RxChar(TelemetrySerial));
			LEDToggle(ledRedSel);
		}
	}
} // ProcessGPSPassThru

void InitPollRxPacket(void) {

	RxPacketByteCount = 0;
	RxCheckSum = 0;

	RxPacketTag = UnknownPacketTag;

	RxPacketLength = 2; // set as minimum
	PacketRxState = WaitRxSentinel;
} // InitRxPollPacket

void AddToRxPacketBuffer(uint8 ch) {
	boolean RxPacketError;

	UAVXPacket[RxPacketByteCount++] = ch;
	if (RxPacketByteCount == 1) {
		RxPacketTag = ch;
		PacketRxState = WaitRxBody;
	} else if (RxPacketByteCount == 2) {
		RxPacketLength = ch;
		PacketRxState = WaitRxBody;
	} else if (RxPacketByteCount >= (RxPacketLength + 3)) {
		RxPacketError = CheckSumError = false; //TODO: zzz !((RxCheckSum == 0) || (RxCheckSum
		//== ASCII_ESC));

		if (CheckSumError)
			RxCheckSumErrors++;

		if (!RxPacketError) {
			PacketReceived = true;
			ReceivedPacketTag = RxPacketTag;
		}
		PacketRxState = WaitRxSentinel;
		//   InitPollPacket();
	} else
		PacketRxState = WaitRxBody;
} // AddToRxPacketBuffer

void ParseRxPacket(uint8 ch) {

	RxCheckSum ^= ch;
	switch (PacketRxState) {
	case WaitRxSentinel:
		if (ch == ASCII_SOH) {
			InitPollRxPacket();
			CheckSumError = false;
			PacketRxState = WaitRxBody;
		}
		break;
	case WaitRxBody:
		if (ch == ASCII_ESC)
			PacketRxState = WaitRxESC;
		else if (ch == ASCII_SOH) // unexpected start of packet
		{
			RxLengthErrors++;
			InitPollRxPacket();
			PacketRxState = WaitRxBody;
		} else if (ch == ASCII_EOT) // unexpected end of packet
		{
			RxLengthErrors++;
			PacketRxState = WaitRxSentinel;
		} else
			AddToRxPacketBuffer(ch);
		break;
	case WaitRxESC:
		AddToRxPacketBuffer(ch);
		break;
	default:
		PacketRxState = WaitRxSentinel;
		break;
	}
} // ParseRxPacket

void ProcessRxPacket(uint8 s) {

	PacketReceived = false;
	PacketsReceived[RxPacketTag]++;

	LEDOn(ledBlueSel);

	switch (RxPacketTag) {
	case UAVXRequestPacketTag:
		switch (UAVXPacket[2]) {
		case UAVXMiscPacketTag:
			switch (UAVXPacket[3]) {
			case miscCalIMU:
				CalibrateAccAndGyro(s);
				SendCalibrationPacket(s);
				break;
			case miscCalAcc:
				CalibrateAccZeros(s);
				SendCalibrationPacket(s);
				break;
			case miscCalMag:
				CalibrateMagnetometer(s);
				SendCalibrationPacket(s);
				break;
			case miscLB:
				SendAckPacket(s, miscLB, false);
				break;
			case miscBBDump:
				BBReplaySpeed = UAVXPacket[4];
				DumpBlackBox(s);
				break;
			case miscGPSPassThru:
				if (!Armed()) {
					InitiateShutdown(GPSSerialPassThru);
					State = Shutdown;
					SendAckPacket(s, miscGPSPassThru, true);
					ProcessGPSPassThru(); // requires power cycle to escape
				} else
					SendAckPacket(s, miscLB, false);
				break;
			case miscBootLoader:
				// goes to bootloader - careful with GPS and Serial Rx
				//as they can be seen as bootloader input!
				systemReset(true);
				break;
			default:
				break;
			} // switch
			break;
		case UAVXParamPacketTag:
			SendParamsPacket(s, UAVXPacket[3]);
			break;
		case UAVXMissionPacketTag:
			SendMission(s);
			break;
		case UAVXOriginPacketTag:
			SendOriginPacket(s);
			break;
		case UAVXWPPacketTag:
			SendWPPacket(s, UAVXPacket[3]);
			break;
		case UAVXMinPacketTag:
			SendMinPacket(s);
			break;
		case UAVXStatsPacketTag:
			//	SendStatsPacket(s);
			break;
		case UAVXFlightPacketTag:
			SendFlightPacket(s);
			break;
		case UAVXControlPacketTag:
			SendControlPacket(s);
			break;
		case UAVXNavPacketTag:
			SendNavPacket(s);
			break;
		default:
			SendAckPacket(s, RxPacketTag, 255);
			break;
		} // switch
		break;
	case UAVXParamPacketTag:
		ProcessParamsPacket(s);
		break;
	case UAVXOriginPacketTag:
		ProcessOriginPacket(s);
		break;
	case UAVXWPPacketTag:
		ProcessWPPacket(s);
		break;
	default:
		break;
	} // switch

	LEDOff(ledBlueSel);

} // ProcessRxPacket

void UAVXPollRx(uint8 s) {
	uint8 ch;

	if (SerialAvailable(s)) {
		ch = RxChar(s);
		ParseRxPacket(ch);
	}

	if (PacketReceived)
		ProcessRxPacket(s);

} // UAVXPollRx

void SendUAVXTelemetry(uint8 s) {
	static boolean SendFlight = true;
	timemS NowmS;

	NowmS = mSClock();

	if (NowmS >= mS[TelemetryUpdatemS]) {
		mSTimer(TelemetryUpdatemS, UAVX_TEL_INTERVAL_MS);

		if (SendFlight) {
			SendFlightPacket(s); // 78
			SendGuidancePacket(s); // 2+24
			//if (F.WindEstValid)
			//	SendWindPacket(s); // 2+10
			SendRCChannelsPacket(s); // 27 -> 105
			SendRCLinkStats(s);
		} else {
			if (CurrGPSType != NoGPS)
				SendNavPacket(s); // 2+54+4 = 60
			SendExecutionTimeStatus(s);
			SendSerialPortStatus(s);
			if ((State == Preflight) || (State == Ready) || (State == Landed))
				SendCalibrationPacket(s);
		}
		SendFlight = !SendFlight;
	}

} // UseUAVXTelemetry

void CheckTelemetry(uint8 s) {

	UAVXPollRx(s);

	if (SoftSerialTxPin.Used && (CurrRxType != CRSFRx))
		SendFrSkyTelemetry(FrSkySerial); // always send

	if ((State == InFlight) || (State == MonitorInstruments)) {
		if (CurrBBLogType == logUAVX) {
			BlackBoxEnabled = true;
			SendUAVXTelemetry(s);
			BlackBoxEnabled = false;
		}
	} else
		SendUAVXTelemetry(s);

	UpdateBlackBox();

} // CheckTelemetry

