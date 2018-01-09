// DO NOT FORMAT  DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT DO NOT FORMAT
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

// Brushless, Brushed, Aileron, Elevon


const ParamStruct_t
		DefaultParams[] = { //

						// Attitude

						{ ThrottleGainRate, { 0, 0, 40, 50} }, // 93 PID overall gain reduction above cruise

						{ MaxRollAngle, { 60, 60, 45, 45 } }, // deg 77
						{ RollAngleKp, { 25, 25, 3, 3 } }, //  03
						{ RollAngleKi, { 3, 3, 2, 2 } }, //  24
						{ RollAngleIntLimit, { 10, 10, 10, 10 } }, //  05
						{ RollRateKp, { 20, 35, 40, 40 } }, //  01
						{ RollRateKi, { 0, 0, 6, 6 } }, // 97
						{ RollRateIntLimit, { 1, 1, 5, 5 } },// 98
						{ RollRateKd, { 45, 45, 40, 40 } }, //  12
						{ MaxRollRate, { 60, 60, 60, 60 } }, // x10 deg/S 83

						{ MaxPitchAngle, { 60, 60, 45, 45 } }, // deg  75
						{ PitchAngleKp, { 25, 25, 3, 3 } }, //  08
						{ PitchAngleKi, { 3, 3, 2, 2 } }, //  25
						{ PitchAngleIntLimit, { 10, 10, 10, 10 } }, //  10
						{ PitchRateKp, { 20, 35, 40, 40 } }, //  06
						{ PitchRateKi, { 0, 0, 6, 6 } },// 99
						{ PitchRateIntLimit, { 1, 1, 5, 5 } }, // 100
						{ PitchRateKd, { 45, 45, 40, 40 } }, //  28
						{ MaxPitchRate, { 60, 60, 30, 30 } }, // x10 deg/S 84

						{ YawRateKp, { 20, 20, 10, 10 } }, //  11
						{ YawRateKi, { 0, 0, 0, 0 } },// 101
						{ YawRateIntLimit, { 1, 1, 1, 1 } }, // 102
						{ YawRateKd, { 45, 45, 0, 0 } }, // // 91
						{ MaxCompassYawRate, { 9, 9, 9, 9 } }, // 10 deg/S 89,
						{ MaxYawRate, { 36, 36, 9, 9 } }, //  *10 deg/S 64

						// Altitude Hold
						{ AltPosKp, { 10, 10, 16, 16 } }, //  07
						{ AltPosKi, { 8, 8, 12, 12 } }, //  02
						{ AltVelKp, { 12, 12, 10, 10, } }, //  30
						{ AltVelKd, { 0, 0, 0, 0 } }, //  14
						{ AltLPF, { 10, 10, 10, 10 } }, //  *10 58
						{ MaxAltHoldComp, { 10, 10, 15, 15 } }, //  % 67


						{ RFSensorType, { UnknownRF, UnknownRF, UnknownRF,
								UnknownRF } }, // 09

						// Camera (Legacy)
						{ RollCamKp, { 10, 10, 0, 0 } }, //  19c
						{ RollCamTrim, { 0, 0, 0, 0 } }, //  40c
						{ PitchCamKp, { 10, 10, 0, 0 } }, //  26c

						// Estimator
						{ StateEst, { MadgwickIMU, MadgwickIMU, MadgwickIMU,
								MadgwickIMU } }, // IMU 13
						{ MadgwickKpMag, { 50, 50, 50, 50 } }, //  32
						{ MadgwickKpAcc, { 20, 20, 20, 20 } }, //  39c
						{ AccConfSD, { 4, 4, 4, 4 } }, //  53c
						{ SensorHint, { UAVXArm32IMU, UAVXArm32IMU,
								UAVXArm32IMU, UAVXArm32IMU } }, // ,35c

						// Filters
						{ DerivativeLPFHz, {75, 75, 75, 75 } }, // 78
						{ GyroLPFHz, { 100, 100, 100, 100 } }, // 48
						{ AccLPFHz, { 20, 20, 20, 20} }, // P90,

						// Rx
						{ RCChannels, { 7, 7, 7, 7 } }, //  37c
						{ RxThrottleCh, { 1, 1, 1, 1 } }, //  17
						{ RxRollCh, { 2, 2, 2, 2 } }, //  38
						{ RxPitchCh, { 3, 3, 3, 3 } }, //  42
						{ RxYawCh, { 4, 4, 4, 4 } }, //  43
						{ RxGearCh, { 5, 5, 5, 5 } }, //  50c
						{ RxAux1Ch, { 6, 6, 6, 6 } }, //  51
						{ RxAux2Ch, { 7, 7, 7, 7 } }, //  55c
						{ RxAux3Ch, { 8, 8, 8, 8 } }, // 56
						{ RxAux4Ch, { 9, 9, 9, 9 } }, //  60
						{ RxAux5Ch, { 10, 10, 10, 10 } }, //  94
						{ RxAux6Ch, { 11, 11, 11, 11 } }, //  95
						{ RxAux7Ch, { 12, 12, 12, 12 } }, //  96
						{ ServoSense, { 0, } }, //  52c

						// Navigation
						{ GPSProtocol, { UBXBinGPS, UBXBinGPS, UBXBinGPS,
								UBXBinGPS } }, // GPSProtocol 62
						{ NavPosKp, { 20, 20, 20, 20 } }, //  57
						{ NavPosKi, { 5, 5, 5, 5 } }, //  61
						{ NavPosIntLimit, { 3, 3, 12, 12 } }, // 41

						{ NavVelKp, { 10, 10, 10, 10 } }, //  29
						{ NavMaxAngle, { 15, 15, 30, 45 } }, // 69,

						{ NavHeadingTurnout, { 60, 60, 60, 60, } }, // 79,
						{ NavCrossTrackKp, { 4, 4, 4, 4 } }, //  49

						{ NavRTHAlt, { 10, 10, 30, 30 } }, //  33
						{ BestROC, { 2, 2, 3, 3 } }, // 73
						{ MaxDescentRateDmpS, { 10, 10, 25, 25 } }, //  46
						{ DescentDelayS, { 15, 15, 15, 15 } }, //  47

						{ NavMagVar, { 13, 13, 13, 13 } }, //  34c 13 Melbourne
						{ ASSensorType, { NoASSensor, NoASSensor, NoASSensor,
								NoASSensor } }, //  72

						{ MinhAcc, { GPS_MIN_HACC * 10.0f,
								GPS_MIN_HACC * 10.0f, GPS_MIN_HACC * 10.0f,
								GPS_MIN_HACC * 10.0f } }, // 81,

						// Control

						{ TiltThrottleFF, { 0, 0, 0, 0 } }, //  % 63

						{ Horizon, { 30, 30, 0, 0 } }, //  % 31
						{ Balance, { 50, 50, 50, 50 } }, //  % 59
						{ StickHysteresis, { 2, 2, 2, 2 } }, //  % 21c
						{ EstCruiseThr, { 50, 50, 20, 20, } }, //  20c
						{ PercentIdleThr, { 10, 7, 0, 0 } }, //  23c

						{ GyroSlewRate, { 20, 20, 20, 20} }, // *100 Deg/S/S P92

						// Fixed Wing
						{ FWClimbThrottle, { 0, 0, 70, 70} }, // % 22
						{ FWMaxClimbAngle, { 60, 60, 15, 15 } }, // deg. 68
						{ FWRollPitchFF, { 0, 0, 25, 25 } }, //  % 65
						{ FWPitchThrottleFF, { 0, 0, 10, 10 } }, //  % 66
						{ FWAileronDifferential, { 0, 0, 30, 0 } }, //  % 71
						{ FWBoardPitchAngle, { 0, 0, 5, 5 } }, // deg. 82
						{ FWAileronRudderMix, { 0, 0, 15, 0 }}, // % 87
						{ FWAltSpoilerFF, { 0, 0, 50, 50 }}, // % 88

						{ FWSpoilerDecayTime, { 0, 0, 15, 15 } }, //  *10 70

						// Configuration
						{ ArmingMode, { RollStickArming, RollStickArming,
								RollStickArming, RollStickArming } }, //  04
						{ Config1Bits, {
								UseRapidDescentMask | UseRTHDescendMask | GPSToLaunchRequiredMask,
								UseRapidDescentMask | UseRTHDescendMask | GPSToLaunchRequiredMask,
								UseRapidDescentMask |  UseManualAltHoldMask,
								UseRapidDescentMask |  UseManualAltHoldMask } }, //  16c
						{ Config2Bits, {
								UseConfigRebootMask,
								UseConfigRebootMask | UseFastStartMask,
								UseConfigRebootMask | UseFastStartMask,
								UseConfigRebootMask | UseFastStartMask} }, //  74
						{ ComboPort1Config, { CPPM_GPS_M7to10, CPPM_GPS_M7to10,
								CPPM_GPS_M7to10, CPPM_GPS_M7to10 } }, //  15
#if defined(V4_BOARD)
						{	ComboPort2Config, {I2C_RF_V4, I2C_RF_V4, I2C_RF_V4,
								I2C_RF_V4}}, // 76,
#else
						{ ComboPort2Config, { I2C_RF_BatV_V3, I2C_RF_BatV_V3,
								I2C_RF_BatV_V3, I2C_RF_BatV_V3 } }, // 76,
#endif
						{ AFType, { QuadXAF, QuadXAF, AileronSpoilerFlapsAF,
								ElevonAF } }, // ,44c
#if defined(V4_BOARD)
						{	TelemetryType, {UAVXTelemetry, UAVXTelemetry,
								FrSkyV1Telemetry, MAVLinkTelemetry}}, //  45c
#else
						{ TelemetryType, { UAVXTelemetry, UAVXTelemetry,
								FrSkyV1Telemetry, MAVLinkTelemetry } }, //  45c
#endif
						{ ESCType, { ESCUnknown, ESCUnknown, ESCUnknown,
								ESCUnknown } }, //  36c
						{ WS2812Leds, { 0, 0, 0, 0 } }, // 80,

						// Battery
						{ LowVoltThres, { 102, 34, 102, 102 } }, //  0.1V
						{ BatteryCapacity, { 22, 3, 22, 22 } }, //  *100 mAH 54c
						{ CurrentScale, {100, 100,100,100}}, // x0.01 85,
						{ VoltScale, { 100, 100,100,100} }, // x0.01 86,

						// Unused


						{ Unused27, { 0, } }, // 27

						{ Unused103, { 0, } }, // 103
						{ Unused104, { 0, } }, // 104
						{ Unused105, { 0, } }, // 105
						{ Unused106, { 0, } }, // 106
						{ Unused107, { 0, } }, // 107
						{ Unused108, { 0, } }, // 108
						{ Unused109, { 0, } }, // 109
						{ Unused110, { 0, } }, // 110

						{ Unused111, { 0, } }, // 111
						{ Unused112, { 0, } }, // 112
						{ Unused113, { 0, } }, // 113
						{ Unused114, { 0, } }, // 114
						{ Unused115, { 0, } }, // 115
						{ Unused116, { 0, } }, // 116
						{ Unused117, { 0, } }, // 117
						{ Unused118, { 0, } }, // 118
						{ Unused119, { 0, } }, // 119
						{ Unused120, { 0, } }, // 120
						{ Unused121, { 0, } }, // 121
						{ Unused122, { 0, } }, // 122
						{ Unused123, { 0, } }, // 123
						{ Unused124, { 0, } }, // 124
						{ Unused125, { 0, } }, // 125
						{ Unused126, { 0, } }, // 126
						{ Unused127, { 0, } }, // 127
						{ Unused128, { 0, } } // 128

				};

const uint8 NoDefaultEntries = sizeof(DefaultParams) / sizeof(ParamStruct_t);

