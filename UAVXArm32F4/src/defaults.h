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

#if defined(V4_BOARD)
						{ PIDTimeSel, { 1, 1, 1, 1} }, // P92
#else
						{ PIDTimeSel, { 0, 0, 0, 0} }, // P92
#endif

						{ TuneParamSel, { NoTuning, NoTuning, NoTuning,
								NoTuning } }, // 78

						// Attitude

						{ ThrottleGainRate, { 0, 0, 0, 0} }, // 93 PID overall gain above throttle cruise

						{ MaxRollAngle, { 60, 60, 45, 60 } }, // deg 77
						{ RollAngleKp, { 25, 25, 20, 25 } }, //  03
						{ RollAngleKi, { 3, 3, 1, 1 } }, //  24
						{ RollIntLimit, { 10, 10, 10, 10 } }, //  05
						{ RollRateKp, { 20, 40, 6, 6 } }, //  01
						{ RollRateKd, { 45, 75, 0, 0 } }, //  12
						{ MaxRollRate, { 60, 60, 60, 60 } }, // x10 deg/S 83

						{ MaxPitchAngle, { 60, 60, 45, 60 } }, // deg  75
						{ PitchAngleKp, { 25, 25, 20, 25 } }, //  08
						{ PitchAngleKi, { 3, 3, 1, 1 } }, //  25
						{ PitchIntLimit, { 10, 10, 10, 10 } }, //  10
						{ PitchRateKp, { 20, 40, 6, 6 } }, //  06
						{ PitchRateKd, { 45, 75, 0, 0 } }, //  28
						{ MaxPitchRate, { 60, 60, 30, 30 } }, // x10 deg/S 84

						{ YawRateKp, { 20, 20, 10, 10 } }, //  11
						{ YawRateKd, { 45, 45, 0, 0 } }, // // 91
						{ MaxCompassYawRate, { 9, 9, 24, 24 } }, // 10 deg/S 89,
						{ MaxYawRate, { 36, 36, 9, 9 } }, //  *10 deg/S 64

						// Altitude Hold
						{ AltPosKp, { 16, 16, 16, 16 } }, //  07
						{ AltPosKi, { 12, 12, 12, 12 } }, //  02
						{ AltVelKp, { 9, 9, 9, 9, } }, //  30
						{ AltVelKd, { 0, 0, 0, 0 } }, //  14
						{ AltLPF, { 10, 10, 10, 10 } }, //  *10 58
						{ MaxAltHoldComp, { 40, 40, 15, 15 } }, //  % 67
						{ AltCompDecayTime, { 15, 15, 15, 15 } }, //  *0.1S 22

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

						{ GyroLPFSel, { MPU_RA_DLPF_BW_98, MPU_RA_DLPF_BW_98,
								MPU_RA_DLPF_BW_98, MPU_RA_DLPF_BW_98 } }, // 48

						{ AccLPFSel, { 4, 4, 4, 4} }, // P90,

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

						{ NavRTHAlt, { 15, 10, 30, 30 } }, //  33
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

						// Fixed Wing
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
						{ Config1Bits, { UseRapidDescentMask
								| UseRTHDescendMask | GPSToLaunchRequiredMask,
								UseRapidDescentMask | UseRTHDescendMask
										| GPSToLaunchRequiredMask,
								UseRapidDescentMask |  UseManualAltHoldMask,
								UseRapidDescentMask |  UseManualAltHoldMask } }, //  16c
						{ Config2Bits, { UseConfigRebootMask,
								UseConfigRebootMask, UseFastStartMask
										| UseConfigRebootMask, UseFastStartMask
										| UseConfigRebootMask } }, //  74
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
								FrSkyTelemetry, MAVLinkTelemetry}}, //  45c
#else
						{ TelemetryType, { UAVXTelemetry, UAVXTelemetry,
								FrSkyTelemetry, MAVLinkTelemetry } }, //  45c
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

						{ UnusedYawAngleKp, { 0, } } // (Compass) 27

				};

const uint8 NoDefaultEntries = sizeof(DefaultParams) / sizeof(ParamStruct_t);

