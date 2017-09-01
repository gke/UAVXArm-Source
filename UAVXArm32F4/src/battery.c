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

#define THR_MAX_CURRENT 32 // Amps total current at full throttle for estimated mAH
#define CURRENT_SENSOR_MAX 50L // Amps range of current sensor - used for estimated consumption - no actual sensor yet.

#if defined(VOLT_MEASUREMENT_ONBOARD)
#define VOLTS_SCALE	((3.3f*(10.0f+2.2f))/2.2f)
#else
#define VOLTS_SCALE	(3.3f)
#endif
#define POWER_SCALE (3.3f)

real32 BatteryVolts, BatterySagR, StartupVolts, BatteryCurrent, BatteryVoltsLimit, BatteryChargeUsedmAH;
real32 VoltScaleTrim, CurrentScaleTrim;
real32 BatteryCurrentADCZero;

real32 BatteryCapacitymAH;
uint8 BatteryCellCount = 3;

real32 MockBattery(void) {
	real32 CellVolts;

	if (BatteryChargeUsedmAH < (BatteryCapacitymAH * 0.05f))
		CellVolts = 4.2 - 0.2f * BatteryChargeUsedmAH / (BatteryCapacitymAH
				* 0.05f);
	else if (BatteryChargeUsedmAH < (BatteryCapacitymAH * 95.0f))
		CellVolts = 4.0 - 1.0f * (BatteryChargeUsedmAH - BatteryCapacitymAH
				* 0.05f) / (BatteryCapacitymAH * 0.90f);
	else
		CellVolts = 3.0 - 3.0f * (BatteryChargeUsedmAH - BatteryCapacitymAH
				* 0.95f) / (BatteryCapacitymAH * 0.05f);

	return (CellVolts);

} // MockBattery


void CheckBatteries(void) {
	enum lvcStates {
		lvcStart = 0, lvcMonitor, lvcWarning, lvcWait, lvcLand
	};
	uint32 NowmS;
	uint16 dTmS;

	NowmS = mSClock();

	if ((NowmS >= mS[BatteryUpdate])) {
		mSTimer(NowmS, BatteryUpdate, BATTERY_UPDATE_MS);

		if (F.Emulation) {
			BatteryCurrent = (DesiredThrottle + AltComp) * THR_MAX_CURRENT; // Mock Sensor
			BatteryVolts = MockBattery() * BatteryCellCount;
		} else {
#if defined(HAVE_CURRENT_SENSOR)
			real32 Temp = -((analogRead(BattCurrentAnalogSel) - BatteryCurrentADCZero )) * (3.3f/0.04f) *CurrentScaleTrim;
					//* CURRENT_SENSOR_MAX;
			BatteryCurrent = HardFilter(BatteryCurrent, Temp);
#endif

			BatteryVolts
					= MediumFilter(BatteryVolts, analogRead(BattVoltsAnalogSel) * VOLTS_SCALE * VoltScaleTrim);
		}

		dTmS = NowmS - mS[LastBattery];
		mS[LastBattery] = NowmS;

		BatterySagR = MediumFilter(BatterySagR, StartupVolts / BatteryVolts);

		BatteryChargeUsedmAH += BatteryCurrent * (real32) dTmS * (1.0f
				/ 3600.0f);

		F.LowBatt = BatteryVolts <= BatteryVoltsLimit;
	}
} // CheckBatteries


void InitBattery(void) {

	CurrentScaleTrim = P(CurrentScale) * 0.01f;
	VoltScaleTrim = P(VoltScale) * 0.01f;

	if (F.Emulation) {
		BatteryCellCount = 3.0;
		StartupVolts = BatteryVolts = 12.6f;
	} else {
		StartupVolts = BatteryVolts = analogRead(BattVoltsAnalogSel) * VOLTS_SCALE * VoltScaleTrim;
		BatteryCellCount = (int16)(BatteryVolts / 3.7f); // OK for 3-6 cell LiPo if charged!
	}

	BatterySagR = 1.0f;

	BatteryVoltsLimit = P(LowVoltThres) * 0.1f;
	BatteryCapacitymAH = (P(BatteryCapacity) * 100.0f);
	BatteryVolts = BatteryVoltsLimit;
	BatteryCurrent = BatteryChargeUsedmAH = 0.0f;

	BatteryCurrentADCZero = analogRead(BattCurrentAnalogSel);

} // InitBattery


