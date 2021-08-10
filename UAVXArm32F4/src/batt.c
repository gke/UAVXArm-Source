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

#if defined(VOLT_MEASUREMENT_ONBOARD)
#define VOLTS_SCALE	((3.3f*(10.0f+2.2f))/2.2f) = 18.3
#else
#define VOLTS_SCALE	(3.3f)
#endif

// APM Power Module:
//	shunt resistor and the unipolar INA169
//	BEC 5.3@3A
// 	internal ref 5.3V 60mV/A or 90A so with UAVX 51A FS
//	CAUTION: Banggood V1.0 Vision voltage divider is x0.5! => SMOKE SMOKE SMOKE SMOKE

// Allegro e.g. ACS758xCB:
// 	Hall effect bipolar
//	no BEC
//	external ref 3-5.5V 40mV/A or 82.5A FS (+/-41.25A)

real32 VoltageScale, CurrentScale;
real32 CurrentSensorSwing;

real32 BatteryVolts, StartupVolts, BatteryCurrent, BatteryVoltsLimit,
		BatteryChargeUsedmAH, BatteryCapacityLimitmAH;

real32 BatteryCurrentADCZero = 0.0f; // takes a while for bipolar capture of offset

real32 BatteryCapacitymAH;
uint8 BatteryCellCount = 3;

real32 MockBattery(void) {
	real32 CellVolts;

	if (BatteryChargeUsedmAH < (BatteryCapacitymAH * 0.05f))
		CellVolts = 4.2
				- 0.2f * BatteryChargeUsedmAH / (BatteryCapacitymAH * 0.05f);
	else if (BatteryChargeUsedmAH < (BatteryCapacitymAH * 95.0f))
		CellVolts = 4.0f
				- 1.0f * (BatteryChargeUsedmAH - BatteryCapacitymAH * 0.05f)
						/ (BatteryCapacitymAH * 0.90f);
	else
		CellVolts = 3.0f
				- 3.0f * (BatteryChargeUsedmAH - BatteryCapacitymAH * 0.95f)
						/ (BatteryCapacitymAH * 0.05f);

	return (CellVolts);

} // MockBattery

void CalcBatThrFFComp(real32 RawBatteryVolts, real32 BattdT) {
	static real32 BatteryBoost = 1.0f;

	if (UsingBatteryComp && (P(LowVoltThres) > 0) && (State == InFlight))
		BatteryBoost = SlewLimit(BatteryBoost, StartupVolts / RawBatteryVolts,
				FromPercent(5.0f), BattdT);
	else
		BatteryBoost = 1.0f;

	BattThrFFComp = Limit(BatteryBoost, 1.0f, 1.2f);

} // CalcBattThrFFComp

void CheckBatteries(void) {
	static timeuS LastUpdateuS = 0;
	real32 RawBatteryVolts, RawBatteryCurrent;
	timeuS NowuS;
	real32 BattdT;

	NowuS = uSClock();

	if (mSTimeout(BatteryUpdatemS)) {
		mSTimer(BatteryUpdatemS, BATTERY_UPDATE_MS);

		BattdT = ((LastUpdateuS > 0) ? NowuS - LastUpdateuS : 0.0f) * 0.000001f;
		LastUpdateuS = NowuS;

		if (F.Emulation) {
			RawBatteryCurrent = (DesiredThrottle + AltHoldThrComp)
					* CurrentScale * 0.5f; // reduce emulated current to a sensible fraction of FS
			if (RawBatteryCurrent < 0.0f)
				RawBatteryCurrent = 0.0f;
			RawBatteryVolts = MockBattery() * BatteryCellCount;
		} else {
			RawBatteryCurrent = (analogRead(BattCurrentAnalogSel)
					- BatteryCurrentADCZero) * CurrentScale;
			RawBatteryVolts = analogRead(BattVoltsAnalogSel) * VoltageScale;
		}

		CalcBatThrFFComp(RawBatteryVolts, BattdT);

		BatteryChargeUsedmAH += RawBatteryCurrent * BattdT * (1.0f / 3.6f);

		BatteryCurrent = LPF1(BatteryCurrent, RawBatteryCurrent,
		BATTERY_LPF_HZ);

		BatteryVolts = SlewLimit(BatteryVolts, RawBatteryVolts, 1.0f, BattdT);

		F.LowBatt = ((BatteryVolts <= BatteryVoltsLimit)
				&& (P(LowVoltThres) > 0))
				|| (BatteryChargeUsedmAH > BatteryCapacityLimitmAH);
	}

} // CheckBatteries

void InitBattery(void) {
	int16 i;

	VoltageScale = P(VoltageSensorFS) * 0.2f * FromPercent(P(VoltScaleTrim));
	CurrentScale = P(CurrentSensorFS) * FromPercent(P(CurrentScaleTrim));

	BattThrFFComp = 1.0f;
	BatteryVoltsLimit = Limit(P(LowVoltThres) * 0.1f, 0.1f, 25.0f);
	BatteryCapacitymAH = (P(BatteryCapacity) * 100.0f);
	BatteryCapacityLimitmAH = BatteryCapacitymAH * 0.75f;

	BatteryVolts = BatteryVoltsLimit;
	BatteryCurrent = BatteryChargeUsedmAH = 0.0f;

	if (F.Emulation) {
		BatteryCellCount = 3.0;
		StartupVolts = BatteryVolts = 12.6f;
	} else {

		StartupVolts = analogRead(BattVoltsAnalogSel) * VoltageScale;
		BatteryCurrentADCZero = analogRead(BattCurrentAnalogSel);

		for (i = 0; i < 200; i++) {
			Delay1mS(5);
			StartupVolts = SlewLimit(StartupVolts,
					analogRead(BattVoltsAnalogSel) * VoltageScale, 2.0f,
					0.005f);
			BatteryCurrentADCZero = SlewLimit(BatteryCurrentADCZero,
					analogRead(BattCurrentAnalogSel), 200.0f, 0.005f);
		}

		BatteryVolts = StartupVolts;
		BatteryCellCount = (int16) (BatteryVolts / 3.7f); // OK for 3-6 cell LiPo if charged!

		// TODO: StartupVolts = BatteryCellCount * 4.2f;
	}

} // InitBattery

