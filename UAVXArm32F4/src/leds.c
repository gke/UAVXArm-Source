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

boolean BeeperIsOn;
boolean LEDsSaved[MAX_LED_PINS];

void BeeperOn(void) {
	if (GPIOPins[BeeperSel].Used)
		DigitalWrite(&GPIOPins[BeeperSel].P, beeperLowOn);
	BeeperIsOn = true;
} // BeeperOn

void BeeperOff(void) {
	if (GPIOPins[BeeperSel].Used)
		DigitalWrite(&GPIOPins[BeeperSel].P, !beeperLowOn); //
	BeeperIsOn = false;
} // BeeperOff

void BeeperToggle(void) {
	DigitalToggle(&GPIOPins[BeeperSel].P);
	BeeperIsOn = !BeeperIsOn;
} // BeeperToggle

void LEDOn(uint8 l) {

	if (LEDPins[l].Used)
		DigitalWrite(&LEDPins[l].P, ledsLowOn);

} // LEDOn

void LEDOff(uint8 l) {

	if (LEDPins[l].Used)
		DigitalWrite(&LEDPins[l].P, !ledsLowOn);

} // LEDOff

void LEDToggle(uint8 l) {

	if (LEDPins[l].Used)
		DigitalToggle(&LEDPins[l].P);
} // LEDToggle

void LEDsOn(void) {
	idx l;

	for (l = 0; l < MAX_LED_PINS; l++)
		LEDOn(l);
} // LEDsOn

void LEDsOff(void) {
	idx l;

	for (l = 0; l < MAX_LED_PINS; l++)
		LEDOff(l);
} // LEDsOff

void LEDsOffExcept(uint8 l) {
	idx lll;

	for (ll = 0; ll < MAX_LED_PINS; ll++)
		if (l == ll)
			LEDOn(l);
		else
			LEDOff(ll);

} // LEDsOffExcept

void LEDChaser(void) {
	static uint16 lastLED = 0;
	static boolean blink = false;

	if (mSTimeout(chaserTimeoutmS)) {
		mSTimer(chaserTimeoutmS, 1000);

		LEDOff(lastLED);
		lastLED++;
		if (lastLED >= MAX_LED_PINS)
			lastLED = 0;
		LEDOn(lastLED);
	}

} // LEDChaser

void LEDRandom(void) {
	static uint16 lastLED = 0;
	static boolean blink = false;
	idx l;

	if (mSTimeout(chaserTimeoutmS)) {
		mSTimer(chaserTimeoutmS, 100);

		for (l = 0; l < MAX_LED_PINS; l++) {
			if (SensorNoise(1.0f) > 0.0f)
				LEDOn(l);
			else
				LEDOff(l);
		}
	}

} // LEDRandom

void InitLEDs(void) {
	idx l;

	for (l = 0; l < MAX_LED_PINS; l++)
		if (LEDPins[l].Used)
			DigitalWrite(&LEDPins[l].P, !ledsLowOn);

	LEDsOff();
	BeeperOff();

} // InitLEDs

