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

// 	  WS281X  routines based on LEDStrip RGB-LED Driver
//    Tobias Mache & Florian Zahn CCC Mannheim e.V.
//    https://github.com/C3MA

#include "UAVX.h"

//_________________________________________________________________

// WS2812 (hard coded to Aux1)

enum Colours { // RGB order
	AliceBlue = 0xF0F8FF,
	Amethyst = 0x9966CC,
	AntiqueWhite = 0xFAEBD7,
	Aqua = 0x00FFFF,
	Aquamarine = 0x7FFFD4,
	Azure = 0xF0FFFF,
	Beige = 0xF5F5DC,
	Bisque = 0xFFE4C4,
	Black = 0x000000,
	BlanchedAlmond = 0xFFEBCD,
	Blue = 0x0000FF,
	BlueViolet = 0x8A2BE2,
	Brown = 0xA52A2A,
	BurlyWood = 0xDEB887,
	CadetBlue = 0x5F9EA0,
	Chartreuse = 0x7FFF00,
	Chocolate = 0xD2691E,
	Coral = 0xFF7F50,
	CornflowerBlue = 0x6495ED,
	Cornsilk = 0xFFF8DC,
	Crimson = 0xDC143C,
	Cyan = 0x00FFFF,
	DarkBlue = 0x00008B,
	DarkCyan = 0x008B8B,
	DarkGoldenrod = 0xB8860B,
	DarkGray = 0xA9A9A9,
	DarkGreen = 0x006400,
	DarkKhaki = 0xBDB76B,
	DarkMagenta = 0x8B008B,
	DarkOliveGreen = 0x556B2F,
	DarkOrange = 0xFF8C00,
	DarkOrchid = 0x9932CC,
	DarkRed = 0x8B0000,
	DarkSalmon = 0xE9967A,
	DarkSeaGreen = 0x8FBC8F,
	DarkSlateBlue = 0x483D8B,
	DarkSlateGray = 0x2F4F4F,
	DarkTurquoise = 0x00CED1,
	DarkViolet = 0x9400D3,
	DeepPink = 0xFF1493,
	DeepSkyBlue = 0x00BFFF,
	DimGray = 0x696969,
	DodgerBlue = 0x1E90FF,
	FireBrick = 0xB22222,
	FloralWhite = 0xFFFAF0,
	ForestGreen = 0x228B22,
	Fuchsia = 0xFF00FF,
	Gainsboro = 0xDCDCDC,
	GhostWhite = 0xF8F8FF,
	Gold = 0xFFD700,
	Goldenrod = 0xDAA520,
	Gray = 0x808080,
	Green = 0x008000,
	GreenYellow = 0xADFF2F,
	Honeydew = 0xF0FFF0,
	HotPink = 0xFF69B4,
	IndianRed = 0xCD5C5C,
	Indigo = 0x4B0082,
	Ivory = 0xFFFFF0,
	Khaki = 0xF0E68C,
	Lavender = 0xE6E6FA,
	LavenderBlush = 0xFFF0F5,
	LawnGreen = 0x7CFC00,
	LemonChiffon = 0xFFFACD,
	LightBlue = 0xADD8E6,
	LightCoral = 0xF08080,
	LightCyan = 0xE0FFFF,
	LightGoldenrodYellow = 0xFAFAD2,
	LightGreen = 0x90EE90,
	LightGrey = 0xD3D3D3,
	LightPink = 0xFFB6C1,
	LightSalmon = 0xFFA07A,
	LightSeaGreen = 0x20B2AA,
	LightSkyBlue = 0x87CEFA,
	LightSlateGray = 0x778899,
	LightSteelBlue = 0xB0C4DE,
	LightYellow = 0xFFFFE0,
	Lime = 0x00FF00,
	LimeGreen = 0x32CD32,
	Linen = 0xFAF0E6,
	Magenta = 0xFF00FF,
	Maroon = 0x800000,
	MediumAquamarine = 0x66CDAA,
	MediumBlue = 0x0000CD,
	MediumOrchid = 0xBA55D3,
	MediumPurple = 0x9370DB,
	MediumSeaGreen = 0x3CB371,
	MediumSlateBlue = 0x7B68EE,
	MediumSpringGreen = 0x00FA9A,
	MediumTurquoise = 0x48D1CC,
	MediumVioletRed = 0xC71585,
	MidnightBlue = 0x191970,
	MintCream = 0xF5FFFA,
	MistyRose = 0xFFE4E1,
	Moccasin = 0xFFE4B5,
	NavajoWhite = 0xFFDEAD,
	Navy = 0x000080,
	OldLace = 0xFDF5E6,
	Olive = 0x808000,
	OliveDrab = 0x6B8E23,
	Orange = 0xFFA500,
	OrangeRed = 0xFF4500,
	Orchid = 0xDA70D6,
	PaleGoldenrod = 0xEEE8AA,
	PaleGreen = 0x98FB98,
	PaleTurquoise = 0xAFEEEE,
	PaleVioletRed = 0xDB7093,
	PapayaWhip = 0xFFEFD5,
	PeachPuff = 0xFFDAB9,
	Peru = 0xCD853F,
	Pink = 0xFFC0CB,
	Plaid = 0xCC5533,
	Plum = 0xDDA0DD,
	PowderBlue = 0xB0E0E6,
	Purple = 0x800080,
	Red = 0xFF0000,
	RosyBrown = 0xBC8F8F,
	RoyalBlue = 0x4169E1,
	SaddleBrown = 0x8B4513,
	Salmon = 0xFA8072,
	SandyBrown = 0xF4A460,
	SeaGreen = 0x2E8B57,
	Seashell = 0xFFF5EE,
	Sienna = 0xA0522D,
	Silver = 0xC0C0C0,
	SkyBlue = 0x87CEEB,
	SlateBlue = 0x6A5ACD,
	SlateGray = 0x708090,
	Snow = 0xFFFAFA,
	SpringGreen = 0x00FF7F,
	SteelBlue = 0x4682B4,
	Tan = 0xD2B48C,
	Teal = 0x008080,
	Thistle = 0xD8BFD8,
	Tomato = 0xFF6347,
	Turquoise = 0x40E0D0,
	Violet = 0xEE82EE,
	Wheat = 0xF5DEB3,
	White = 0xFFFFFF,
	WhiteSmoke = 0xF5F5F5,
	Yellow = 0xFFFF00,
	YellowGreen = 0x9ACD32
};

// ledYellowSel, ledRedSel, ledDBlueSel, ledGreenSel

boolean LEDState[MAX_LED_PINS];
boolean LEDsSaved[MAX_LED_PINS] = { false, };
boolean UsingWS28XXLEDs = false;
uint8 CurrNoOfWSLEDs = 0;

uint8 CurrBeeperSel = BeeperSel; // Aux2Sel; //

#if (defined(USE_WS2812) || defined(USE_WS2812B))

// Y/O, R, B, G
const WSLEDStruct WSLEDColours[] = { { 0xff, 0xa5, 0 }, { 0xff, 0, 0 }, { 0, 0,
		0xff }, { 0, 0x80, 0 } }; // RGB

const WSLEDStruct LEDNone = { 0, 0, 0 };

uint16 WSLEDPWMBuffer[MAX_PWM_BUFFER_SIZE];
WSLEDStruct WSLEDs[MAX_WS2812_LEDS];
uint8 NoOfWSLEDs, WSLEDGroupSize;
uint16 WSLEDBufferSize;
uint8 CurrWSLED = 0;
boolean LEDStateChanged;

// Recommendation post 2013 from WorldSemi are now: 0 = 400ns high/850ns low, and 1 = 850ns high, 400ns low"
// Currently the timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.
// Betaflight timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.

/*
 * "Note that the timing on the WS2812/WS2812B LEDs has changed as of batches from WorldSemi
 * manufactured made in October 2013, and timing tolerance for approx 10-30% of parts is very small.
 * Recommendation from WorldSemi is now: 0 = 400ns high/850ns low, and 1 = 850ns high, 400ns low"
 *
 * Currently the timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.
 *
 *
 *
 * 375/875 750/500 TODO:
 *
 *
 *
 */

//Compute the prescaler value
// uint16 period = WS2811_TIMER_HZ / WS2811_CARRIER_HZ;


const uint16 WS_H = 27; // 20; //period * 2 / 3;
const uint16 WS_L = 14; //10; //period / 3;


static inline void GenWSLEDPWM(uint16 ** const dest, const uint8 color) {
	// generates the PWM patterns for each colour byte - lookup table would be too large

	uint8 mask = 0x80;

	do {
		**dest = color & mask ? WS_H : WS_L;
		*dest += 1;
		mask >>= 1;
	} while (mask != 0);

} // GenWSLEDPWM

void UpdateWSLEDBuffer(void) {
	WSLEDStruct *WSLEDptr;
	uint16 *WSPWMptr;
	idx CurrWSLED;
	uint16 i;

	if (UsingWS28XXLEDs) {
		if (WSDMAInactive) {
			if (LEDStateChanged) {
				CurrWSLED = 0;
				for (i = 0; i < WSLEDBufferSize; i += WS2812_COLOUR_FRAME_LEN) {
					WSLEDptr = &WSLEDs[CurrWSLED];
					WSPWMptr = &WSLEDPWMBuffer[0] + WS2812_RESET_LEN + i;

					GenWSLEDPWM(&WSPWMptr, WSLEDptr->c.g);
					GenWSLEDPWM(&WSPWMptr, WSLEDptr->c.r);
					GenWSLEDPWM(&WSPWMptr, WSLEDptr->c.b);
					CurrWSLED++;
				}
				LEDStateChanged = false;
			}
			WSPinDMAEnable(WSLEDBufferSize);
			WSDMAInactive = false;
		}
	}
} // UpdateWSLEDBuffer


void SetWSLEDColours(idx i, uint8 R, uint8 G, uint8 B) {
	WSLEDs[i].c.g = G;
	WSLEDs[i].c.r = R;
	WSLEDs[i].c.b = B;
} // wsSetColours


void InitWSLEDs(void) {
	idx l;

	if (UsingWS28XXLEDs) {

		NoOfWSLEDs = Limit(CurrNoOfWSLEDs, MAX_LED_PINS, MAX_WS2812_LEDS);
		WSLEDGroupSize = NoOfWSLEDs / MAX_LED_PINS;
		WSLEDBufferSize = NoOfWSLEDs * WS2812_COLOUR_FRAME_LEN;

		memset(&WSLEDPWMBuffer, 0, sizeof(WSLEDPWMBuffer));
		for (l = 0; l < NoOfWSLEDs; l++)
			SetWSLEDColours(l, 0, 0, 0);

		InitWSPin(WSLEDBufferSize);
		LEDStateChanged = WSDMAInactive = true;
	}

} // InitWSLEDs


void WSLEDColour(idx i, const WSLEDStruct w) {
	SetWSLEDColours(i, w.c.r, w.c.g, w.c.b);
} // WSLEDColour

void WSLEDOn(idx l) { // run in pairs
	idx ll;

	if (!LEDState[l]) {
		for (ll = 0; ll < WSLEDGroupSize; ll++)
			WSLEDColour(l * WSLEDGroupSize + ll, WSLEDColours[l]);

		LEDStateChanged |= true;
		LEDState[l] = true;
	}

} // WSLEDOn

void WSLEDOff(idx l) {
	idx ll;

	if (LEDState[l]) {
		for (ll = 0; ll < WSLEDGroupSize; ll++)
			SetWSLEDColours(l * WSLEDGroupSize + ll, 0, 0, 0);
		LEDStateChanged |= true;
		LEDState[l] = false;
	}

} // WSLEDOff;

void WSLEDToggle(idx l) {

	if (LEDState[l])
		WSLEDOff(l);
	else
		WSLEDOn(l);
} // WSLEDToggle


#else

void UpdateWSLEDBuffer(void) {}

void WSLEDOn(idx l) {
}

void WSLEDOff(idx l) {
}

void WSLEDToggle(idx l) {
}

void InitWSLEDs(void) {}

#endif

//______________________________________________________________


void BeeperOff(void) {
	if (CurrBeeperSel == Aux2Sel)
		DigitalWrite(&GPIOPins[CurrBeeperSel].P, true);
	else
		DigitalWrite(&GPIOPins[CurrBeeperSel].P, false); //
} // BeeperOff

void BeeperOn(void) {
	if (CurrBeeperSel == Aux2Sel)
		DigitalWrite(&GPIOPins[CurrBeeperSel].P, false);
	else
		DigitalWrite(&GPIOPins[CurrBeeperSel].P, true);
} // BeeperOn

void BeeperToggle(void) {
	DigitalToggle(&GPIOPins[CurrBeeperSel].P);
} // BeeperToggle

boolean BeeperIsOn(void) {
	return (DigitalRead(&GPIOPins[CurrBeeperSel].P));
} // BeeperIsOn

void LEDOn(uint8 l) {

	if (UsingWS28XXLEDs)
		WSLEDOn(l);
	else
		DigitalWrite(&LEDPins[l].P, ledsLowOn);

} // LEDOn

void LEDOff(uint8 l) {
	if (UsingWS28XXLEDs)
		WSLEDOff(l);
	else
		DigitalWrite(&LEDPins[l].P, !ledsLowOn);

} // LEDOff

void LEDToggle(uint8 l) {
	if (UsingWS28XXLEDs)
		WSLEDToggle(l);
	else
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
			LEDOff(l);

} // LEDsOffExcept

void LEDChaser(void) {
	static uint16 lastLED = 0;
	static boolean blink = false;

	if (UsingWS28XXLEDs)
		if (mSTimeout(chaserTimeoutmS)) {
			mSTimer(chaserTimeoutmS, 100);

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

		if (UsingWS28XXLEDs) {
			for (l = 0; l < NoOfWSLEDs; l++)
				WSLEDs[l].cc = SensorNoise((real32) 0xffffff);
		} else
			for (l = 0; l < MAX_LED_PINS; l++) {
				if (SensorNoise(1.0f) > 0.0f)
					LEDOn(l);
				else
					LEDOff(l);
			}
	}

} // LEDRandom

void SaveLEDs(void) { // one level only
	idx l;

	for (l = 0; l < MAX_LED_PINS; l++)
		LEDsSaved[l] = LEDState[l];
} // SaveLEDs

void RestoreLEDs(void) {
	idx l;

	LEDsOff();
	for (l = 0; l < MAX_LED_PINS; l++)
		if (LEDsSaved[l])
			LEDOn(l);
		else
			LEDOff(l);
} // RestoreLEDs


void InitLEDs(void) {
	idx l;

	for (l = 0; l < MAX_LED_PINS; l++) {
		DigitalWrite(&LEDPins[l].P, !ledsLowOn);
		LEDsSaved[l] = LEDState[l] = false;
	}

	LEDsOff();
	BeeperOff();

} // InitLEDs
