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

// LEDYellowSel, LEDRedSel, LEDBlueSel, LEDGreenSel, LEDNone


uint8 CurrwsNoOfLeds = 0;

#if (defined(USE_WS2812) || defined(USE_WS2812B)) && !defined(OMNIBUSF4V1_BOARD)

// Y/O, R, B, G
const wsLEDStruct wsLEDColours[] = { { 0xff, 0x45, 0 }, { 0xff, 0, 0 }, { 0, 0,
		0xff }, { 0, 0x80, 0 }, { 0, 0, 0 } }; // RGB

uint16 wsPWMBuffer[MAX_PWM_BUFFER_SIZE];
wsLEDStruct wsLEDs[MAX_WS2812_LEDS];
uint8 wsNoOfLeds, wsGroupSize;
uint16 wsBufferSize;
uint32 wsCurrLED = 0;
boolean incomplete = false;

// Recommendation post 2013 from WorldSemi are now: 0 = 400ns high/850ns low, and 1 = 850ns high, 400ns low"
// Currently the timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.
// Betaflight timings are 0 = 350ns high/800ns and 1 = 700ns high/650ns low.

static void wsStartDMA1(uint16 BuffSize) {
	static DMA_InitTypeDef dma_init = { .DMA_BufferSize = MAX_WS2812_LEDS * 24,
			.DMA_Channel = DMA_Channel_0,
			.DMA_DIR = DMA_DIR_MemoryToPeripheral, .DMA_FIFOMode =
					DMA_FIFOMode_Disable, .DMA_FIFOThreshold =
					DMA_FIFOThreshold_HalfFull, .DMA_Memory0BaseAddr =
					(uint32) wsPWMBuffer, .DMA_MemoryBurst =
					DMA_MemoryBurst_Single, .DMA_MemoryDataSize =
					DMA_MemoryDataSize_HalfWord, .DMA_MemoryInc =
					DMA_MemoryInc_Enable, .DMA_Mode = DMA_Mode_Circular,
			.DMA_PeripheralBaseAddr = (uint32) &TIM8->CCR1,
			.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
			.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord,
			.DMA_PeripheralInc = DMA_PeripheralInc_Disable, .DMA_Priority =
					DMA_Priority_High }; // Medium

	dma_init.DMA_BufferSize = BuffSize;

	DMA_Init(DMA2_Stream2, &dma_init);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	TIM_DMACmd(TIM8, TIM_DMA_CC1, ENABLE);

} // wsStartDMA1


static void wsInitBuffers(void) {
	int16 i;

	for (i = 0; i < wsBufferSize; i++)
		wsPWMBuffer[i] = 0;

	for (i = 0; i < wsNoOfLeds; i++) {
		wsLEDs[i].r = 0;
		wsLEDs[i].g = 0;
		wsLEDs[i].b = 0;
	}
} // wsInitBuffers


void wsInit(void) { // hard coded to PORTC Pin 6 Aux1
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
	TIM_OCInitTypeDef TIM_OC_InitStructure;
	NVIC_InitTypeDef nvic_init;

	if (CurrwsNoOfLeds > 0) {

		wsNoOfLeds = Limit(CurrwsNoOfLeds, 4, MAX_WS2812_LEDS);
		wsGroupSize = wsNoOfLeds / MAX_LEDS;
		wsBufferSize = wsNoOfLeds * 24;

		wsInitBuffers();

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);

		TIM_TimeBase_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBase_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBase_InitStructure.TIM_Period = 41; //210;
		TIM_TimeBase_InitStructure.TIM_Prescaler = 4; // 0 TODO:
		TIM_TimeBaseInit(TIM8, &TIM_TimeBase_InitStructure);

		TIM_OCStructInit(&TIM_OC_InitStructure);
		TIM_OC_InitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OC_InitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
		TIM_OC_InitStructure.TIM_OutputState = TIM_OutputState_Enable;

		TIM_OC1Init(TIM8, &TIM_OC_InitStructure);

		TIM_CtrlPWMOutputs(TIM8, ENABLE);

		TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM8, ENABLE);

		TIM_CCxCmd(TIM8, TIM_Channel_1, TIM_CCx_Enable);
		TIM_Cmd(TIM8, ENABLE);

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		TIM_DMACmd(TIM8, TIM_DMA_CC1, ENABLE);
		DMA_ITConfig(DMA2_Stream2, DMA_IT_HT, ENABLE);
		DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);

		wsStartDMA1(wsBufferSize);

		nvic_init.NVIC_IRQChannel = DMA2_Stream2_IRQn;
		nvic_init.NVIC_IRQChannelPreemptionPriority = 4;
		nvic_init.NVIC_IRQChannelSubPriority = 0;
		nvic_init.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic_init);
	}

} // wsInit

#define ss 33 // 168.0f
static void wsGenPWM(uint16 ** const dest, const uint8 color) {
	// generates the PWM patterns for each colour byte

#if defined(USE_WS2812)
	const uint16 L = (uint16) (0.35f * ss); // 11.55 473
	const uint16 H = (uint16) (0.7f * ss); // 23.1 947

#else // USE_WS2812B
	const uint16 L = (uint16)(0.4f*ss); // 13.2 541.2
	const uint16 H = (uint16)(0.8f*ss); // 26.4 1082.4
#endif
	uint8 mask = 0x80;

	do {
		**dest = color & mask ? H : L; // pulse width 0.625, 0.25
		*dest += 1;
		mask >>= 1;
	} while (mask != 0);

} // wsGenPWM


void wsUpdateBuffer(uint16* B) { // computationally expensive ~6%
	wsLEDStruct *wsLEDptr;
	uint32 i, j;
	uint16 * wsPWMptr;

	if (CurrwsNoOfLeds > 0) {

		for (i = 0; i < (wsBufferSize >> 1); i += 24) {
			if (incomplete) {
				incomplete = false;
				for (j = 0; j < 24; j++)
					B[i + j] = 0;

			} else {
				if (wsCurrLED == wsNoOfLeds) {
					incomplete = true;
					wsCurrLED = 0;

					for (j = 0; j < 24; j++)
						B[i + j] = 0;
				} else {
					wsLEDptr = &wsLEDs[wsCurrLED++];
					wsPWMptr = B + i;

					// WS2812 order is G R B
					wsGenPWM(&wsPWMptr, wsLEDptr->g);
					wsGenPWM(&wsPWMptr, wsLEDptr->r);
					wsGenPWM(&wsPWMptr, wsLEDptr->b);
				}
			}
		}
	}
} // wsUpdateBuffer


void wsSetColours(int i, uint8 R, uint8 G, uint8 B) {
	wsLEDs[i].g = G;
	wsLEDs[i].r = R;
	wsLEDs[i].b = B;
} // wsSetColours

void wsLEDColour(int i, const wsLEDStruct w) {

	wsSetColours(i, w.r, w.g, w.b);
} // wsLEDColour

void wsLED(uint8 l) {
	uint8 ll;

	for (ll = 0; ll < MAX_LEDS; ll++)
		wsLEDColour(ll, wsLEDColours[l]);
} // wsLEDColour

void wsLEDOn(uint8 l) { // run in pairs
	uint8 ll;

	if (CurrwsNoOfLeds >= MAX_LEDS)
		for (ll = 0; ll < wsGroupSize; ll++)
			wsLEDColour(l * wsGroupSize + ll, wsLEDColours[l]);

} // wsLEDOn

void wsLEDOff(uint8 l) {
	uint8 ll;

	if (CurrwsNoOfLeds >= MAX_LEDS)
		for (ll = 0; ll < wsGroupSize; ll++)
			wsSetColours(l * wsGroupSize + ll, 0, 0, 0);

} // wsLEDOff;

boolean wsLEDIsOn(uint8 l) {

	if (CurrwsNoOfLeds >= MAX_LEDS)
		return ((wsLEDs[l].r != 0) || (wsLEDs[l].g != 0) || (wsLEDs[l].b != 0));
	else
		return false;

} // wsLEDIsOn

void wsLEDToggle(uint8 l) {

	if (CurrwsNoOfLeds >= MAX_LEDS) {
		if (wsLEDIsOn(l))
			wsLEDOff(l);
		else
			wsLEDOn(l);
	}

} // wsLEDToggle

void UpdatewsLed(void) {
	static uint8 PrevState = 255;
	static boolean PrevOriginValid = false;

	if (CurrwsNoOfLeds == 1) {
		if ((State != PrevState) || (F.OriginValid != PrevOriginValid)) {
			PrevState = State;
			PrevOriginValid = F.OriginValid;

			switch (State) {
			case Preflight:
				wsLED(ledRedSel);
				break;
			case Ready:
				wsLED(ledYellowSel);
				break;
			case Starting:
				wsLED(ledYellowSel);
				break;
			case Warmup:
				wsLED(ledYellowSel);
				break;
			case Landing:
			case Landed:
			case InFlight:
				if (F.OriginValid)
					wsLED(ledBlueSel);
				else
					wsLED(ledGreenSel);
				break;
			case Shutdown:
				wsLED(MAX_LEDS);
				break;
			case IREmulate:
				wsLED(ledGreenSel);
				break;
			default:
				wsLED(MAX_LEDS);
				break;
			}
		}
	}
} // UpdatewsLed

#else

void wsInit(void) {
} // wsInit

void wsLEDOn(uint8 l) {
} // wsLEDOn

void wsLEDOff(uint8 l) {
} // wsLEDOff;

boolean wsLEDIsOn(uint8 l) {

	return (false);
} // wsLEDIsOn

void wsLEDToggle(uint8 l) {
} // wsLEDToggle

void UpdatewsLed(void) {
} // UpdatewsLed

#endif

//______________________________________________________________

uint8 LEDChase[] = { ledBlueSel, ledGreenSel, ledRedSel, ledYellowSel };
boolean LEDsSaved[4] = { false };
uint8 LEDPattern = 0;
boolean UsingExtLEDs = false;

void BeeperOff(void) {
	if (BeeperSel < MAX_GPIO_PINS)
#if defined(V1_BOARD)
		digitalWrite(&GPIOPins[BeeperSel], 1);
#else
		digitalWrite(&GPIOPins[BeeperSel], 0);
#endif
} // BeeperOff

void BeeperOn(void) {
	if (BeeperSel < MAX_GPIO_PINS)
#if defined(V1_BOARD)
		digitalWrite(&GPIOPins[BeeperSel], 0);
#else
		digitalWrite(&GPIOPins[BeeperSel], 1);
#endif
} // BeeperOn

void BeeperToggle(void) {
	if (BeeperSel < MAX_GPIO_PINS)
		digitalToggle(&GPIOPins[BeeperSel]);
} // BeeperToggle

boolean BeeperIsOn(void) {
	if (BeeperSel < MAX_GPIO_PINS)
#if defined(V1_BOARD)
		return (digitalRead(&GPIOPins[BeeperSel]) == 0);
#else
		return (digitalRead(&GPIOPins[BeeperSel]) != 0);
#endif
	else
		return false;
} // BeeperIsOn

void LEDOn(uint8 l) {

#if defined(V1_BOARD)
	if (State == InFlight)
	LEDsOff();
	else
#endif
	if (l < MAX_LEDS) {
		digitalWrite(&LEDPins[l], 0);
		wsLEDOn(l);
	}

} // LEDOn

void LEDOff(uint8 l) {

	if (l < MAX_LEDS) {
		digitalWrite(&LEDPins[l], 1);
		wsLEDOff(l);
	}
} // LEDOff

void LEDToggle(uint8 l) {
	if (l < MAX_LEDS) {
		digitalToggle(&LEDPins[l]);
		wsLEDToggle(l);
	}
} // LEDToggle

boolean LEDIsOn(uint8 l) {
	if (l < MAX_LEDS)
		return (!digitalRead(&LEDPins[l]));
	else
		return (false);
} // LEDIsOn

void LEDsOn(void) {
	uint8 l;

	for (l = 0; l < MAX_LEDS; l++)
		LEDOn(l);
} // LEDsOn

void LEDsOff(void) {
	uint8 l;

	for (l = 0; l < MAX_LEDS; l++)
		LEDOff(l);
} // LEDsOff

void SaveLEDs(void) { // one level only
	uint8 l;

	for (l = 0; l < MAX_LEDS; l++)
		LEDsSaved[l] = LEDIsOn(l);
} // SaveLEDs

void RestoreLEDs(void) {
	uint8 l;

	for (l = 0; l < MAX_LEDS; l++)
		LEDOn(LEDsSaved[l]);
} // RestoreLEDs

void LEDChaser(void) {
	uint32 NowmS;

	NowmS = mSClock();
	if (NowmS > mS[LEDChaserUpdate]) {
		if (F.AltControlEnabled && F.HoldingAlt) {
			LEDOff(LEDChase[LEDPattern]);
			if (LEDPattern < MAX_LEDS)
				LEDPattern++;
			else
				LEDPattern = 0;
			LEDOn(LEDChase[LEDPattern]);
		} else
			LEDsOff();

		mSTimer(NowmS, LEDChaserUpdate, 100);
	}

} // LEDChaser


void PowerOutput(uint8 d) {
	uint8 s;

	LEDOff(d);
	for (s = 0; s < 10; s++) { // 10 flashes (count MUST be even!)
		LEDToggle(d);
		Delay1mS(50);
	}
} // PowerOutput


void InitLEDs(void) {

	LEDsOff();
	BeeperOff();

} // InitLEDs


