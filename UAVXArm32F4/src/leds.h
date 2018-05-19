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


#ifndef _leds_h
#define _leds_h


void SaveLEDs(void);
void RestoreLEDs(void);
void LEDsOn(void);
void LEDsOff(void);
void LEDToggle(uint8 l);
void LEDOn(uint8 l);
void LEDOff(uint8 l);
void LEDChaser(void);
boolean LEDIsOn(uint8 l);

void PowerOutput(uint8);
void LEDsAndBuzzer(uint8 s);
void InitLEDs(void);

void BeeperOff(void);
void BeeperOn(void);
void BeeperToggle(void);
boolean BeeperIsOn(void);

extern boolean UsingExtLEDs;

#define MAX_WS2812_LEDS 12 // 240
// circular PWM waveform buffer
#define MAX_PWM_BUFFER_SIZE (MAX_WS2812_LEDS*24) // PWM waveform samples
typedef struct {
	uint8 r :8;
	uint8 g :8;
	uint8 b :8;
} wsLEDStruct;

extern void wsInit(void);
void wsUpdateBuffer(uint16_t* buffer);
void wsSetColours(int i, uint8 R, uint8 G, uint8 B);
void wsLEDColour(int i, const wsLEDStruct w);
void wsLED(uint8 l);
void UpdatewsLed(void);

extern const wsLEDStruct wsLEDColours[];

extern uint16 wsPWMBuffer[];
extern uint16 wsBufferSize;
extern uint8 CurrwsNoOfLeds, wsNoOfLeds, wsGroupSize;



#endif

