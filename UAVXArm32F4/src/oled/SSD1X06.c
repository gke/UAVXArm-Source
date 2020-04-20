// Code to drive 128x64 monochrome oled display module
// ceptimus.  September 2016
// Edited by Benik3 January 2019
// Complete rework for UAVX Prof Greg Egan June 2019

#include "UAVX.h"

#if defined(USE_OLED)

uint8 oledPixels[8][128];

const uint8 ssdfont[] = { 0x00, 0x00, 0x00, 0x00, 0x00, // space
		0x00, 0x00, 0x5F, 0x00, 0x00, // !
		0x00, 0x07, 0x00, 0x07, 0x00, // "
		0x14, 0x7F, 0x14, 0x7F, 0x14, // #
		0x24, 0x2A, 0x7F, 0x2A, 0x12, // $
		0x23, 0x13, 0x08, 0x64, 0x62, // %
		0x36, 0x49, 0x56, 0x20, 0x50, // &
		0x00, 0x08, 0x07, 0x03, 0x00, // '
		0x00, 0x1C, 0x22, 0x41, 0x00, // (
		0x00, 0x41, 0x22, 0x1C, 0x00, // )
		0x2A, 0x1C, 0x7F, 0x1C, 0x2A, // *
		0x08, 0x08, 0x3E, 0x08, 0x08, // +
		0x00, 0x80, 0x70, 0x30, 0x00, // ,
		0x08, 0x08, 0x08, 0x08, 0x08, // -
		0x00, 0x00, 0x60, 0x60, 0x00, // .
		0x20, 0x10, 0x08, 0x04, 0x02, // /
		0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
		0x00, 0x42, 0x7F, 0x40, 0x00, // 1
		0x72, 0x49, 0x49, 0x49, 0x46, // 2
		0x21, 0x41, 0x49, 0x4D, 0x33, // 3
		0x18, 0x14, 0x12, 0x7F, 0x10, // 4
		0x27, 0x45, 0x45, 0x45, 0x39, // 5
		0x3C, 0x4A, 0x49, 0x49, 0x31, // 6
		0x41, 0x21, 0x11, 0x09, 0x07, // 7
		0x36, 0x49, 0x49, 0x49, 0x36, // 8
		0x46, 0x49, 0x49, 0x29, 0x1E, // 9
		0x00, 0x00, 0x14, 0x00, 0x00, // :
		0x00, 0x40, 0x34, 0x00, 0x00, // ;
		0x00, 0x08, 0x14, 0x22, 0x41, // <
		0x14, 0x14, 0x14, 0x14, 0x14, // =
		0x00, 0x41, 0x22, 0x14, 0x08, // >
		0x02, 0x01, 0x59, 0x09, 0x06, // ?
		0x3E, 0x41, 0x5D, 0x59, 0x4E, // @
		0x7C, 0x12, 0x11, 0x12, 0x7C, // A
		0x7F, 0x49, 0x49, 0x49, 0x36, // B
		0x3E, 0x41, 0x41, 0x41, 0x22, // C
		0x7F, 0x41, 0x41, 0x41, 0x3E, // D
		0x7F, 0x49, 0x49, 0x49, 0x41, // E
		0x7F, 0x09, 0x09, 0x09, 0x01, // F
		0x3E, 0x41, 0x41, 0x51, 0x73, // G
		0x7F, 0x08, 0x08, 0x08, 0x7F, // H
		0x00, 0x41, 0x7F, 0x41, 0x00, // I
		0x20, 0x40, 0x41, 0x3F, 0x01, // J
		0x7F, 0x08, 0x14, 0x22, 0x41, // K
		0x7F, 0x40, 0x40, 0x40, 0x40, // L
		0x7F, 0x02, 0x1C, 0x02, 0x7F, // M
		0x7F, 0x04, 0x08, 0x10, 0x7F, // N
		0x3E, 0x41, 0x41, 0x41, 0x3E, // O
		0x7F, 0x09, 0x09, 0x09, 0x06, // P
		0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
		0x7F, 0x09, 0x19, 0x29, 0x46, // R
		0x26, 0x49, 0x49, 0x49, 0x32, // S
		0x03, 0x01, 0x7F, 0x01, 0x03, // T
		0x3F, 0x40, 0x40, 0x40, 0x3F, // U
		0x1F, 0x20, 0x40, 0x20, 0x1F, // V
		0x3F, 0x40, 0x38, 0x40, 0x3F, // W
		0x63, 0x14, 0x08, 0x14, 0x63, // X
		0x03, 0x04, 0x78, 0x04, 0x03, // Y
		0x61, 0x59, 0x49, 0x4D, 0x43, // Z
		0x00, 0x7F, 0x41, 0x41, 0x41, // [
		0x02, 0x04, 0x08, 0x10, 0x20, // backslash
		0x00, 0x41, 0x41, 0x41, 0x7F, // ]
		0x04, 0x02, 0x01, 0x02, 0x04, // ^
		0x40, 0x40, 0x40, 0x40, 0x40, // _
		0x00, 0x03, 0x07, 0x08, 0x00, // `
		0x20, 0x54, 0x54, 0x78, 0x40, // a
		0x7F, 0x28, 0x44, 0x44, 0x38, // b
		0x38, 0x44, 0x44, 0x44, 0x28, // c
		0x38, 0x44, 0x44, 0x28, 0x7F, // d
		0x38, 0x54, 0x54, 0x54, 0x18, // e
		0x00, 0x08, 0x7E, 0x09, 0x02, // f
		0x18, 0xA4, 0xA4, 0x9C, 0x78, // g - note first use of 8th (bottom) line
		0x7F, 0x08, 0x04, 0x04, 0x78, // h
		0x00, 0x44, 0x7D, 0x40, 0x00, // i
		0x20, 0x40, 0x40, 0x3D, 0x00, // j
		0x7F, 0x10, 0x28, 0x44, 0x00, // k
		0x00, 0x41, 0x7F, 0x40, 0x00, // l
		0x7C, 0x04, 0x78, 0x04, 0x78, // m
		0x7C, 0x08, 0x04, 0x04, 0x78, // n
		0x38, 0x44, 0x44, 0x44, 0x38, // o
		0xFC, 0x18, 0x24, 0x24, 0x18, // p - uses 8th line
		0x18, 0x24, 0x24, 0x18, 0xFC, // q - uses 8th line
		0x7C, 0x08, 0x04, 0x04, 0x08, // r
		0x48, 0x54, 0x54, 0x54, 0x24, // s
		0x04, 0x04, 0x3F, 0x44, 0x24, // t
		0x3C, 0x40, 0x40, 0x20, 0x7C, // u
		0x1C, 0x20, 0x40, 0x20, 0x1C, // v
		0x3C, 0x40, 0x30, 0x40, 0x3C, // w
		0x44, 0x28, 0x10, 0x28, 0x44, // x
		0x4C, 0x90, 0x90, 0x90, 0x7C, // y - uses 8th line
		0x44, 0x64, 0x54, 0x4C, 0x44, // z
		0x00, 0x08, 0x36, 0x41, 0x00, // {
		0x00, 0x00, 0x77, 0x00, 0x00, // |
		0x00, 0x41, 0x36, 0x08, 0x00, // }
		0x02, 0x01, 0x02, 0x04, 0x02, // ~
		0x3C, 0x26, 0x23, 0x26, 0x3C // house shape: up pointer better than ^
		}; // oledPixels


void cmd(uint8 c) {
	SIOWrite(oledSel, 0, c);
} // cmd


void oledStart(void) {

	// initial setup commands for the display
#if defined(SSD1306)
	cmd(SSD1X06_DISPLAYOFF); // 0xAE
	cmd(SSD1X06_SETMULTIPLEX);
	cmd(SSD1X06_LCDHEIGHT - 1); //COMs (Px - 1)
	cmd(SSD1X06_SETDISPLAYOFFSET);
	cmd(0x00);
	cmd(SSD1X06_SETSTARTLINE | 0x00);
	cmd(SSD1X06_SEGREMAP | 0x01);
	cmd(SSD1X06_COMSCANDEC);
	cmd(SSD1X06_SETCOMPINS);
	cmd(0x12); //selection of connection of SSD1306 chip to OLED. Available values: 0x02, 0x12, 0x22, 0x32. Check this if you have each second line empty.
	cmd(SSD1X06_SETCONTRAST);
	cmd(0xFF);
	cmd(SSD1X06_DISPLAYALLON_RESUME);
	cmd(SSD1X06_NORMALDISPLAY);
	cmd(SSD1X06_SETDISPLAYCLOCKDIV);
	cmd(0xF0); // 0xF0 for max frequency
	cmd(SSD1X06_CHARGEPUMP);
	cmd(0x14); // enable charge pump
	cmd(SSD1X06_SETPRECHARGE);
	cmd(0xF1); // 0xF1 for max brightness, 0x11 for max refresh rate
	cmd(SSD1X06_MEMORYMODE);
	cmd(0x01); // vertical addressing mode
	cmd(SSD1X06_SETVCOMDETECT);
	cmd(0x70); // higher number, higher brightness, max 0x70
	cmd(SSD1X06_DISPLAYON);
#elif defined(SSD1106)
	cmd(0xAE); // display off

	cmd(0x02); // set lower column address
	cmd(0x10); // set higher column address

	cmd(0x40); // set display start line

	cmd(0xB0); // set page address

	cmd(0x81); // contract control
	cmd(0x80); // 128

	cmd(0xA1); // set segment remap

	cmd(0xA6); // normal / reverse

	cmd(0xA8); // multiplex ratio
	cmd(0x3F); // duty = 1/32

	cmd(0xad); // set charge pump enable
	cmd(0x8b); // external VCC

	cmd(0x30); // 0X30---0X33  set VPP   9V liangdu!!!!

	cmd(0xC8); // Com scan direction

	cmd(0xD3); // set display offset
	cmd(0x00); // 0x20

	cmd(0xD5); // set osc division
	cmd(0x80);

	cmd(0xD9); // set pre-charge period
	cmd(0x1f); // 0x22

	cmd(0xDA); // set COM pins
	cmd(0x12);

	cmd(0xdb); // set vcomh
	cmd(0x40);

	cmd(0xAF); // display ON

#endif

} // oledStart


void oledSetColmnPage(uint8 row, uint8 x) {

#if defined(SSD1306)
	cmd(SSD1X06_PAGEADDR);
	cmd(row);
	cmd(row);
	cmd(SSD1X06_COLUMNADDR);
	cmd(x);
	cmd(x < SSD1X06_LCDWIDTH - 6 ? x + 5 : SSD1X06_LCDWIDTH - 1);
#elif defined(SSD1106)
	cmd(0xB0 + line);
	x += 2;
	cmd(x & 0x0F);
	cmd(((x >> 4) & 0x0F) | 0x10);
#endif
} // oledSetColmnPage


boolean oledInWindow(int16 line, int16 col) {
	return (col >= 0) && (col < SSD1X06_LCDWIDTH) && (line >= 0) && (line
			< SSD1X06_LCDHEIGHT);
} // oledInWindow


void oledUpdatePageByte(uint8 page, uint8 col) {
	idx i;

	uint8 B[] = { 0, 0 };

	//	oledSetColmnPage(p, x);

	cmd(SSD1X06_PAGEADDR);
	cmd(page);
	cmd(page);
	cmd(SSD1X06_COLUMNADDR);
	cmd(col);
	cmd(col);

	//	B[0] = 0;
	B[1] = oledPixels[page][col];

	SIOWriteBlock(oledSel, 0x40, 2, &B[0]);

} // oledUpdatePageByte


void oledSetPixel(int16 x, int16 y, boolean on) {
	uint8 old, page;

	if (oledInWindow(x, y)) {
		page = x / 8;

		old = oledPixels[page][y];

		if (on)
			oledPixels[page][y] |= (1 << (x % 8));
		else
			oledPixels[page][y] &= ~(1 << (x % 8));

		if (old != oledPixels[page][y])
			oledUpdatePageByte(page, y);

	}
} // oledSetPixel


void oledDrawCircle(int16 x, int16 y, uint16 r) {
	uint16 p;

	int16 np = TWO_PI * r;
	real32 npR = 1.0f / (real32) (np);

	for (p = 0; p <= np; p++) {
		real32 a = TWO_PI * (real32) p * npR;
		uint8 px = (uint8) (x + r * cosf(a));
		uint8 py = (uint8) (y + r * sinf(a));

		oledSetPixel(px, py, true);
	}
} // oledDrawCircle


void oledFillDisplay(uint8 c) { // fill whole display using character c
	idx line, x;

	for (line = 0; line < SSD1X06_LCDHEIGHTCHARS; line++)
		for (x = 0; x < SSD1X06_LCDWIDTH; x += SSD1X06_CHAR_WIDTH)
			oledDisplayChar(line, x, c);
} // oledFillDisplay


void oledDisplayInt32(uint8 line, uint8 x, int32 V) {
	uint8 S[16];
	uint8 c;
	int32 Rem, NewV;

	x *= SSD1X06_CHAR_WIDTH;

	if (V < 0) {
		oledDisplayChar(line, x, '-');
		x += SSD1X06_CHAR_WIDTH;
		V = -V;
	}

	c = 0;
	do {
		NewV = V / 10;
		Rem = V - (NewV * 10);
		S[c++] = Rem + '0';
		V = NewV;
	} while (V > 0);

	do {
		oledDisplayChar(line, x, S[--c]);
		x += SSD1X06_CHAR_WIDTH;
	} while (c > 0);

} // oledDisplayInt


void oledDisplayString(uint8 line, uint8 x, const char *s, uint8 rvsField) {

	if (line > SSD1X06_LCDHEIGHTCHARS - 1)
		line = SSD1X06_LCDHEIGHTCHARS - 1;

	if (rvsField)
		rvsField = 0x80;

	x *= 6;
	while (*s != (char) 0) {
		oledDisplayChar(line, x, *s++ ^ rvsField);
		x += 6;
	}

} // oledDisplayString


void oledDisplayByte(uint8 line, uint8 x, uint8 b) {

	oledSetColmnPage(line, x);
	SIOWrite(oledSel, 0x40, b);
} // oledDisplayByte


void oledDisplayChar(uint8 line, uint8 x, uint8 c) {
	idx i;
	uint16 p;
	uint8 B[8];

	oledSetColmnPage(line, x);

	uint8 xorMask = c & 0x80 ? 0xff : 0x00; // if MSB set display character reverse field

	B[0] = xorMask;

	p = (uint16) ((c & 0x7f) - 32) * 5;
	for (i = 0; i < 5; i++)
		B[i + 1] = ssdfont[p + i] ^ xorMask;

	SIOWriteBlock(oledSel, 0x40, 6, &B[0]);

} // oledDisplayChar


real32 WindowWidthM = 800.0f;

void oledDisplayNavTreck(void) {
	real32 oledNavScaleR = 128.0f / WindowWidthM;

	static uint8 pxP = 0;
	static uint8 pyP = 0;
	uint8 px, py;

	px = Limit((uint8)(32.0f - Nav.C[NorthC].Pos * oledNavScaleR), 0, 63);
	py = Limit((uint8)(64.0f + Nav.C[EastC].Pos * oledNavScaleR), 0, 127);

	if ((pxP != px) || (pyP != py)) {
		oledSetPixel(px, py, true);
		pxP = px;
		pyP = py;
	}

} // oledDisplayNavTreck


void InitOLED(void) {
	uint8 x, y;

	memset(oledPixels, 0, sizeof(oledPixels));

	oledStart();

	oledFillDisplay(' ');

// ??? initial screen

	//oledDisplayInt32(7, 30, (int32) WindowWidthM);

} // InitOLED

void UpdateOLED(void) {

	if (mSTimeout(oledDisplayUpdate)) {
		mSTimer(oledDisplayUpdate, 1000);

		if (State != InFlight) {
			if (!Armed())
				oledDisplayString(0, 0, "Not ARMED  ", 0);
			else if (F.OriginValid)
				oledDisplayString(0, 0, "FLY        ", 0);
			else if (F.GPSValid)
				oledDisplayString(0, 0, "WAIT Origin", 0);
			else {
				oledDisplayString(0, 0, "WAIT GPS   ", 0);
				oledDisplayInt32(0, 9, GPS.noofsats);
			}

		}
	}

} // UpdateOLED

#else

void InitOLED(void) {} // oledInit
void UpdateOLED(void) {} // oledInit

#endif