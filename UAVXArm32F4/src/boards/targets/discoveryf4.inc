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

//    UAVX is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.
//    If not, see http://www.gnu.org/licenses/

#ifndef _discoveryf4_h
#define _discoveryf4_h

#include "UAVX.h"

//#define VBUS_SENSING_ENABLED
//#define USB_VCP_DISABLE_VBUS // disable library VBUS definition
//#define USB_VCP_DISABLE_ID

const PinDef SoftSerialTxPin = { false, };

const PinDef CPPMPin = { false, };

const PinDef PWMPins[MAX_PWM_OUTPUTS] = { //
		{ true, PB9, PWMPinConfig, PWMOut_4_4 }, //
		{ true, PB8, PWMPinConfig, PWMOut_4_3 }, //
		{ true, PB7, PWMPinConfig, PWMOut_4_2 }, //
		{ true, PB6, PWMPinConfig, PWMOut_4_1 }, //
		{ false, 0 }, //
		{ false, 0 }, //
		{ false, 0 }, //
		{ false, 0 }, //
		{ false, 0 }, //
		{ false, 0 } //
};

const PinDef GPIOPins[MAX_GPIO_PINS] = { //
		{ true, PA15, OutPinConfig, }, // Beeper
		{ false, 0, }, // Landing
		{ false, 0, }, // Aux1/WS2812 (WS2812 hard coded)
		{ false, 0, }, // Aux2/SoftUSARTTx
		{ true, PB4, OutPinConfig, }, // Aux3/Probe
		{ false, 0, }, // Aux4/Inverter
		{ false, 0, }, // MPU6XXXIntSel
		{ false, 0, }, // HMC5XXXRdySel
};

const PinDef LEDPins[MAX_LED_PINS] = {
		{ true, PD13, OutPinConfig, }, // Yellow
		{ true, PD14, OutPinConfig, }, // Red
		{ true, PD15, OutPinConfig, }, // Blue
		{ true, PD12, OutPinConfig, } // Green
};

const AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = { //
		{ true, ADC1, PC0, ADC_Channel_10, DMA2_0_0_noIRQn, 1 }, // RF
		{ true, ADC1, PC1, ADC_Channel_11, DMA2_0_0_noIRQn, 2 }, // Current
		{ true, ADC1, PC2, ADC_Channel_12, DMA2_0_0_noIRQn, 3 }, // Volts

		{ false, }, //
		{ false, }, //
		{ false, } //
};

const SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { // Tx, Rx
		{ true, }, // mapped to USBSerial
		{ false, USART1, PA9, PA10, 115200, USART_Parity_No, USART_StopBits_2, false, USART1Config }, // USB clash
		{ true, USART2, PA2, PA3, 9600, USART_Parity_No, USART_StopBits_2, false, USART2Config }, //
		{ true, USART3, PD8, PD9, 115200, USART_Parity_No, USART_StopBits_2, false, USART3Config }, //
		{ true, UART4, PC10, PC11, 115200, USART_Parity_No, USART_StopBits_2, false, UART4Config }, //
		{ true, 0 } // SoftSerial
};

const I2CPortDef I2CPorts[MAX_I2C_PORTS] = { { false, 0, }, // notionally I2C0
		{ false, I2C1, PB6, PB7, GPIO_AF_I2C1 }, //
		{ true, I2C2, PB10, PB11, GPIO_AF_I2C2 }, //
		{ false, I2C3, } //
};

const SPIPortDef SPIPorts[MAX_SPI_PORTS] = { // SCK, MISO, MOSI
		{ false, 0, }, // notionally SPI0
		{ false, SPI1, { PA5, PA6, PA7 } }, // Aux1, RC5, RC6
		{ false, SPI2, { PB13, PB14, PB15 } }, //
		{ false, SPI3, { PC10, PC11, PC12 } } // Arming, LED3, LED2
};

const DevDef busDev[] = { //
		{ true, imuSel, mpu6050IMU, false, 2, MPU_0x68_ID }, // imuSel
		{ true, baroSel, ms5611Baro, false, 2, MS56XX_ID }, // baroSel
		{ true, magSel, hmc5xxxMag, false, 2, HMC5XXX_ID }, // magSel
		{ false, memSel, noMem, true, 0 }, // memSel
		{ false, gpsSel, }, //
		{ false, rfSel, }, //
		{ false, asSel, }, //
		{ false, flowSel, }, //
		{ false, oledSel, oledDisplay, }, //
		{ false, escSPISel, }, //
		{ false, escI2CSel, ESCI2C, false, 2, ESCI2C_ID, }, //
		{ true, mag2Sel, hmc5xxxMag, false, 2, HMC5XXX_ID }, // mag2Sel
};

const uint8 IMUQuadrant = 0;
const uint8 MagQuadrant = 0;

const uint8 currGimbalType = noGimbal;
const boolean ledsLowOn = true;
const boolean beeperLowOn = true;
const uint8 CurrBattVoltsAnalogSel = BattVoltsAnalogSel;
uint8 CurrMagSel = magSel;

void InitTarget(void) {

	CurrMaxPWMOutputs = 4;

	TelemetrySerial = Uart4; //Uart4; //USBSerial;
	InitSerialPort(TelemetrySerial, true);

	F.HaveSerialRC = true;
	RCSerial = Usart2;
	InitSerialPort(RCSerial, false);

	F.HaveGPS = true; // removed for debug
	GPSSerial = Usart3;
	InitSerialPort(GPSSerial, false);

	FrSkySerial = SoftSerial;
	InitSerialPort(FrSkySerial, true);

} // InitTarget

#endif

