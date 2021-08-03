
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

#ifndef _omnibusf4nxt_h
#define _omnibusf4nxt_h

#define USE_VCP
#define VBUS_SENSING_PIN        PC5
#define VBUS_SENSING_ENABLED

#include "UAVX.h"

const PinDef USBDisconnectPin = { true, PA12, }; // initialisation done in USBConnect

const PinDef SoftSerialTxPin = { false, };

const PinDef CPPMPin = { false, };

const PinDef RCPins[MAX_RC_INPUTS] = { { false, }}; // SBus only No CPPM

const PinDef PWMPins[MAX_PWM_OUTPUTS] = {
// Drives 1-4
		{ true, PB5, PWMPinConfig, PWMOut_3_2 }, // D(1, 5, 5)
		{ true, PB4, PWMPinConfig, PWMOut_3_1 }, // D(1, 4, 5)
		{ true, PB0, PWMPinConfig, PWMOut_3_3 }, // D(1, 7, 5)
		{ true, PB1, PWMPinConfig, PWMOut_3_4 }, // D(1, 2, 5)

		{ false, PC9, PWMPinConfig, PWMOut_8_4 }, // D(2, 7, 7)
		{ false, PC8, PWMPinConfig, PWMOut_8_3 }, // D(2, 2, 0)

		{ false, }, //
		{ false, }, //

		{ false, }, //
		{ false, } //
};

const PinDef GPIOPins[MAX_GPIO_PINS] = { //
		{ false, PC13, OutPinConfig, }, // Beeper
		{ false, }, // Armed
		{ false, }, // Landing
		{ false, }, // Aux1
		{ false, PC0, OutPinConfig, }, // Aux2/Inverter
		{ false, PB14, OutPinConfig, }, // Probe (Aux3)
		{ false, PC4, InpPinConfig, }, // MPU6XXXIntSel
		{ false, } // HMC5XXXRdySel
};

const PinDef LEDPins[MAX_LED_PINS] = {
		{ false, }, // Yellow
		{ true, PB2, LEDPinConfig, }, // Red
		{ false, }, // Blue
		{ false, } // Green
};

const AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = { //
		{ false, }, //
		{ true, ADC1, PC1, ADC_Channel_11, DMA2_0_0_noIRQn, 2 }, // Current
		{ true, ADC1, PC0, ADC_Channel_12, DMA2_0_0_noIRQn, 3 }, // Volts
		{ false, }, //
		{ false, }, //
		{ false, } //
		};

const SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { //
		{ true, }, // mapped to USBSerial
		{ true, USART1, PB6, PB7, 115200, true, false, USART1Config, }, //
		{ false, USART2, PA2, PA3, 9600, true, false, USART2Config, }, //
		{ false, USART3, PB10, PB11, 115200, true, false, USART3Config, }, //
		{ true, UART4, PA0, PA1, 115200, true, false, UART4Config, }, //
		{ false, 0 } // SoftSerial
};

const I2CPortDef I2CPorts[MAX_I2C_PORTS] = { //
		{ false, 0, }, // notionally I2C0
		{ true, I2C1, PB8, PB9, GPIO_AF_I2C1 }, //
		{ false, I2C2, PB10,  PB11,  GPIO_AF_I2C2 }, // used by usart3
		{ false, I2C3, } //
};

const SPIPortDef SPIPorts[MAX_SPI_PORTS] = { // SCK, MISO, MOSI
		{ false, 0, }, //
		{ true, SPI1, { PA5, PA6, PA7 } }, //
		{ false, SPI2, { PB13, PB14, PC3 } }, //
		{ false, SPI3, {PC10, PC11, PC12 } } // USART clashes
};

const DevDef busDev[] = {
		{ true, imuSel, mpu6000IMU, true, 1, 0, PB12 },
		{ false, baroSel,  }, // baroSel
		{ false, magSel, hmc5xxxMag, false, 1, HMC5XXX_ID,},
		{ false, memSel, spiFlashMem, true, 2, 0, PC14}, //
		{ false, gpsSel, }, //
		{ false, rfSel, }, //
		{ false, asSel, },//
		{ false, flowSel, },//
		{ false, oledSel, oledDisplay, },//
		{ false, escSPISel, },//
		{ false, escI2CSel, }//
};

const uint8 IMUQuadrant = 1;
const uint8 MagQuadrant = 1;

const uint8 currGimbalType = noGimbal;
const boolean ledsLowOn = true;
const boolean beeperLowOn = true;

void InitTarget(void) {

	// ????
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//?????

	TelemetrySerial = USBSerial;
	RxEnabled[USBSerial] = true;
	USBConnect();

	// CurrRxType always FutabaSBus - forced in params.c

	CurrMaxPWMOutputs = 4;

	F.HaveSerialRC = true;
	RCSerial = Usart1;
	InitSerialPort(RCSerial, false, CurrRxType == FutabaSBusRx);

	F.HaveGPS = false; // removed for debug
	GPSSerial = GPSSerial = Uart4;
	InitSerialPort(GPSSerial, false, false);

	FrSkySerial = SoftSerial;
	InitSerialPort(FrSkySerial, true, false);

} // omnibusf4nxtInit
#endif
