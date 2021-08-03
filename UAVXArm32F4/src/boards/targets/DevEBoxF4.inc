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

#ifndef _discoveryf4ml_h
#define _discoveryf4ml_h

#include "UAVX.h"

const PinDef CPPMPin = { false, }; //PA0, PWMPinConfig, PWMIn_2_1 };

const PinDef SoftSerialTxPin = // old RC2 Pin
		{ false, }; //PA1, OutPinConfig }; // use TIM5

const PinDef PWMPins[MAX_PWM_OUTPUTS] = {
// Drives 1-4
		{ true, PB9, PWMPinConfig, PWMOut_4_4 }, //
		{ true, PB8, PWMPinConfig, PWMOut_4_3 }, //
		{ true, PB7, PWMPinConfig, PWMOut_4_2 }, //
		{ true, PB6, PWMPinConfig, PWMOut_4_1 }, //
		{ true, PC6, PWMPinConfig, PWMOut_8_1 }, //
		{ true, PC7, PWMPinConfig, PWMOut_8_2 }, //

		// Drives 4-8 for brushless
		{ false, PA6, PWMPinConfig, PWMOut_3_1 }, //
		{ false, PA7, PWMPinConfig, PWMOut_3_2 },//
		{ false, PB0, PWMPinConfig, PWMOut_3_3 }, //
		{ false, PB1, PWMPinConfig, PWMOut_3_4 } };

const PinDef GPIOPins[MAX_GPIO_PINS] = { //
		{ true, PE14, OutPinConfig, }, // BeeperSel
		{ true, PB0, InpPinConfig, }, // ArmedSel
		{ true, PE13, InpPinConfig, }, // LandingSel
		{ true, PE11, OutPinConfig, }, // Aux1Sel
		{ false, }, 					// Aux2Sel
#if defined(USE_AUX3_PROBE_PIN)
		{ true, PE9, OutPinConfig,}, // WPMissionOrProbeSel
#else
		{ true, PE9, InpPinConfig, }, // WPMissionOrProbeSel
#endif
		{ false, }, // InverterSel
		{ false, }, // MPU6XXXIntSel
		{ false, } // HMC5XXXRdySel
};

const PinDef LEDPins[MAX_LED_PINS] = { //
		{ true, PD13, LEDPinConfig, }, // Yellow
		{ true, PD14, LEDPinConfig, }, // Red
		{ true, PD11, LEDPinConfig, }, // Blue
		{ true, PD12, LEDPinConfig, }, // Green
};

const AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = { //
		{ true, ADC1, PC0, ADC_Channel_10, DMA2_0_0_noIRQn, 1 }, // RF
		{ true, ADC1, PC1, ADC_Channel_11, DMA2_0_0_noIRQn, 2 }, // Current aka Aux4
		{ true, ADC1, PC2, ADC_Channel_12, DMA2_0_0_noIRQn, 3 }, // Volts
};

const SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { // Tx, Rx
		{ false, }, // mapped to USBSerial
		{ false, USART1, PA9, PA10, 115200, true, false, USART1Config }, // USB Clash
		{ true, USART2, PD5, PD6, 115200, true, false, USART2Config }, // Sbus/Spektrum
		{ true, USART3, PD8, PD9, 115200, true, false, USART3Config }, //
		{ true, UART4, PC10, PC11, 115200, true, false, UART4Config }, //
		{ true, 0 } // SoftSerial
};

const I2CPortDef I2CPorts[MAX_I2C_PORTS] = { { false, 0, }, // notionally I2C0
		{ false, I2C1, PB6, PB7, GPIO_AF_I2C1 }, //
		{ true, I2C2, PB10, PB11, GPIO_AF_I2C2 }, // USART3
		{ false, I2C3, } };

const SPIPortDef SPIPorts[MAX_SPI_PORTS] = { // SCK, MISO, MOSI
		{ false, 0, }, // notionally SPI0
		{ true, SPI1, { PA5, PA6, PA7 } }, // Aux1, RC5, RC6 <- SPIESC?
		{ true, SPI2, { PB13, PB14, PB15 } }, //
		{ true, SPI3, { PC10, PC11, PC12 } } // Arming, LED3, LED2
};

const DevDef busDev[maxDevSel] = { //
		{ true, imuSel, mpu6050IMU, false, 2, MPU_0x68_ID, }, //
		{ true, baroSel, ms5611Baro, false, 2, MS56XX_0_ID, }, //
		{ true, magSel, hmc5xxxMag, false, 2, HMC5XXX_ID, },
#if defined(USE_ARM_BLACKBOX_FLASH)
		{ false, memSel, ArmFlashMem,}, // does not use either SPI or I2C
#else
		{ true, memSel, i2cEEPROMMem, false, 2, EEPROM_ID, },
#endif
		{ false, gpsSel, }, //
		{ false, rfSel, }, //
		{ false, asSel, }, //
		{ false, flowSel, }, //
		{ false, oledSel, oledDisplay, }, //
		{ false, escSPISel, }, //
		{ false, escI2CSel, ESCI2C, false, 2, ESCI2C_ID, } };

const PinDef USBDisconnectPin = { false, 0, };

const uint8 IMUQuadrant = 0;
const uint8 MagQuadrant = 0;

const uint8 currGimbalType = noGimbal;
const boolean ledsLowOn = true;
const boolean beeperLowOn = true;

void InitTarget(void) {

	TelemetrySerial = Usart1;
	InitSerialPort(TelemetrySerial, true, false);

	// CurrRxType always FutabaSBus - forced in params.c

	CurrMaxPWMOutputs = (UsingDCMotors) ? 4 : 10;
	F.HaveSerialRC = true;
	RCSerial = Usart2;
	InitSerialPort(RCSerial, false, CurrRxType == FutabaSBusRx);

	GPSSerial = Usart3;
	InitSerialPort(GPSSerial, false, false);
	F.HaveGPS = CurrGPSType != NoGPS; // check for port limitation V3

	FrSkySerial = SoftSerial;
	InitSerialPort(FrSkySerial, true, false);


} // InitTarget

#endif

