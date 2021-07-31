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

#ifndef _pinsf4v4_h
#define _pinsf4v4_h

#include "UAVX.h"

PinDef CPPMPin = {true, PA0, PWMPinConfig, PWMIn_2_1};

PinDef SoftSerialTxPin = // old RC2 Pin
	{true, PA1, OutPinConfig }; // use TIM5

PinDef PWMPins[MAX_PWM_OUTPUTS] = {
		// Drives 1-4
	{true, PB9, PWMPinConfig, PWMOut_4_4},
	{true, PB8, PWMPinConfig, PWMOut_4_3},
	{true, PB7, PWMPinConfig, PWMOut_4_2},
	{true, PB6, PWMPinConfig, PWMOut_4_1},
	{true, PC6, PWMPinConfig, PWMOut_8_1}, 
	{true, PC7, PWMPinConfig, PWMOut_8_2},

	// Drives 4-8 for brushless
	{true, PA6, PWMPinConfig, PWMOut_3_1},
	{true, PA7, PWMPinConfig, PWMOut_3_2}, 
	{true, PB0, PWMPinConfig, PWMOut_3_3},
	{true, PB1, PWMPinConfig, PWMOut_3_4}
};

PinDef GPIOPins[MAX_GPIO_PINS] = {
	{true, PA4, OutPinConfig,}, // 12? Beeper
	{true, PC10, InpPinConfig,}, // Armed
	{true, PC9, InpPinConfig,}, // Landing
	{true, PA5, OutPinConfig,}, // Aux1
	{true, PA15, OutPinConfig,}, // Aux2
#if defined(USE_AUX3_PROBE_PIN)
	{true, PC8, OutPinConfig,}, // Aux3 (Probe for timing diagnostics)
#else
	{true, PC8, InpPinConfig,}, // Aux3 (WPMissionEnable active low)
#endif
	{true, PC14, InpPinConfig,}, // MPU6XXXIntSel 
	{true, PC15, InpPinConfig,} // HMC5XXXRdySel
};

PinDef LEDPins[MAX_LED_PINS] = { //
	{true, PB3, LEDPinConfig,}, // Yellow
	{true, PB4, LEDPinConfig,}, // Red
	{true, PC11, LEDPinConfig,}, // Blue
	{true, PC12, LEDPinConfig,},// Green
};

AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = {			
	{true, ADC1, PC0, ADC_Channel_10, DMA2_0_0_noIRQn, 1 }, // RF
	{true, ADC1, PC1, ADC_Channel_11, DMA2_0_0_noIRQn, 2 }, // Current aka Aux4
	{true, ADC1, PC2, ADC_Channel_12, DMA2_0_0_noIRQn, 3 }, // Volts
};

SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { // Tx, Rx
	{false, }, // mapped to USBSerial
	{true, USART1, PA9, PA10, 115200, true, false, USART1Config},
	{true, USART2, PA2, PA3, 115200, true, false, USART2Config},
	{true, USART3, PB10, PB11, 115200, true, false, USART3Config},
	{false, UART4, PB10, PB11, 115200, true, false, UART4Config},
	{false, 0 } // SoftSerial
};

I2CPortDef I2CPorts[MAX_I2C_PORTS] = {
	{false, 0, }, // notionally I2C0
	{false, I2C1, PB6, PB7, GPIO_AF_I2C1 },//
	{false, I2C2, PB10, PB11, GPIO_AF_I2C2 }, // USART3
	{false, I2C3, }
};

SPIPortDef SPIPorts[MAX_SPI_PORTS] = { // SCK, MISO, MOSI
	{false, 0, }, // notionally SPI0
	{true, SPI1, {PA5, PA6, PA7}}, // Aux1, RC5, RC6 <- SPIESC?
	{true, SPI2, {PB13, PB14, PB15}}, //
	{true, SPI3, {PC10, PC11, PC12}} // Arming, LED3, LED2
};

PinDef USBDisconnectPin = { false, 0, };

DevDef busDev[maxDevSel] = {
	{true, imuSel, mpu6000IMU, true, 2, 0, PB12},
	{true, baroSel, ms5611Baro, true, 2, 0, PC5},
	{true, magSel, hmc5xxxMag, true, 2, 0, PC4},
#if defined(USE_ARM_BLACKBOX_FLASH)
	{true, memSel, spiFlashMem, true, 2, 0, PC3},
#else
	{false, memSel, spiFlashMem, true, 2, 0, PC3},	
#endif
	{false, gpsSel, },
	{false, rfSel, },
	{false, asSel, },
	{false, flowSel, }, 
#if defined(USE_OLED)
	{true, oledSel, oledDisplay, false, 2, SSD1X06_I2C_ADDRESS,},
#else
	{false, oledSel, oledDisplay,},
#endif
#if defined(USE_SPI_ESC)
	{true, escSPISel, ESCSPI, true, 1, 0, PC13}, // PC13 NC no select required
#else
	{false, escSPISel, }, // PC13 not connected
#endif
	{false, escI2CSel, ESCI2C, false, 2, ESCI2C_ID, }
};

const uint8 IMUQuadrant = 0;
const uint8 MagQuadrant = 0;

const uint8 currGimbalType = noGimbal;
const boolean ledsLowOn = false;
const boolean beeperLowOn = true;

void InitTarget(void) {

	TelemetrySerial = Usart1;
	
	InitSerialPort(TelemetrySerial, true, false);

	switch (CurrRxType) {
	case CPPMRx:
		CurrMaxPWMOutputs = (UsingDCMotors || (CurrESCType == ESCSPI)) ? 4 : 10;
		F.HaveSerialRC = false;
		break;
	default:
		CurrMaxPWMOutputs = (UsingDCMotors) ? 4 : 10;
		F.HaveSerialRC = true;
		RCSerial = Usart2;
		InitSerialPort(RCSerial, false, CurrRxType == FutabaSBusRx);
		break;
	} // switch
	
	F.HaveGPS = CurrGPSType != NoGPS;
    GPSSerial = Usart3;
	InitSerialPort(GPSSerial, false, false);
			
	FrSkySerial = SoftSerial;
	InitSoftSerialTxTimer();	

} // uavxf4v4Init
#endif

