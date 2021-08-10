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

#ifndef _pinsomnibusf4_h
#define _pinsomnibusf4_h

#include "UAVX.h"

/*
#ifdef OMNIBUSF4SD
// These inverter control pins collide with timer channels on CH5 and CH6 pads.
// Users of these timers/pads must un-map the inverter assignment explicitly.
#define INVERTER_PIN_UART6      PC8 // Omnibus F4 V3 and later
#define INVERTER_PIN_UART3      PC9 // Omnibus F4 Pro Corners
#elif defined(EXUAVF4PRO)
#define INVERTER_PIN_UART6      PC8
#else
#define INVERTER_PIN_UART1      PC0 // DYS F4 Pro; Omnibus F4 AIO (1st gen) have a FIXED inverter on UART1
#endif
*/

// I do not like USB!!!!!
// Flight Control Boards do not usually use the default USB VBUS pins
// so disable library VBUS definition

#define USB_VCP_DISABLE_VBUS
#define USB_VCP_DISABLE_ID

#define VBUS_SENSING_ENABLED
const PinDef VBusPin {true, PC5, GPIO_Mode_IN, GPIO_OType_OD, GPIO_PuPd_NOPULL, };

PinDef CPPMPin = {false, };

PinDef SoftSerialTxPin = {false, };

PinDef RCPins[MAX_RC_INPUTS] = {

};

PinDef PWMPins[MAX_PWM_OUTPUTS] = {
	{true, PB0, PWMPinConfig, PWMOut_3_3},
	{true, PB1, PWMPinConfig, PWMOut_3_4},
	{true, PA3, PWMPinConfig, PWMOut_2_4},
	{true, PA2, PWMPinConfig, PWMOut_2_3},	 
#if defined(IS_PRO)
    {true, PB6, PWMPinConfig, PWMOut_4_1},
#else
    {true, PA1, PWMPinConfig, PWMOut_5_2},
#endif	
	{true, PA8, PWMPinConfig, PWMOut_1_1},
};

PinDef GPIOPins[MAX_GPIO_PINS] = { //
	{true, PB4, OutPinConfig,}, // Beeper
	{false, }, // Armed
	{false,}, // Landing
	{false, }, // Aux1
	{true, PC0, OutPinConfig,}, // Aux2/Inverter
	{true, PB14, OutPinConfig,}, // Probe (Aux3)
	{true, PC4, InpPinConfig,}, // MPU6XXXIntSel 
	{false, } // HMC5XXXRdySel					
};

PinDef LEDPins[MAX_LED_PINS] = { //
	{true, PB5, LEDPinConfig,}, 
};

//PinDef WSPin = 
//	{true, PB6, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL, 
//		{true, TIM4, TIM_Channel_1, TIM_DMA_CC1, &(TIM4->CCR1), GPIO_AF_TIM4 },
//		{true, DMA_Channel_0, DMA2_Stream2}, DMA2_Stream2_IRQn };

SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { 
	{true, }, // mapped to USBSerial
#if defined(GKE)
	{true, USART1, PA9, PA10, 115200, true, false, USART1Config,},
#else
	{false, USART1, },
#endif
	{false, USART2, PA14, PA15, 9600, true, false, USART2Config,},
	{true, USART3, PB10, PB11, 115200, true, false, USART3Config,},
	{false, UART4, PB10, PB11, 115200, true, false, UART4Config,},
	{false, 0 } // SoftSerial
};

AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = { //
	{false,  }, 
	{true, ADC1, PC1, ADC_Channel_11, DMA2_0_0_noIRQn, 2}, // Current
	{true, ADC1, PC2, ADC_Channel_12, DMA2_0_0_noIRQn, 3}, // Volts
};

I2CPortDef I2CPorts[MAX_I2C_PORTS] = {
	{false, 0, }, // notionally I2C0
	{true, I2C1, PB6, PB7, GPIO_AF_I2C1},
	{true, I2C2, PB10, PB11, GPIO_AF_I2C2}, 
	{false, I2C3, }
};

SPIPortDef SPIPorts[MAX_SPI_PORTS] = { // SCK, MISO, MOSI
	{false, 0, }, // notionally SPI0
	{true, SPI1, {PA5, PA6, PA7}}, //
	{true, SPI2, {PB13, PB14, PB15}}, //
	{true, SPI3, {PC10, PC11, PC12}}
};

DevDef busDev[maxDevSel] = { // MUST be in enum order otherwise CheckBusDev fails	 	
	{true, imuSel, mpu6000IMU, true, 1, 0, PA4},
	{false, baroSel, noBaro },//baroSel, ms5611Baro, false, 2, MS56XX_ID,},
	{false, magSel, noMag, }, //magSel, hmc5xxxMag, false, 2, HMC5XXX_ID,},
	{false, memSel, noMem, }, 
	{false, gpsSel, },
	{false, rfSel, },
	{false, asSel, },
	{false, flowSel, },
	{false, escSPISel, },
	{false, escI2CSel, }
};


const uint8 IMUQuadrant = 1;
const uint8 MagQuadrant = 1;

const uint8 currGimbalType = noGimbal;
const boolean ledsLowOn = false;
const boolean beeperLowOn = true;

#define GKE // for debugging until I get USB working!!!!

#if defined(GKE)

void InitTarget(void) {

	TelemetrySerial = Usart1; 
	InitSerialPort(TelemetrySerial, true, false);

	F.HaveSerialRC = false; // use CPPM for now

	GPSSerial = GPSSerial = Usart3;
	InitSerialPort(GPSSerial, false, false);

	CurrMaxPWMOutputs = 4;

} // InitTarget

#else

void InitTarget(void) {

	InitPin(&VBusPin);
	TelemetrySerial = USBSerial;
	InitSerialPort(TelemetrySerial, true, false);

	F.HaveSerialRC = true;
	RCSerial = Usart1;
	InitSerialPort(RCSerial, false, true);

	GPSSerial = GPSSerial = Usart3;
	InitSerialPort(GPSSerial, false, false);

	CurrMaxPWMOutputs = 4;

} // InitTarget

#endif

#endif

