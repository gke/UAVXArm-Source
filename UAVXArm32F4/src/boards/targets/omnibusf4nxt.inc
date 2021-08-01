

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
#define VBUS_SENSING_PIN        PC15
#define VBUS_SENSING_ENABLED

#include "UAVX.h"

PinDef SoftSerialTxPin = {false, };

PinDef CPPMPin = {false, };

PinDef RCPins[MAX_RC_INPUTS] = {		
	{true, PB7, PWMPinConfig, PWMIn_4_2}, // PPM / UART1_RX
};
		
PinDef PWMPins[MAX_PWM_OUTPUTS] = {
		// Drives 1-4
	{true, PB5, PWMPinConfig, PWMOut_3_2}, // D(1, 5, 5)
	{true, PB4, PWMPinConfig, PWMOut_3_1}, // D(1, 4, 5)
	{true, PB0, PWMPinConfig, PWMOut_3_3}, // D(1, 7, 5)
	{true, PB1, PWMPinConfig, PWMOut_3_4}, // D(1, 2, 5)
		
	{true, PC9, PWMPinConfig, PWMOut_8_4}, // D(2, 7, 7)
	{true, PC8, PWMPinConfig, PWMOut_8_3}, // D(2, 2, 0)

	{ true, PA9, PWMPinConfig, PWMOut_1_2},  // LED
	{ true, PB6, PWMPinConfig, PWMOut_4_1 }, // SS1 TX / UART1_TX
		
	{ false, },
	{ false, }
};

PinDef GPIOPins[MAX_GPIO_PINS] = {
	{true, PB4, OutPinConfig,}, // Beeper
	{false, }, // Armed
	{false,}, // Landing
	{false, }, // Aux1
	{true, PC0, OutPinConfig,}, // Aux2/Inverter
	{true, PB14, OutPinConfig,}, // Probe (Aux3)
	{true, PC4, InpPinConfig,}, // MPU6XXXIntSel 
	{false, } // HMC5XXXRdySel	
};

PinDef WSPin = // Aux1 Pin for UAVXF4V3
	{false, PA9, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL, 
		{true, TIM1, TIM_Channel_2, TIM_DMA_CC2, &(TIM1->CCR2), GPIO_AF_TIM1 },
		{true, DMA_Channel_0, DMA2_Stream2}, DMA2_Stream2_IRQn }; // ???

PinDef LEDPins[MAX_LED_PINS] = {
	{true, PB5, LEDPinConfig,},
};

const AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = {			
	{false,  }, 
	{true, ADC1, PC1, ADC_Channel_11, DMA2_0_0_noIRQn, 2}, // Current
	{true, ADC1, PC2, ADC_Channel_12, DMA2_0_0_noIRQn, 3}, // Volts
};
	
SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { 
	{true, }, // mapped to USBSerial
	{true, USART1, PB6, PB7, 115200, true, false, USART1Config,},
	{true, USART2, PA2, PA3, 9600, true, false, USART2Config,},
	{true, USART3, PB10, PB11, 115200, true, false, USART3Config,},
	{true, UART4, PA0, PA1, 115200, true, false, UART4Config,},
	{false, 0 } // SoftSerial
};

I2CPortDef I2CPorts[MAX_I2C_PORTS] = {
	{false, 0, }, // notionally I2C0
	{true, I2C1, 
			PB8, // PB8, alt MST8 
			PB9, // PB9, alt MST7
			GPIO_AF_I2C1},
	{false, I2C2, 
			PB10, // PB10 alt UART3TX
			PB11, // PB11 alt UART3RX
			GPIO_AF_I2C2}, // used by usart3
	{false, I2C3, }
};

SPIPortDef SPIPorts[MAX_SPI_PORTS] = { // SCK, MISO, MOSI
	{false, 0, }, // notionally SPI0
	{true, SPI1, { PA5, PA6, PA7}}, //
	{true, SPI2, { PB13, PB14,  PC3 }}, // PC3????
	{true, SPI3, { PC10, PC11, PC12   // PC10, alt UART3_TX, UART4_TX PC11, alt UART3_RX, UART4_RX PC12, alt UART5_TX
	}} 
};

PinDef USBDisconnectPin = {true, PA12,};

const DevDef busDev[] = {
    {true, imuSel, mpu6000IMU, true, 1, 0, PA4},
	{true, ms5611Baro, useSPI, 2, }, // baroSel
	{true, hmc5xxxMag, useI2C, 1, HMC5XXX_ID}, // magSel 
	{false, spiFlashMem, useSPI, 2,}, // memSel 
	{false, gpsSel, },
	{false, rfSel, },
	{false, asSel, },
	{false, flowSel, },
	{false, oledSel, oledDisplay,},
	{false, escSPISel, }, // PC13 not connected
	{false, escI2CSel, ESCI2C, false, 2, ESCI2C_ID, }
};

const uint8 IMUQuadrant = 1;
const uint8 MagQuadrant = 1;

const uint8 currGimbalType = noGimbal;
const boolean ledsLowOn = true;
const boolean beeperLowOn = true;

void InitTarget(void) {
	
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    TelemetrySerial = USBSerial;
// No should be USBInit    InitSerialPort(TelemetrySerial, true, false);

	CurrRxType = FutabaSBusRx;
	
	switch (CurrRxType) {
	case CPPMRx:
		CurrMaxPWMOutputs = 4;

		F.HaveSerialRC = false;
		break;
	default:
		CurrMaxPWMOutputs = 4;
		F.HaveSerialRC = true;
		RCSerial = Usart2;
		InitSerialPort(RCSerial, false, CurrRxType == FutabaSBusRx);
		break;
	} // switch
	
	GPSSerial = GPSSerial = Uart4;
	InitSerialPort(GPSSerial, false, false);

} // omnibusf4nxtInit
#endif
