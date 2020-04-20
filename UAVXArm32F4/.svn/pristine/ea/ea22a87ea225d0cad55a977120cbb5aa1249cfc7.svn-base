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

#include "UAVX.h"

PinDef RCPins[MAX_RC_INPUTS] = {
	{ true, GPIOB, GPIO_Pin_7, GPIO_PinSource7, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, 
		{true, TIM4, TIM_Channel_2, TIM_IT_CC2, 0, GPIO_AF_TIM4 }, TIM4_IRQn },
		{false, },
		{false, },
		{false, },
		{false, },
		{false, },
		{false, },
		{false, }
};
		

PinDef PWMPins[MAX_PWM_OUTPUTS] = {
		// Drives 1-4
	{ true, GPIOB, GPIO_Pin_5, GPIO_PinSource5, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, 
		{true, TIM3, TIM_Channel_2, 0, &(TIM3->CCR2), GPIO_AF_TIM3 }}, //
	{ true, GPIOB, GPIO_Pin_4, GPIO_PinSource4, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, 
		{true, TIM3, TIM_Channel_1, 0,&(TIM3->CCR1), GPIO_AF_TIM3 }}, //
	{ true, GPIOB, GPIO_Pin_0, GPIO_PinSource0, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, 
		{true, TIM3, TIM_Channel_3, 0, &(TIM3->CCR3), GPIO_AF_TIM3 }}, //
	{ true, GPIOB, GPIO_Pin_1, GPIO_PinSource1, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, 
		{true, TIM3, TIM_Channel_4, 0, &(TIM3->CCR4), GPIO_AF_TIM3 }},
		
	{ true, GPIOC, GPIO_Pin_9, GPIO_PinSource9, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		{true, TIM8, TIM_Channel_4, 0, &(TIM8->CCR4), GPIO_AF_TIM8 }}, // optionally WS2812
	{ true, GPIOC, GPIO_Pin_8, GPIO_PinSource8, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, 
		{true, TIM8, TIM_Channel_3, 0, &(TIM8->CCR3), GPIO_AF_TIM8 }},
		
		
	{ false, GPIOB, GPIO_Pin_9, GPIO_PinSource9, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, 
		{true, TIM11, TIM_Channel_1, 0, &(TIM11->CCR1), GPIO_AF_TIM11 } }, //
	{ false, GPIOB, GPIO_Pin_8, GPIO_PinSource8, GPIO_Mode_AF,GPIO_OType_PP, GPIO_PuPd_UP, 
		{true, TIM10, TIM_Channel_1, 0, &(TIM10->CCR1), GPIO_AF_TIM10 } }, 
		
		
	{ false, },
	{ false, }
};

PinDef GPIOPins[MAX_GPIO_PINS] = {
	{ true, GPIOC, GPIO_Pin_13, GPIO_PinSource13, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,}, // Beeper
	{ false, GPIOC, GPIO_Pin_10, GPIO_PinSource10, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP,}, // Armed
	{ false, GPIOC, GPIO_Pin_9, GPIO_PinSource9, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP,}, // Landing
	{ false, GPIOA, GPIO_Pin_5, GPIO_PinSource5, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,}, // Aux1
	{ false, GPIOA, GPIO_Pin_15, GPIO_PinSource15, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,}, // Aux2
	{ false, GPIOC, GPIO_Pin_8, GPIO_PinSource8, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,}, // Probe (Aux3)
	{ false, GPIOC, GPIO_Pin_14, GPIO_PinSource14, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP,}, // MPU6XXXIntSel 
	{ false, GPIOC, GPIO_Pin_15, GPIO_PinSource15, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP,} // HMC5XXXRdySel
};

PinDef WSPin = // Aux1 Pin for UAVXF4V3
	{false, GPIOA, GPIO_Pin_9, GPIO_PinSource9, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL, 
		{true, TIM1, TIM_Channel_2, TIM_DMA_CC2, &(TIM1->CCR2), GPIO_AF_TIM1 },
		{true, DMA_Channel_0, DMA2_Stream2}, DMA2_Stream2_IRQn }; // ???

PinDef LEDPins[MAX_LED_PINS] = {
	{ true, GPIOB, GPIO_Pin_2, GPIO_PinSource2, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL,}, // 
	{ false, },
	{ false, },
	{ false, }
};

const AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = {			
	{ false, }, // RF
	{ true, ADC1, GPIOC, GPIO_Pin_1, ADC_Channel_11, {true, DMA_Channel_0, DMA2_Stream0}, 2 }, // Current
	{ true, ADC1, GPIOC, GPIO_Pin_0, ADC_Channel_12, {true, DMA_Channel_0, DMA2_Stream0}, 3 }, // Volts

	{ false, 0, }, // Roll
	{ false, 0, }, // Pitch
	{ false, 0, } // Yaw
};

SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { // Tx, Rx
	{false, }, // mapped to USBSerial
	{true, USART1, //  
		GPIO_AF_USART1, GPIOB, GPIO_Pin_6, GPIO_PinSource6, GPIO_Pin_7, GPIO_PinSource7, 
		true, USART1_IRQn,
		false, DMA_Channel_4, DMA2_Stream7, DMA2_Stream7_IRQn, DMA2_Stream5, 115200 }, //
	{true, USART2, // 
		GPIO_AF_USART2, GPIOA, GPIO_Pin_2, GPIO_PinSource2, GPIO_Pin_3, GPIO_PinSource3, 
		true, USART2_IRQn, 
		false, DMA_Channel_4, DMA1_Stream6, DMA1_Stream6_IRQn, DMA1_Stream5, 9600 }, //
	{true, USART3, // 
		GPIO_AF_USART3, GPIOB, GPIO_Pin_10, GPIO_PinSource10, GPIO_Pin_11, GPIO_PinSource11, 
		true, USART3_IRQn, 
		false, DMA_Channel_4, DMA1_Stream3, DMA1_Stream3_IRQn, DMA1_Stream1, 115200 }, //
	{true, UART4, 
		GPIO_AF_UART4, GPIOA, GPIO_Pin_0, GPIO_PinSource0, GPIO_Pin_1, GPIO_PinSource1, 
		true, UART4_IRQn, 
		false, DMA_Channel_4, DMA1_Stream3, DMA1_Stream3_IRQn, DMA1_Stream1, 115200 }, //
	{false, 0 } // Soft Serial
};


PinDef USBDisconnectPin = { false, 0, };

const DevDef busDev[] = {
	{ true, mpu6000IMU, useSPI, 1, xxx}, // imu0Sel
	{ false, },// imu1Sel 
	{ true, ms5611Baro, useSPI, 2, }, // baroSel
	{ true, hmc5xxxMag, useI2C, 1, HMC5XXX_ID}, // magSel 
	{ false, spiFlashMem, useSPI, 2,}, // memSel 
	{ false, }, // gpsSel
	{ false, }, // rfSel
	{ false, }, // asSel,
	{ false, }, // flowSel
	{ false, }, // escSel
};

const uint8 currGimbalType = noGimbal;
const boolean ledsLowOn = true;
const uint8 imuSel = imu0Sel;

void InitTarget(void) {
			
	TelemetrySerial = Usart3;
	InitSerialPort(TelemetrySerial, true, false);

	CurrRxType = FutabaSBusRx;
	
	switch (CurrRxType) {
	case CPPMRx:
		CurrMaxPWMOutputs = 4;
		CurrNoOfRCPins = 1;
		RxUsingSerial = false;
		break;
	default:
		CurrMaxPWMOutputs = 4;
		CurrNoOfRCPins = 0;
		RxUsingSerial = true;
		RCSerial = Usart2;
		InitSerialPort(RCSerial, false, CurrRxType == FutabaSBusRx);
		break;
	} // switch
	
	GPSRxSerial = GPSTxSerial = Uart4;
	InitSerialPort(GPSRxSerial, false, false); 

	if (CurrwsNoOfLeds > 0) 
		CurrMaxPWMOutputs = 4;

} // omnibusf4nxtInit
#endif

