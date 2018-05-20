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

#ifndef _pinsomnibusf4_h
#define _pinsomnibusf4_h

#include "UAVX.h"

const uint8 currIMUType = mpu6000IMU;
const uint8 currBaroType = noBaro;
const uint8 currMagType = noMag;
const uint8 currGimbalType = noGimbal;

// imuSel, baroSel, magSel, memSel, gpsSel, rfSel, escSel, flowSel, assel
const uint8 spiMap[] = {1, 3, 2, 2, 2, 2, 2, 2, 2};
const uint8 i2cMap[] = {2, 2, 2, 2, 2, 2, 2, 2, 2};

boolean spiDevUsed[] = {true, true, false, false, false, false, false, false, false};

PinDef RCPins[MAX_RC_INPS] = {
	//{ GPIOB, GPIO_Pin_14, GPIO_PinSource14, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
	//		true, {TIM8, TIM_Channel_2, TIM_IT_CC2, 0, GPIO_AF_TIM8 }, TIM8_BRK_TIM12_IRQn }
};

I2CPortDef I2CPorts[MAX_I2C_PORTS] = {
	{ false, I2C1, 1,
			GPIOB, GPIO_Pin_6, GPIO_PinSource6,
			GPIOB, GPIO_Pin_7 , GPIO_PinSource7,
			GPIO_AF_I2C1 },
	{ false, I2C2, 2,
			GPIOB, GPIO_Pin_10, GPIO_PinSource10,
			GPIOB, GPIO_Pin_11, GPIO_PinSource11,
			GPIO_AF_I2C2 }
};

AnalogPinDef AnalogPins[ANALOG_CHANNELS] = {
	{ true, ADC1, GPIOC, GPIO_Pin_1, ADC_Channel_11, DMA_Channel_0, DMA2_Stream0, 2}, // Amps
	{ true, ADC1, GPIOC, GPIO_Pin_2, ADC_Channel_12, DMA_Channel_0, DMA2_Stream0, 3} // Volts
};

PinDef PWMPins[MAX_PWM_OUTPUTS] = {
	{ GPIOB, GPIO_Pin_0, GPIO_PinSource0, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		true, {TIM3, TIM_Channel_3, 0, &(TIM3->CCR3), GPIO_AF_TIM3}},
	{ GPIOB, GPIO_Pin_1, GPIO_PinSource1, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		true, {TIM3, TIM_Channel_4, 0, &(TIM3->CCR4), GPIO_AF_TIM3}},
	{ GPIOA, GPIO_Pin_3, GPIO_PinSource3, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		true, {TIM9, TIM_Channel_2, 0, &(TIM9->CCR2), GPIO_AF_TIM9}},
	{ GPIOA, GPIO_Pin_2, GPIO_PinSource2, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		true, {TIM2, TIM_Channel_3, 0, &(TIM2->CCR3), GPIO_AF_TIM2}},

	{ GPIOA, GPIO_Pin_1, GPIO_PinSource1, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		true, {TIM5, TIM_Channel_2, 0, &(TIM5->CCR2), GPIO_AF_TIM5}},
#if defined(IS_PRO)
	{ GPIOB, GPIO_Pin_6, GPIO_PinSource6, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		true, {TIM4, TIM_Channel_1, 0, &(TIM5->CCR1), GPIO_AF_TIM4}},
#endif
	{ GPIOA, GPIO_Pin_8, GPIO_PinSource8, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		true, {TIM1, TIM_Channel_1, 0, &(TIM1->CCR1), GPIO_AF_TIM1}}




		/*
    { TIM3,  IO_TAG(PB4),  TIM_Channel_1, 0, IOCFG_AF_PP, GPIO_AF_2,  TIM_USE_PPM }, // Pin PPM - PB4
    // PB5 / TIM3 CH2 is connected to USBPresent

*/
};


PinDef GPIOPins[MAX_GPIO_PINS] = {
		{ GPIOB, GPIO_Pin_4, GPIO_PinSource4, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,
			false, { 0, }, 0, }, // Beeper
		{ GPIOC, GPIO_Pin_0, GPIO_PinSource0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,
			false, { 0, }, 0,}, // Inverter
		{ GPIOC, GPIO_Pin_4, GPIO_PinSource4, GPIO_Mode_IN, GPIO_OType_PP, GPIO_PuPd_UP,
			false, { 0, }, 0 }, // MPU6XXXIntSel
		{ GPIOB, GPIO_Pin_14, GPIO_PinSource14,  GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,
			false, { 0, }, 0, }, // Probe
	};

SPIPortDef SPIPorts[MAX_SPI_PORTS] = { // SCK, MISO, MOSI
	{ true, SPI1, GPIOA,  {{GPIO_Pin_5, GPIO_PinSource5},
			{GPIO_Pin_6, GPIO_PinSource6},
			{GPIO_Pin_7, GPIO_PinSource7}}},
	{ false, SPI2, GPIOB, {{GPIO_Pin_13, GPIO_PinSource13},
			{GPIO_Pin_14, GPIO_PinSource14},
			{GPIO_Pin_15, GPIO_PinSource15}}},
	{ false, SPI3, GPIOC, {{GPIO_Pin_10, GPIO_PinSource10},
			{GPIO_Pin_11, GPIO_PinSource11},
			{GPIO_Pin_12, GPIO_PinSource12}}}
};

PinDef SPISelectPins[MAX_SPI_DEVICES] = {
	// mpu60xxSel, ms56xxSel, hmc5xxxSel, memSel, gpsSel, rfSel, escSel, flowSel, asSel
	{ GPIOA, GPIO_Pin_4, GPIO_PinSource4, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,
		false, { 0, }, 0, }, // MPU6XXX
	//{ GPIOB, GPIO_Pin_3, GPIO_PinSource3, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_UP,
	//	false, { 0, }, 0, }, // MS56XX
};

SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { // Tx, Rx

		{ true, USART1, GPIO_AF_USART1, GPIOA,
	        GPIO_Pin_9, GPIO_PinSource9,
	        GPIO_Pin_10, GPIO_PinSource10,
	        true, USART1_IRQn,
	        false, DMA_Channel_4,
	        DMA2_Stream7, DMA2_Stream7_IRQn,
	        DMA2_Stream5,
	        115200
	        },
	    { false, USART2, GPIO_AF_USART2, GPIOA,
	        GPIO_Pin_2, GPIO_PinSource2,
	        GPIO_Pin_3, GPIO_PinSource3,
	        true, USART2_IRQn,
	        false, DMA_Channel_4,
	        DMA1_Stream6, DMA1_Stream6_IRQn,
	        DMA1_Stream5,
	        115200
	    	}
};

PinDef LEDPins[MAX_LEDS] = { // LEDYellowSel, LEDRedSel, LEDBlueSel, LEDGreenSel
	{ GPIOB, GPIO_Pin_5, GPIO_PinSource3, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL,
		false, { 0, }, 0, }
	};

#endif


