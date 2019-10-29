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

const PinDef RCPins[MAX_RC_INPUTS] = {
//
		{ true, GPIOA, GPIO_Pin_0, GPIO_PinSource0, GPIO_Mode_AF,
				GPIO_OType_PP, GPIO_PuPd_UP, true, { TIM2, TIM_Channel_1,
						TIM_IT_CC1, 0, GPIO_AF_TIM2 }, TIM2_IRQn }, //
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, }, //
		};

// ESC PWM1 TIM4_CH4 PB9
// ESC PWM2 TIM4_CH3 PB8
// ESC PWM3 TIM4_CH2 PB7
// ESC PWM4 TIM4_CH1 PB6
// ESC PWM5 TIM8_CH1 PC6
// ESC PWM6 TIM8_CH2 PC7

const PinDef PWMPins[MAX_PWM_OUTPUTS] = {
//
		{ true, GPIOB, GPIO_Pin_9, GPIO_PinSource0, GPIO_Mode_AF,
				GPIO_OType_PP, GPIO_PuPd_UP, true, { TIM4, TIM_Channel_1, 0,
						&(TIM3->CCR1), GPIO_AF_TIM4 } }, //
		{ true, GPIOB, GPIO_Pin_8, GPIO_PinSource1, GPIO_Mode_AF,
				GPIO_OType_PP, GPIO_PuPd_UP, true, { TIM4, TIM_Channel_2, 0,
						&(TIM4->CCR2), GPIO_AF_TIM4 } },//
		{ true, GPIOB, GPIO_Pin_7, GPIO_PinSource3, GPIO_Mode_AF,
				GPIO_OType_PP, GPIO_PuPd_UP, true, { TIM4, TIM_Channel_3, 0,
						&(TIM4->CCR3), GPIO_AF_TIM4 } }, //
		{ true, GPIOB, GPIO_Pin_6, GPIO_PinSource2, GPIO_Mode_AF,
				GPIO_OType_PP, GPIO_PuPd_UP, true, { TIM4, TIM_Channel_4, 0,
						&(TIM4->CCR4), GPIO_AF_TIM4 } },//
		{ false, 0 }, //
		{ false, 0 }, //
		{ false, 0 }, //
		{ false, 0 }, //
		{ false, 0 }, //
		{ false, 0 } //
};

const PinDef GPIOPins[MAX_GPIO_PINS] = {
//
		{ true, GPIOA, GPIO_Pin_15, GPIO_PinSource15, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_UP, false, { 0, }, 0, }, // Beeper
		{ false, 0, }, // Landing
		{ false, 0, }, // Aux1/WS2812 (WS2812 hard coded)
		{ false, 0, }, // Aux2/SoftUSARTTx
		{ true, GPIOB, GPIO_Pin_4, GPIO_PinSource4, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_UP, false, { 0, }, 0, }, // Aux3/Probe
		{ false, 0, }, // Aux4/Inverter
		{ false, 0, }, // MPU6XXXIntSel
		{ false, 0, }, // HMC5XXXRdySel
		};

const PinDef LEDPins[MAX_LED_PINS] = {
// LEDYellowSel, LEDRedSel, LEDBlueSel, LEDGreenSel
		{ true, GPIOD, GPIO_Pin_13, GPIO_PinSource13, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, }, // Yellow
		{ true, GPIOD, GPIO_Pin_14, GPIO_PinSource14, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, }, // Red
		{ true, GPIOD, GPIO_Pin_15, GPIO_PinSource15, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, }, // Blue
		{ true, GPIOD, GPIO_Pin_12, GPIO_PinSource12, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, }, // Green
		};

const AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = {
//
		{ true, ADC1, GPIOC, GPIO_Pin_1, ADC_Channel_11, {DMA_Channel_0,
				DMA2_Stream0}, 1 }, // Rangefinder
		{ false, ADC1, GPIOC, GPIO_Pin_0, ADC_Channel_10, {DMA_Channel_0,
				DMA2_Stream0}, 2 }, // Amps
		{ true, ADC1, GPIOC, GPIO_Pin_2, ADC_Channel_12, {DMA_Channel_0,
				DMA2_Stream0}, 3 }, // Volts
		{ false, 0, }, // Roll
		{ false, 0, }, // Pitch
		{ false, 0, }, // Yaw
		};

	
SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { // Tx, Rx
	{false, }, // mapped to USBSerial
	{true, USART1, PA9, PA10, 115200, true, false, USART1Config},
	{true, USART2, PA2, PA3, 9600, true, false, USART2Config},
	{true, USART3, PD8, PD9, 115200, true, false, USART3Config},
	{false, UART4, PC10, PC11, 115200, true, false, UART4Config},
	{false, 0 } // SoftSerial
};

const PinDef USBDisconnectPin = { true, GPIOD, GPIO_Pin_11, GPIO_PinSource11,
		GPIO_Mode_OUT, GPIO_OType_OD, GPIO_PuPd_NOPULL, false, { 0, }, 0, };


const DevDef busDev[] = {
// imu0Sel, imu1Sel, baroSel, magSel, memSel, gpsSel, rfSel, asSel, flowSel, escSel,
		{ true, mpu6050IMU, useI2C, 2, MPU_0x68_ID },// imu0Sel
		{ false, },// imu1Sel 
		{ true, ms5611Baro, useI2C, 2, MS56XX_ID },// baroSel
		{ true, hmc5xxxMag, useI2C, 2, HMC5XXX_ID },// magSel 
		{ false, noMem, useSPI, 0 },// memSel 
		{ false, noAS, useI2C, 0, MS4525_ID },// gpsSel
		{ false, noRF, useI2C, 0, MAXSONAR_ID },// rfSel
		{ false, noAS },// asSel,
		{ false, },// flowSel, 
		{ false, }, // escSel,
		};

const boolean ledsLowOn = true;
const uint8 currGimbalType = noGimbal;

void InitTarget(void) {

	imuSel = imu0Sel;

	TelemetrySerial = USBSerial; // Uart4; //
	InitSerialPort(TelemetrySerial, true, false);
	while(true) { Delay1mS(200); TxChar(TelemetrySerial, '?'); }

	CurrNoOfRCPins = 0;
	RxUsingSerial = true;

	if (RxUsingSerial) {
		RCSerial = Usart2;
		InitSerialPort(RCSerial, false, true);
	}

	GPSRxSerial = GPSTxSerial = Usart3;
	InitSerialPort(GPSRxSerial, false, false);

	CurrMaxPWMOutputs = 4;

} // InitTarget

#endif

