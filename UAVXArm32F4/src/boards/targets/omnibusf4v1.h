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

#define MAX_SPI_DEVICES 4

PinDef RCPins[MAX_RC_INPUTS] = { //
		//{ GPIOB, GPIO_Pin_14, GPIO_PinSource14, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
		//		true, {TIM8, TIM_Channel_2, TIM_IT_CC2, 0, GPIO_AF_TIM8 }, TIM8_BRK_TIM12_IRQn }
		{ false, 0, }, //
				{ false, 0, }, //
				{ false, 0, }, //
				{ false, 0, }, //
				{ false, 0, }, //
				{ false, 0, }, //
				{ false, 0, }, //
				{ false, 0, },
		};

AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = { //
		{ false, 0, }, // Rangefinder
				{ true, ADC1, GPIOC, GPIO_Pin_1, ADC_Channel_11, DMA_Channel_0,
						DMA2_Stream0, 2 }, // Amps
				{ true, ADC1, GPIOC, GPIO_Pin_2, ADC_Channel_12, DMA_Channel_0,
						DMA2_Stream0, 3 }, // Volts
				{ false, 0, }, // Roll
				{ false, 0, }, // Pitch
				{ false, 0, } // Yaw
		};

PinDef
		PWMPins[MAX_PWM_OUTPUTS] =
				{
						{ true, GPIOB, GPIO_Pin_0, GPIO_PinSource0,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM3, TIM_Channel_3, 0, &(TIM3->CCR3),
										GPIO_AF_TIM3 } },
						{ true, GPIOB, GPIO_Pin_1, GPIO_PinSource1,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM3, TIM_Channel_4, 0, &(TIM3->CCR4),
										GPIO_AF_TIM3 } },
						{ true, GPIOA, GPIO_Pin_3, GPIO_PinSource3,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM9, TIM_Channel_2, 0, &(TIM9->CCR2),
										GPIO_AF_TIM9 } },
						{ true, GPIOA, GPIO_Pin_2, GPIO_PinSource2,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM2, TIM_Channel_3, 0, &(TIM2->CCR3),
										GPIO_AF_TIM2 } },

						{ true, GPIOA, GPIO_Pin_1, GPIO_PinSource1,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM5, TIM_Channel_2, 0, &(TIM5->CCR2),
										GPIO_AF_TIM5 } },
#if defined(IS_PRO)
						{	true, GPIOB, GPIO_Pin_6, GPIO_PinSource6, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
							true, {TIM4, TIM_Channel_1, 0, &(TIM5->CCR1), GPIO_AF_TIM4}},

#else
						{ false, 0 },
#endif
						{ true, GPIOA, GPIO_Pin_8, GPIO_PinSource8,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM1, TIM_Channel_1, 0, &(TIM1->CCR1),
										GPIO_AF_TIM1 } }

				};

PinDef GPIOPins[MAX_GPIO_PINS] = { //
		{ true, GPIOB, GPIO_Pin_4, GPIO_PinSource4, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_UP, false, { 0, }, 0, }, // Beeper
				{ false, 0, }, // Landing
				{ false, 0, }, // Aux1/WS2812 (WS2812 hard coded)
				{ false, 0, }, // Aux2/SoftUSARTTx
				{ true, GPIOB, GPIO_Pin_14, GPIO_PinSource14, GPIO_Mode_OUT,
						GPIO_OType_PP, GPIO_PuPd_UP, false, { 0, }, 0, }, // Probe
				{ true, GPIOC, GPIO_Pin_0, GPIO_PinSource0, GPIO_Mode_OUT,
						GPIO_OType_PP, GPIO_PuPd_UP, false, { 0, }, 0, }, // Inverter
				{ true, GPIOC, GPIO_Pin_4, GPIO_PinSource4, GPIO_Mode_IN,
						GPIO_OType_PP, GPIO_PuPd_UP, false, { 0, }, 0 }, // MPU6XXXIntSel
				{ false, 0, }, // HMC5XXXRdySel
		};

PinDef LEDPins[MAX_LED_PINS] = { //
		{ true, GPIOB, GPIO_Pin_5, GPIO_PinSource3, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, }, // Yellow
				{ false, 0 }, // Red
				{ false, 0 }, // Blue
				{ false, 0 }, // Green

		};

PinDef SPISelectPins[MAX_SPI_DEVICES] = { { false, }, //
		{ false, }, //
		{ false, }, //
		{ false, }, //

		};

uint8 GPSRxSerial, GPSTxSerial, RCSerial, TelemetrySerial;
boolean RxUsingSerial;

const uint8 currIMUType = mpu6000IMU;
const uint8 currBaroType = noBaro;
const uint8 currMagType = noMag;
const uint8 currGimbalType = noGimbal;

// imuSel, baroSel, magSel, memSel, gpsSel, rfSel, escSel, flowSel, assel
const uint8 spiMap[] = { 1, 3, 2, 2, 2, 2, 2, 2, 2 };
const uint8 i2cMap[] = { 2, 2, 2, 2, 2, 2, 2, 2, 2 };

boolean spiDevUsed[] = { true, true, false, false, false, false, false, false,
		false };

void InitTarget(void) {

	TelemetrySerial = USBSerial;

	if (TelemetrySerial == USBSerial)
		USBConnect();

	RxUsingSerial = true;
	CurrNoOfRCPins = 0;

	RCSerial = Usart2;
	InitSerialPort(RCSerial, false, true);

	//GPSRxSerial = GPSTxSerial = Usart3;
	//InitSerialPort(GPSRxSerial, false, false);

	CurrMaxPWMOutputs = 4;

} // InitTarget

#endif

