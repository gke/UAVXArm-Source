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

#define MAX_SPI_DEVICES 0

PinDef RCPins[MAX_RC_INPUTS] = { { true, GPIOA, GPIO_Pin_0, GPIO_PinSource0,
		GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP, true, { TIM2, TIM_Channel_1,
				TIM_IT_CC1, 0, GPIO_AF_TIM2 }, TIM2_IRQn }, //
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, },//
		{ false, 0, }, //
		};

AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = { //
		{ true, ADC1, GPIOC, GPIO_Pin_1, ADC_Channel_11, DMA_Channel_0,
				DMA2_Stream0, 2 }, // Rangefinder
				{ false, 0, }, // Amps
				{ true, ADC1, GPIOC, GPIO_Pin_2, ADC_Channel_12, DMA_Channel_0,
						DMA2_Stream0, 3 }, // Volts
				{ false, 0, }, // Roll
				{ false, 0, }, // Pitch
				{ false, 0, }, // Yaw
		};

// ESC PWM1 TIM4_CH4 PB9
// ESC PWM2 TIM4_CH3 PB8
// ESC PWM3 TIM4_CH2 PB7
// ESC PWM4 TIM4_CH1 PB6
// ESC PWM5 TIM8_CH1 PC6
// ESC PWM6 TIM8_CH2 PC7

PinDef
		PWMPins[MAX_PWM_OUTPUTS] = { //
						{ true, GPIOB, GPIO_Pin_9, GPIO_PinSource0,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM4, TIM_Channel_1, 0, &(TIM3->CCR1),
										GPIO_AF_TIM4 } }, //
						{ true, GPIOB, GPIO_Pin_8, GPIO_PinSource1,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM4, TIM_Channel_2, 0, &(TIM4->CCR2),
										GPIO_AF_TIM4 } },//
						{ true, GPIOB, GPIO_Pin_7, GPIO_PinSource3,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM4, TIM_Channel_3, 0, &(TIM4->CCR3),
										GPIO_AF_TIM4 } }, //
						{ true, GPIOB, GPIO_Pin_6, GPIO_PinSource2,
								GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_UP,
								true, { TIM4, TIM_Channel_4, 0, &(TIM4->CCR4),
										GPIO_AF_TIM4 } }, { false, 0 }, //
						{ false, 0 }, //
						{ false, 0 }, //
						{ false, 0 }, //
						{ false, 0 }, //
						{ false, 0 }, //
						{ false, 0 } //
				};

PinDef GPIOPins[MAX_GPIO_PINS] = { //
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

PinDef SPISelectPins[MAX_SPI_DEVICES] = {

};

PinDef LEDPins[MAX_LED_PINS] = { // LEDYellowSel, LEDRedSel, LEDBlueSel, LEDGreenSel
		{ true, GPIOD, GPIO_Pin_12, GPIO_PinSource12, GPIO_Mode_OUT,
				GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, }, //
				{ true, GPIOD, GPIO_Pin_13, GPIO_PinSource13, GPIO_Mode_OUT,
						GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, }, //
				{ true, GPIOD, GPIO_Pin_14, GPIO_PinSource14, GPIO_Mode_OUT,
						GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, }, //
				{ true, GPIOD, GPIO_Pin_15, GPIO_PinSource15, GPIO_Mode_OUT,
						GPIO_OType_PP, GPIO_PuPd_NOPULL, false, { 0, }, 0, } };

#define INCLUDE_USB

#define USE_USB_OTG_FS
#define USB_OTG_FS_CORE
#define USE_EMBEDDED_PHY
#define VBUS_SENSING_ENABLED
#define USE_DEVICE_MODE

#define USB_DETECT_PIN //  you need to be able to force Windows to release USB
#define USB_DISCONNECT_GPIO    GPIOA
#define USB_DISCONNECT_PIN     GPIO_Pin_12

// commonly PA8 or PC5
#define VBUS_GPIO		GPIOC
#define VBUS_PIN		GPIO_Pin_5

//#define INCLUDE_USB

#define USE_USB_OTG_FS
#define USB_OTG_FS_CORE
#define USE_EMBEDDED_PHY
#define VBUS_SENSING_ENABLED
#define USE_DEVICE_MODE

#define USB_DETECT_PIN //  you need to be able to force Windows to release USB
#define USB_DISCONNECT_GPIO    GPIOA
#define USB_DISCONNECT_PIN     GPIO_Pin_12

// commonly PA8 or PC5
#define VBUS_GPIO		GPIOC
#define VBUS_PIN		GPIO_Pin_5

#define LED_ON_HIGH

uint8 GPSRxSerial, GPSTxSerial, RCSerial, TelemetrySerial;
boolean RxUsingSerial;

// imuSel, baroSel, magSel, memSel, gpsSel, rfSel, escSel, flowSel, assel
const uint8 spiMap[] = { 1, 3, 2, 2, 2, 2, 2, 2, 2 };
const uint8 i2cMap[] = { 2, 2, 2, 2, 2, 2, 2, 2, 2 };

boolean spiDevUsed[] = { false, false, false, false, false, false, false,
		false, false };

const uint8 currIMUType = mpu6050IMU;
const uint8 currBaroType = ms5611Baro;
const uint8 currMagType = hmc5xxxMag;
const uint8 currGimbalType = noGimbal;

void InitTarget(void) {

	TelemetrySerial = Uart4; //USBSerial;

	if (TelemetrySerial == USBSerial)
		USBConnect();

	InitSerialPort(TelemetrySerial, true, false);

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

