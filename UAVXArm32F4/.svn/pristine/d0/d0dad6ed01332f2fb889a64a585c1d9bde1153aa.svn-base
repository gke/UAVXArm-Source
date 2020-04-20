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

PinDef RCPins[MAX_RC_INPUTS] = { };

PinDef PWMPins[MAX_PWM_OUTPUTS] = {
	{true, PB0, PWMPinConfig, PWMOut_3_3},
	{true, PB1, PWMPinConfig, PWMOut_3_4},
	{true, PA3, PWMPinConfig, PWMOut_2_4},
	{true, PA2, PWMPinConfig, PWMOut_2_3},	 
	{true, PA1, PWMPinConfig, PWMOut_5_2},
	{true, PA8, PWMPinConfig, PWMOut_1_1},
};

PinDef GPIOPins[MAX_GPIO_PINS] = { //
	{true, PC15, OutPinConfig,}, // Beeper
	{false, }, // Armed
	{false,}, // Landing
	{false, }, // Aux1
	{true, PA15, OutPinConfig,}, // Aux2/Inverter
	{true, PB14, OutPinConfig,}, // Probe (Aux3)
	{true, PC13, InpPinConfig,}, // MPU6XXXIntSel 
	{false, } // HMC5XXXRdySel					
};

PinDef LEDPins[MAX_LED_PINS] = { //
	{true, PB3, LEDPinConfig,}, 
};

//PinDef WSPin = 
//	{true, PB6, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL, 
//		{true, TIM4, TIM_Channel_1, TIM_DMA_CC1, &(TIM4->CCR1), GPIO_AF_TIM4 },
//		{true, DMA_Channel_0, DMA2_Stream2}, DMA2_Stream2_IRQn };

SerialPortDef SerialPorts[MAX_SERIAL_PORTS] = { 
	{false, }, // mapped to USBSerial
	{true, USART1, PA9, PA10, 115200, USART1Config,},
	{true, USART2, PA14, PA15, 9600, USART2Config,},
	{true, USART3, PB10, PB11, 115200, USART3Config,},
	{false, UART4, PB10, PB11, 115200, UART4Config,},
	{false, 0 } // SoftSerial
};

AnalogPinDef AnalogPins[MAX_ANALOG_CHANNELS] = { //
	{false,  }, 
	{true, ADC1, PA1, ADC_Channel_11, DMA2_0_0_noIRQn, 3 }, // Current
	{true, ADC1, PA0, ADC_Channel_12, DMA2_0_0_noIRQn, 2 }, // Volts
};

PinDef USBDisconnectPin = {true, PA12, OutPinConfig,};
 
DevDef busDev[maxDevSel] = { 	 	
	{true, imu0Sel, mpu6000IMU, true, 1, 0, {true, PA4, OutPinConfig,}}, 
	{false, imu1Sel, }, 
	{false, baroSel, ms5611Baro, false, 2, MS56XX_ID, {}},
	{false, magSel, hmc5xxxMag, false, 2, HMC5XXX_ID, {}},
	{false, memSel, }, 
	{false, gpsSel, },
	{false, rfSel, },
	{false, asSel, },
	{false, flowSel, }, 
	{false, escSel, }
};

const uint8 currGimbalType = noGimbal;
const boolean ledsLowOn = false;
const uint8 imuSel = imu0Sel;

void InitTarget(void) {
	
	TelemetrySerial = Usart1; //USBSerial;
	InitSerialPort(TelemetrySerial, true, false);

	RxUsingSerial = true;
	CurrNoOfRCPins = 0; // Always 0

	RCSerial = Usart2;
	InitSerialPort(RCSerial, false, true);

	//GPSRxSerial = GPSTxSerial = Usart3;
	//InitSerialPort(GPSRxSerial, false, false);

	CurrMaxPWMOutputs = 4;

} // InitTarget

#endif

