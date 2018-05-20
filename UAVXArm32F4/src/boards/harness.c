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

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without
//    even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
//    See the GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/


#include "UAVX.h"

__attribute__((always_inline))                   inline boolean digitalRead(PinDef * d) {
	return GPIO_ReadInputDataBit(d->Port, d->Pin);
} // digitalRead

__attribute__((always_inline)) inline void digitalWrite(PinDef * d, uint8 m) {

	if (m)
		d->Port->BSRRL = d->Pin;
	else
		d->Port->BSRRH = d->Pin;

} // digitalWrite

__attribute__((always_inline)) inline void digitalToggle(PinDef * d) {
	d->Port->ODR ^= d->Pin;
} // digitalToggle


void pinInit(PinDef * d) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = d->Pin;
	GPIO_InitStructure.GPIO_Mode = d->Mode;
	GPIO_InitStructure.GPIO_OType = d->OType;
	GPIO_InitStructure.GPIO_PuPd = d->PuPd;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(d->Port, &GPIO_InitStructure);
} // pinInit


void pinInitMode(PinDef * d, boolean IsInput) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = d->Pin;

	if (IsInput)
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	else
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(d->Port, &GPIO_InitStructure);

} // pinInitMode


void pinInitOutput(PinDef * d) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = d->Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = d->OType;
	GPIO_InitStructure.GPIO_PuPd = d->PuPd;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(d->Port, &GPIO_InitStructure);
} // pinInitOutput

void NVICDisable(IRQn_Type ISR) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = ISR;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

} // NVICDisable


TIM_ICInitTypeDef TIM_ICInitStructure = { 0, }; // global for rc

void InitRCPins(uint8 PPMInputs) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = { 0, };
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	uint8 i;
	PinDef * u;

	for (i = 0; i < PPMInputs; i++) {
		TIM_CtrlPWMOutputs(RCPins[i].Timer.Tim, DISABLE);
		TIM_DeInit(RCPins[i].Timer.Tim);
	}

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_ICStructInit(&TIM_ICInitStructure);

	// Common initialisation

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // JIH
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // TODO: 32 bit period?
	TIM_TimeBaseStructure.TIM_Prescaler = (TIMER_PS >> 1) - 1;

	// Pin specific
	for (i = 0; i < PPMInputs; i++) {
		u = &RCPins[i];

		pinInit(u);

		GPIO_PinAFConfig(u->Port, u->PinSource, u->Timer.TimAF);

		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStructure.NVIC_IRQChannel = u->PinISR;
		NVIC_Init(&NVIC_InitStructure);

		TIM_TimeBaseInit(u->Timer.Tim, &TIM_TimeBaseStructure);

		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = u->Timer.Channel;
		TIM_ICInit(u->Timer.Tim, &TIM_ICInitStructure);

		//u->Timer.Tim->DIER |= u->Timer.CC; // TIM_ITConfig ENABLE Channel
		TIM_ITConfig(u->Timer.Tim, u->Timer.CC, ENABLE);

	}
	for (i = 0; i < PPMInputs; i++)
		TIM_Cmd(RCPins[i].Timer.Tim, ENABLE);

} // InitRCPins


void i2cInit(uint8 i2cCurr) {
	// Original source unknown but based on those in baseflight by TimeCop
	NVIC_InitTypeDef NVIC_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	I2CPortDef * d;

	d = &I2CPorts[i2cCurr];

	if (d->Used) {

		GPIO_PinAFConfig(d->SCLPort, d->SCLPinSource, d->I2C_AF);
		GPIO_PinAFConfig(d->SDAPort, d->SDAPinSource, d->I2C_AF);

		i2cUnstick(i2cCurr); // attempt to unfreeze slave(s) - initialises pins
		if (F.i2cFatal)
			return;

		I2C_DeInit(d->I2C);
		I2C_StructInit(&I2C_InitStructure);

		I2C_ITConfig(d->I2C, I2C_IT_EVT | I2C_IT_ERR, DISABLE); //Enable EVT and ERR interrupts - they are enabled by the first request
		I2C_InitStructure.I2C_ClockSpeed = I2C_CLOCK_HZ;
		I2C_Cmd(d->I2C, ENABLE);
		I2C_Init(d->I2C, &I2C_InitStructure);

		NVIC_PriorityGroupConfig(0x500);

		// I2C ER Interrupt
		NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		// I2C EV Interrupt
		NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_Init(&NVIC_InitStructure);
	}
} // i2cInit

void i2cUnstick(uint8 i2cCurr) {
	// Original source unknown but based on those in baseflight by TimeCop
#define I2C_DELAY_US 10
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8 i;
	uint16 Timeout;
	I2CPortDef * d;

	d = &I2CPorts[i2cCurr];

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = d->SCLPin;
	GPIO_Init(d->SCLPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = d->SDAPin;
	GPIO_Init(d->SDAPort, &GPIO_InitStructure);

	GPIO_SetBits(d->SCLPort, d->SCLPin);
	GPIO_SetBits(d->SDAPort, d->SDAPin);

	Timeout = mSClock() + 100;
	for (i = 0; i < 8; i++) { // Wait for any clock stretching to finish
		while (!GPIO_ReadInputDataBit(d->SCLPort, d->SCLPin)) {
			Delay1uS(I2C_DELAY_US);
			if (mSClock() > Timeout) {
				F.i2cFatal = true;
				return;
			}
		}

		GPIO_ResetBits(d->SCLPort, d->SCLPin);
		Delay1uS(I2C_DELAY_US);

		GPIO_SetBits(d->SCLPort, d->SCLPin);
		Delay1uS(I2C_DELAY_US);
	}

	GPIO_ResetBits(d->SDAPort, d->SDAPin);
	Delay1uS(I2C_DELAY_US);
	GPIO_ResetBits(d->SCLPort, d->SCLPin);
	Delay1uS(I2C_DELAY_US);
	GPIO_SetBits(d->SCLPort, d->SCLPin);
	Delay1uS(I2C_DELAY_US);
	GPIO_SetBits(d->SDAPort, d->SDAPin);

	GPIO_InitStructure.GPIO_Pin = d->SCLPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(d->SCLPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = d->SDAPin;
	GPIO_Init(d->SDAPort, &GPIO_InitStructure);

} // i2cUnstick

enum {
	sckPin, misoPin, mosiPin
};

void spiInitGPIOPins(uint8 spiPort, boolean highClock) {
	GPIO_InitTypeDef GPIO_InitStructure;
	SPIPortDef * p;

	p = &SPIPorts[spiPort];

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = p->P[sckPin].Pin | p->P[mosiPin].Pin
			| p->P[misoPin].Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	if (highClock)
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	else
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(p->Port, &GPIO_InitStructure);

} // spiInitGPIOPins

void spiInit(uint8 spiPort) {
	volatile uint8 dummyread __attribute__((unused));
	enum {
		CS, SCK, MISO, MOSI
	};

	SPI_InitTypeDef SPI_InitStructure;
	SPIPortDef * p;

	p = &SPIPorts[spiPort];

	if (p->Used) {

		spiInitGPIOPins(spiPort, false);

		switch (spiPort) {
		case 0:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
			GPIO_PinAFConfig(p->Port, p->P[sckPin].PinSource, GPIO_AF_SPI1);
			GPIO_PinAFConfig(p->Port, p->P[misoPin].PinSource, GPIO_AF_SPI1);
			GPIO_PinAFConfig(p->Port, p->P[mosiPin].PinSource, GPIO_AF_SPI1);
			break;
		case 1:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
			GPIO_PinAFConfig(p->Port, p->P[sckPin].PinSource, GPIO_AF_SPI2);
			GPIO_PinAFConfig(p->Port, p->P[misoPin].PinSource, GPIO_AF_SPI2);
			GPIO_PinAFConfig(p->Port, p->P[mosiPin].PinSource, GPIO_AF_SPI2);
			break;
		case 2:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
			GPIO_PinAFConfig(p->Port, p->P[sckPin].PinSource, GPIO_AF_SPI3);
			GPIO_PinAFConfig(p->Port, p->P[misoPin].PinSource, GPIO_AF_SPI3);
			GPIO_PinAFConfig(p->Port, p->P[mosiPin].PinSource, GPIO_AF_SPI3);
			break;
		}
		SPI_DeInit(p->SPIx);
		SPI_StructInit(&SPI_InitStructure);
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // MISO & MOSI
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;

		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;

		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; // 42/128 = 328.125 kHz SPI Clock
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;

		SPI_Init(p->SPIx, &SPI_InitStructure);

		SPI_CalculateCRC(p->SPIx, DISABLE);

		SPI_Cmd(p->SPIx, ENABLE);

		while (SPI_I2S_GetFlagStatus(p->SPIx, SPI_I2S_FLAG_TXE) == RESET) {
			// BLOCKING!
		};

		dummyread = SPI_I2S_ReceiveData(p->SPIx);
	}

} // spiInit


void spiDeInit(uint8 spiPort) {

	GPIO_InitTypeDef GPIO_InitStructure;
	SPIPortDef * p;

	p = &SPIPorts[spiPort];

	if (p->Used) {
		SPI_I2S_DeInit(p->SPIx);

		SPI_Cmd(p->SPIx, DISABLE);
		switch (spiPort) {
		case 0:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);
			break;
		case 1:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, DISABLE);
			break;
		case 2:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, DISABLE);
			break;
		}

		/* All SPI-Pins to input with weak internal pull-downs */
		GPIO_InitStructure.GPIO_Pin = p->P[sckPin].Pin | p->P[misoPin].Pin
				| p->P[mosiPin].Pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

		GPIO_Init(p->Port, &GPIO_InitStructure);

		spiSetDivisor(p->SPIx, 2); // 21 MHz SPI clock (within 20 +/- 10%)
	}

} // spiDeInit


void InitPWMPin(PinDef * u, uint16 pwmprescaler, uint32 pwmperiod,
		uint32 pwmwidth, boolean usingSync) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure = { 0, };
	uint16 preload;

	pinInit(u);

	if (u->TimerUsed) {
		//TIM_Cmd(u->Timer.Tim, DISABLE);
		//TIM_CtrlPWMOutputs(u->Timer.Tim, DISABLE);

		GPIO_PinAFConfig(u->Port, u->PinSource, u->Timer.TimAF);

		TIM_OCStructInit(&TIM_OCInitStructure);
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_Pulse = pwmwidth;

		preload = usingSync ? TIM_OCPreload_Disable : TIM_OCPreload_Enable;

		switch (u->Timer.Channel) {
		case TIM_Channel_1:
			TIM_OC1Init(u->Timer.Tim, &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(u->Timer.Tim, preload);
			break;
		case TIM_Channel_2:
			TIM_OC2Init(u->Timer.Tim, &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(u->Timer.Tim, preload);
			break;
		case TIM_Channel_3:
			TIM_OC3Init(u->Timer.Tim, &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(u->Timer.Tim, preload);
			break;
		case TIM_Channel_4:
			TIM_OC4Init(u->Timer.Tim, &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(u->Timer.Tim, preload);
			break;
		}

		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

		if ((u->Timer.Tim == TIM1 || u->Timer.Tim == TIM8 || u->Timer.Tim
				== TIM9 || u->Timer.Tim == TIM10 || u->Timer.Tim == TIM11))
			TIM_TimeBaseStructure.TIM_Prescaler = pwmprescaler - 1;
		else
			TIM_TimeBaseStructure.TIM_Prescaler = (pwmprescaler >> 1) - 1;

		TIM_TimeBaseStructure.TIM_Period = pwmperiod - 1;

		TIM_Cmd(u->Timer.Tim, ENABLE);
		TIM_TimeBaseInit(u->Timer.Tim, &TIM_TimeBaseStructure);

		TIM_CtrlPWMOutputs(u->Timer.Tim, ENABLE);

	}

} // InitPWMPin

void serialBaudRate(uint8 s, uint32 BaudRate) {
	USART_InitTypeDef USART_InitStructure;
	SerialPortDef * u;

	switch (s) {
	case SoftSerialTx:
		SoftUSARTBaudRate = BaudRate;
		break;
	case USBSerial:
		// zzz
		break;
	default:
		u = &SerialPorts[s];
		USART_Cmd(u->USART, DISABLE);
		USART_StructInit(&USART_InitStructure);
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_BaudRate = BaudRate;
		USART_Init(u->USART, &USART_InitStructure);
		USART_Cmd(u->USART, ENABLE);
		break;
	} // switch
} // serialBaudRate

void InitSerialPortxxx(uint8 s, boolean Enable, boolean SBusConfig) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	SerialPortDef * u;

	u = &SerialPorts[s];

	if (u->Used) {

		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = u->TxPin | u->RxPin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(u->Port, &GPIO_InitStructure);

		GPIO_PinAFConfig(u->Port, u->TxPinSource, u->USART_AF);
		GPIO_PinAFConfig(u->Port, u->RxPinSource, u->USART_AF);

		USART_StructInit(&USART_InitStructure);

		if (SBusConfig) {
			USART_InitStructure.USART_WordLength = USART_WordLength_9b;
			USART_InitStructure.USART_StopBits = USART_StopBits_2;
			USART_InitStructure.USART_Parity = USART_Parity_Even;
		} else
			USART_InitStructure.USART_Parity = USART_Parity_No;

		USART_InitStructure.USART_BaudRate = u->Baud;

		USART_Init(u->USART, &USART_InitStructure);

		if (u->InterruptsUsed) {

			RxQTail[s] = RxQHead[s] = TxQTail[s] = TxQHead[s] = 0;
			NVIC_InitStructure.NVIC_IRQChannel = u->ISR;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			USART_ITConfig(u->USART, USART_IT_RXNE, ENABLE);
			USART_ITConfig(u->USART, USART_IT_TXE, ENABLE);

		} else {

			TxQTail[s] = TxQHead[s] = 0;

			// Common
			DMA_StructInit(&DMA_InitStructure);
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)
					& u->USART->DR;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
			DMA_InitStructure.DMA_Channel = u->DMAChannel;
			DMA_InitStructure.DMA_BufferSize = SERIAL_BUFFER_SIZE;

			// Receive DMA
			DMA_DeInit(u->RxDMAStream);
			while (DMA_GetCmdStatus(u->RxDMAStream) != DISABLE) {
			};

			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) RxQ[s];
			DMA_Init(u->RxDMAStream, &DMA_InitStructure);
			RxDMAPos[s] = DMA_GetCurrDataCounter(DMA1_Stream1);
			DMA_Cmd(u->RxDMAStream, ENABLE);
			USART_DMACmd(u->USART, USART_DMAReq_Rx, ENABLE);

			// Transmit DMA
			NVIC_Init(&NVIC_InitStructure);
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannel = u->TxDMAISR;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

			DMA_DeInit(u->TxDMAStream);
			while (DMA_GetCmdStatus(u->TxDMAStream) != DISABLE) {
			};

			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) TxQ[s];
			DMA_Init(u->TxDMAStream, &DMA_InitStructure);

			DMA_SetCurrDataCounter(u->TxDMAStream, 0);
			DMA_ITConfig(u->TxDMAStream, DMA_IT_TC, ENABLE);

			USART_DMACmd(u->USART, USART_DMAReq_Tx, ENABLE);

		}

		RxEnabled[s] = true;
		USART_Cmd(u->USART, ENABLE);

	}
} // InitSerialPort


void InitSerialPort(uint8 s, boolean Enable, boolean SBusConfig) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	SerialPortDef * u;

	u = &SerialPorts[s];

	if (u->Used) {

		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_StructInit(&GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = u->TxPin | u->RxPin;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(u->Port, &GPIO_InitStructure);

		GPIO_PinAFConfig(u->Port, u->TxPinSource, u->USART_AF);
		GPIO_PinAFConfig(u->Port, u->RxPinSource, u->USART_AF);

		USART_StructInit(&USART_InitStructure);

		if (SBusConfig) {
			USART_InitStructure.USART_BaudRate = 100000; // 96000; //100000;
			USART_InitStructure.USART_WordLength = USART_WordLength_9b;
			USART_InitStructure.USART_StopBits = USART_StopBits_2;
			USART_InitStructure.USART_Parity = USART_Parity_Even;
			USART_Init(u->USART, &USART_InitStructure);
		} else {
			USART_InitStructure.USART_Parity = USART_Parity_No;
			USART_InitStructure.USART_BaudRate = u->Baud;
		}
		USART_Init(u->USART, &USART_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		TxQTail[s] = TxQHead[s] = TxQNewHead[s] = 0;

		if (u->InterruptsUsed) {
			RxQTail[s] = RxQHead[s] = 0;
			NVIC_InitStructure.NVIC_IRQChannel = u->ISR;
			NVIC_Init(&NVIC_InitStructure);

			USART_ITConfig(u->USART, USART_IT_RXNE, ENABLE);
		} else {
			// Common
			DMA_StructInit(&DMA_InitStructure);
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) &u->USART->DR;
			DMA_InitStructure.DMA_BufferSize = SERIAL_BUFFER_SIZE;

			// Receive DMA
			DMA_DeInit(u->RxDMAStream);
			while (DMA_GetCmdStatus(u->RxDMAStream) != DISABLE) {
			};

			DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) &u->USART->DR;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
			DMA_InitStructure.DMA_Channel = u->DMAChannel;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32) RxQ[s];

			DMA_Init(u->RxDMAStream, &DMA_InitStructure);
			DMA_Cmd(u->RxDMAStream, ENABLE);
			USART_DMACmd(u->USART, USART_DMAReq_Rx, ENABLE);

			RxQTail[s] = RxQHead[s] = SERIAL_BUFFER_SIZE
					- DMA_GetCurrDataCounter(u->RxDMAStream);

			// Transmit DMA
			NVIC_InitStructure.NVIC_IRQChannel = u->TxDMAISR;
			NVIC_Init(&NVIC_InitStructure);

			DMA_DeInit(u->TxDMAStream);
			while (DMA_GetCmdStatus(u->TxDMAStream) != DISABLE) {
			};

			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32) TxQ[s];
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_Init(u->TxDMAStream, &DMA_InitStructure);

			DMA_SetCurrDataCounter(u->TxDMAStream, 0);
			DMA_ITConfig(u->TxDMAStream, DMA_IT_TC, ENABLE);

			USART_DMACmd(u->USART, USART_DMAReq_Tx, ENABLE);
		}

		RxEnabled[s] = Enable;
		USART_Cmd(u->USART, ENABLE);
	}

} // InitSerialPort

void InitAnalogPorts(void) {
	uint8 a;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	DMA_InitTypeDef DMA_InitStructure;
	AnalogPinDef * u, *ux;

	if (ANALOG_CHANNELS > 0) {
		// all pins already configured as AIN

		ux = &AnalogPins[0];

		DMA_DeInit(ux->DMAStream);
		while (DMA_GetCmdStatus(ux->DMAStream) != DISABLE) {
		};

		DMA_StructInit(&DMA_InitStructure);
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32) &ux->ADCx->DR;
		DMA_InitStructure.DMA_BufferSize = ANALOG_CHANNELS;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize
				= DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_Channel = ux->DMAChannel;
		DMA_InitStructure.DMA_Memory0BaseAddr = (uint32) ADCValues;
		//DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;

		DMA_Init(ux->DMAStream, &DMA_InitStructure);
		DMA_Cmd(ux->DMAStream, ENABLE);

		ADC_CommonStructInit(&ADC_CommonInitStructure);
		ADC_CommonInit(&ADC_CommonInitStructure);

		ADC_StructInit(&ADC_InitStructure);
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
		ADC_InitStructure.ADC_ScanConvMode = ENABLE;
		ADC_InitStructure.ADC_NbrOfConversion = ANALOG_CHANNELS;
		ADC_Init(ux->ADCx, &ADC_InitStructure);

		for (a = 0; a < ANALOG_CHANNELS; a++) {
			u = &AnalogPins[a];

			ADC_RegularChannelConfig(u->ADCx, u->ADCChannel, u->Rank,
					ADC_SampleTime_28Cycles);
		}

		ADC_DMARequestAfterLastTransferCmd(ux->ADCx, ENABLE);

		ADC_DMACmd(ux->ADCx, ENABLE);
		ADC_Cmd(ux->ADCx, ENABLE);

		ADC_SoftwareStartConv(ux->ADCx);
	}

} // InitAnalog

void InitSensorInterrupts(void) {
#if defined(SPI_MPU)
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	/*
	 *            1- Configure the I/O in input mode using GPIO_Init()
	 *            2- Select the input source pin for the EXTI line using SYSCFG_EXTILineConfig()
	 *            3- Select the mode(interrupt, event) and configure the trigger
	 *               selection (Rising, falling or both) using EXTI_Init()
	 *            4- Configure NVIC IRQ channel mapped to the EXTI line using NVIC_Init()
	 *
	 *  @note  SYSCFG APB clock must be enabled to get write access to SYSCFG_EXTICRx
	 *         registers using RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	 */
	//pinInit(); // as input

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);
	EXTI_InitStruct.EXTI_Line = EXTI_Line14;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01; // zzz
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//pinInit();
	//SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource15);

#endif
} // InitSensorInterrupts

void usbEnableInterrupt(void) {
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

} // usbEnableInterrupt


void InitComboPorts(void) {
	uint8 CurrNoOfRCPins;

	CurrNoOfRCPins = 0;

#if defined(UAVXF4V3)

	uint8 i;

	//for (i = 0; i < MAX_SERIAL_PORTS; i++)
	//	USART_DeInit(SerialPorts[i].USART);

	switch (CurrComboPort1Config) {
		case CPPM_GPS_M7to10:
		CurrMaxPWMOutputs = (UsingDCMotors) ? 4 : 10;
		CurrNoOfRCPins = 1;
		GPSTxSerial = GPSRxSerial = Serial1;
		InitSerialPort(GPSRxSerial, false, false);
		RxUsingSerial = false;
		break;
		case ParallelPPM:
		CurrMaxPWMOutputs = (UsingDCMotors) ? 4 : 6;
		CurrNoOfRCPins = MAX_RC_INPS; //P(RCChannels);
		RxUsingSerial = false;
		GPSRxSerial = Serial0;
		GPSTxSerial = SoftSerialTx;
		break;
		default:
		CurrMaxPWMOutputs = (UsingDCMotors) ? 4 : 10;
		CurrNoOfRCPins = 0;
		RxUsingSerial = true;
		GPSRxSerial = TelemetrySerial;
		GPSTxSerial = SoftSerialTx;
		RCSerial = Serial1;
		InitSerialPort(RCSerial, false, CurrComboPort1Config
				== FutabaSBus_M7to10);
		break;
	} // switch

#elif defined(UAVXF4V4)
	uint8 i;

	if (CurrComboPort2Config != RF_GPS_V4) {
		I2CPorts[i2cMap[baroSel] - 1].Used |= (CurrComboPort2Config
				== RF_Baro_V4) || (CurrComboPort2Config == RF_Baro_Mag_V4);
		I2CPorts[i2cMap[magSel] - 1].Used
		|= (CurrComboPort2Config == RF_Mag_V4) || (CurrComboPort2Config
				== RF_Baro_Mag_V4);

		spiDevUsed[baroSel] = (CurrComboPort2Config != RF_Baro_V4)
		&& (CurrComboPort2Config != RF_Baro_Mag_V4);
		spiDevUsed[magSel] = (CurrComboPort2Config != RF_Mag_V4)
		&& (CurrComboPort2Config != RF_Baro_Mag_V4);

		//spiDevUsed[ms56xxSel] = false;
		//I2CPorts[1].Used = true;

		for (i = 0; i < MAX_I2C_PORTS; i++)
		i2cInit(i);
	}

	switch (CurrComboPort1Config) {
		case CPPM_GPS_M7to10:
		CurrMaxPWMOutputs = (UsingDCMotors) ? 4 : 10;
		CurrNoOfRCPins = 1;
		InitRCPins(CurrNoOfRCPins);
		GPSTxSerial = GPSRxSerial = Serial1;
		InitSerialPort(GPSRxSerial, false, false);
		RxUsingSerial = false;
		break;
		case ParallelPPM:
		CurrMaxPWMOutputs = (UsingDCMotors) ? 4 : 6;
		CurrNoOfRCPins = MAX_RC_INPS; //P(RCChannels);
		InitRCPins(CurrNoOfRCPins);
		RxUsingSerial = false;

		if (CurrComboPort2Config == RF_GPS_V4) {
			GPSRxSerial = GPSTxSerial = Serial2;
			InitSerialPort(GPSRxSerial, false, CurrComboPort1Config
					== FutabaSBus_M7to10); // zzz
		} else {
			GPSRxSerial = Serial0;
			GPSTxSerial = SoftSerialTx;
		}

		break;
		default:
		CurrMaxPWMOutputs = (UsingDCMotors) ? 4 : 10;
		CurrNoOfRCPins = 0;
		RxUsingSerial = true;

		if (CurrComboPort2Config == RF_GPS_V4) {
			GPSRxSerial = GPSTxSerial = Serial2;
			InitSerialPort(GPSRxSerial, false, CurrComboPort1Config
					== FutabaSBus_M7to10); // zzz
		} else {
			GPSRxSerial = TelemetrySerial;
			GPSTxSerial = SoftSerialTx;
		}
		RCSerial = Serial1;
		InitSerialPort(RCSerial, false, CurrComboPort1Config
				== FutabaSBus_M7to10);
		break;
	} // switch

	if (CurrwsNoOfLeds > 0) // temporary KLUDGE - need to remap gimbal function
	CurrMaxPWMOutputs = 4;

#elif defined(OMNIBUSF4V1)

	RxUsingSerial = !((CurrComboPort1Config == CPPM_GPS_M7to10)
			|| (CurrComboPort1Config == FutabaSBus_M7to10));

	RCSerial = Serial0;
	InitSerialPort(Serial0, false, CurrComboPort1Config
			== FutabaSBus_M7to10);

	TelemetrySerial = USBSerial;

	if (SerialPorts[Serial2].Used) {
		GPSRxSerial = GPSTxSerial = Serial2;
		InitSerialPort(GPSRxSerial, false, false);
	}

	CurrMaxPWMOutputs = 4;

#else

#endif

} // InitComboPorts

void InitHarness(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8 i;

	// Using all ports - could be generalised but .... later
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); // for interrupts zzz
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	//gke? RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	USART_DeInit(USART1);
#if (MAX_SERIAL_PORTS >1)
	USART_DeInit(USART2);
#endif
#if (MAX_SERIAL_PORTS >2)
	USART_DeInit(USART3);
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#if (MAX_SERIAL_PORTS >1)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
#endif
#if (MAX_SERIAL_PORTS >2)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
#endif

	RCC_ClearFlag();

	// Make all GPIO input by default
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	for (i = 0; i < MAX_PWM_OUTPUTS; i++) { // switch off all (potential) motor output pins
		pinInitOutput(&PWMPins[i]);
		digitalWrite(&PWMPins[i], 0);
	}

	for (i = 0; i < MAX_LEDS; i++) {
		pinInit(&LEDPins[i]);
		digitalWrite(&LEDPins[i], 1);
	}

	for (i = 0; i < MAX_GPIO_PINS; i++)
		pinInit(&GPIOPins[i]);
	BeeperOff();

	for (i = 0; i < MAX_I2C_PORTS; i++)
		i2cInit(i);

	Delay1mS(10);

	InitSerialPort(Serial0, true, false); // Must have telemetry initially!

	if (Aux2Sel < MAX_GPIO_PINS)
		digitalWrite(&GPIOPins[Aux2Sel], 1); // soft USART Tx

	for (i = 0; i < MAX_SPI_DEVICES; i++) { // deselect all
		pinInit(&SPISelectPins[i]);
		digitalWrite(&SPISelectPins[i], 1);
	}

	for (i = 0; i < MAX_SPI_PORTS; i++)
		spiInit(i);

	InitAnalogPorts();

} // InitHarness

