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

// USART routines

#include "UAVX.h"

// Rewritten from AQ original Copyright © 2011 Bill Nesbitt

volatile uint8 TxQ[MAX_SERIAL_PORTS][SERIAL_BUFFER_SIZE] __attribute__((aligned(4)));
volatile int16 TxQTail[MAX_SERIAL_PORTS] = { 0, };
volatile int16 TxQHead[MAX_SERIAL_PORTS] = { 0, };
volatile int16 TxQNewHead[MAX_SERIAL_PORTS] = { 0, };

volatile uint8 RxQ[MAX_SERIAL_PORTS][SERIAL_BUFFER_SIZE] __attribute__((aligned(4)));
volatile int16 RxQTail[MAX_SERIAL_PORTS] = { 0, };
volatile int16 RxQHead[MAX_SERIAL_PORTS] = { 0, };
volatile int16 RxQNewHead[MAX_SERIAL_PORTS] = { 0, };
volatile boolean RxEnabled[MAX_SERIAL_PORTS] = { false, };
volatile boolean RxCTS[MAX_SERIAL_PORTS] = { false, };

volatile uint32 RxDMAPos[MAX_SERIAL_PORTS] = { 0, };

uint8 TxCheckSum[MAX_SERIAL_PORTS];
uint32 SoftUSARTBaudRate = 115200;
boolean RxUsingSerial = false;

void SerialTxDMA(uint8 s) {

	SerialPorts[s].TxStream->M0AR = (uint32) &TxQ[s][TxQHead[s]];

	if (TxQTail[s] > TxQHead[s]) { // Tail not wrapped around yet
		DMA_SetCurrDataCounter(SerialPorts[s].TxStream, TxQTail[s] - TxQHead[s]);
		TxQNewHead[s] = TxQTail[s];
	} else {// Tail has wrapped do balance from Head to end of Buffer
		DMA_SetCurrDataCounter(SerialPorts[s].TxStream, SERIAL_BUFFER_SIZE
				- TxQHead[s]);
		TxQNewHead[s] = 0;
	}
	DMA_Cmd(SerialPorts[s].TxStream, ENABLE);

} // SerialTxDMA

boolean SerialAvailable(uint8 s) {
	boolean r;
	//uint8 ch;

	switch (s) {
	case USBSerial:
		r = usbAvailable();
		break;
	case SoftSerialTx:
		r = false;
		break;
	default:
		if (SerialPorts[s].DMAUsed) {
			RxQTail[s] = SERIAL_BUFFER_SIZE - DMA_GetCurrDataCounter(
					SerialPorts[s].RxStream);
			r = RxQHead[s] != RxQTail[s];
		} else if (SerialPorts[s].InterruptsUsed)
			r = RxQTail[s] != RxQHead[s];
		else
			r = (USART_GetFlagStatus(SerialPorts[s].USART, USART_FLAG_RXNE)
					== SET);
		break;
	} // switch

	return (r);
} // SerialAvailable

uint8 RxChar(uint8 s) {
	uint8 ch;

	ch = ASCII_NUL;
	switch (s) {
	case SoftSerialTx:
		break;
	case USBSerial:
		ch = USBRxChar();
		break;
	default:
		if (SerialPorts[s].DMAUsed || SerialPorts[s].InterruptsUsed) {
			ch = RxQ[s][RxQHead[s]];
			RxQHead[s] = (RxQHead[s] + 1) & (SERIAL_BUFFER_SIZE - 1);
		} else
			ch = USART_ReceiveData(SerialPorts[s].USART);
		break;
	}// switch

	return (ch);
} // RxChar

uint8 PollRxChar(uint8 s) {
	uint8 ch;

	switch (s) {
	case SoftSerialTx:
		return (ASCII_NUL);
		break;
	case USBSerial:
		if (SerialAvailable(s)) {
			ch = USBRxChar();
			if (!Armed())
				USBTxChar(ch); // echo for UAVPSet
			return (ch);
		} else
			return (ASCII_NUL);
		break;
	default:
		if (SerialAvailable(s)) {
			ch = RxChar(s);
			if (!Armed())
				TxChar(s, ch); // echo for UAVPSet
			return (ch);
		} else
			return (ASCII_NUL);
	}

} // PollRxChar


void TxChar(uint8 s, uint8 ch) {
	int16 NewTail;

	TxCheckSum[s] ^= ch;

	if (!BLHeliSuiteActive)
		BlackBox(ch);

	switch (s) {
	case USBSerial:
		USBTxChar(ch);
		break;
	case SoftSerialTx:
		SoftTxChar(ch); // blocking???
		break;
	default:
		if (SerialPorts[s].DMAUsed || SerialPorts[s].InterruptsUsed) {
			NewTail = (TxQTail[s] + 1) & (SERIAL_BUFFER_SIZE - 1);
			while (NewTail == TxQHead[s]) { // BLOCKING!!!!!!!!!!!!
			};
			TxQ[s][TxQTail[s]] = ch;
			// tail points to NEXT free slot
			TxQTail[s] = NewTail;

			if (SerialPorts[s].DMAUsed) {
				if (DMA_GetCmdStatus(SerialPorts[s].TxStream) == DISABLE)
					SerialTxDMA(s);
			} else {
				// if TXE then interrupt will be pending
				USART_ITConfig(SerialPorts[s].USART, USART_IT_TXE, ENABLE);
			}
		} else {
			while (USART_GetFlagStatus(SerialPorts[s].USART, USART_FLAG_TXE)
					== RESET) { // BLOCKING!!!!!!
			};
			USART_SendData(SerialPorts[s].USART, ch);
		}
		break;
	} //switch

} // TxChar

void SerialISR(uint8 s) {
	uint8 ch;

	if (USART_GetITStatus(SerialPorts[s].USART, USART_IT_RXNE) == SET) {
		ch = USART_ReceiveData(SerialPorts[s].USART);
		if (RxEnabled[s]) {
			if ((s == RCSerial) && RxUsingSerial)
				RCUSARTISR(ch); // longish interrupt service time - could reduce?
			else {
				RxQ[s][RxQTail[s]] = ch;
				RxQTail[s] = (RxQTail[s] + 1) & (SERIAL_BUFFER_SIZE - 1);
				if (RxQTail[s] == RxQHead[s]) { // full
					//USART_ITConfig(SerialPorts[s].USART, USART_IT_RXNE, DISABLE);
				}
			}
		}
	}

	if (USART_GetITStatus(SerialPorts[s].USART, USART_IT_TXE) == SET) {
		if (TxQTail[s] != TxQHead[s]) {
			ch = TxQ[s][TxQHead[s]];

			USART_SendData(SerialPorts[s].USART, ch);

			TxQHead[s] = (TxQHead[s] + 1) & (SERIAL_BUFFER_SIZE - 1);
		}
		if (TxQHead[s] == TxQTail[s])
			USART_ITConfig(SerialPorts[s].USART, USART_IT_TXE, DISABLE);
	}
} // SerialISR

void SoftTxChar(uint8 ch) {
	uint8 b;
	const uint16 d = (uint16) (1000000.0f / SoftUSARTBaudRate + 0.5f);

	if (!Armed()) { // MUST NOT BE USED IN FLIGHT

		DigitalWrite(&GPIOPins[Aux2Sel].P, false);
		Delay1uS(d);
		for (b = 0; b < 8; b++) {
			DigitalWrite(&GPIOPins[Aux2Sel].P, (ch & 1) != 0);
			Delay1uS(d);
			ch = ch >> 1;
		}
		DigitalWrite(&GPIOPins[Aux2Sel].P, true);
		Delay1uS(d);
	}
} // SoftTxChar

