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

// Master Clock

volatile timemS sysTickUptime = 0;
volatile uint32 sysTickCycleCounter = 0;

void SysTick_Handler(void) {
	sysTickCycleCounter = SysTick->VAL; // *DWT_CYCCNT;
	sysTickUptime++;
} // SysTick_Handler

void NMI_Handler(void) {
}

void HardFault_Handler(void) {
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1) {
		Catastrophe();
	}
}

void MemManage_Handler(void) {
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1) {
		Catastrophe();
	}
}

void BusFault_Handler(void) {
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1) {
		Catastrophe();
	}
}

void UsageFault_Handler(void) {
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1) {
		Catastrophe();
	}
}

void SVC_Handler(void) {
}

void DebugMon_Handler(void) {
}

void PendSV_Handler(void) {
}

//______________________________________________________________________________________________

// RC Timers

void TIM8_BRK_TIM12_IRQHandler(void) {

	if (TIM_GetITStatus(TIM8, TIM_IT_CC2) == SET) {
		//RCSerialISR(TIM_GetCapture1(TIM8));
		LEDToggle(ledYellowSel);
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC2);
	}

} // TIM8_BRK_TIM12_IRQHandler

void TIM2_IRQHandler(void) {

	if (CurrRxType == CPPMRx) {
		if (TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET)
			RCSerialISR(TIM_GetCapture1(TIM2));
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
	} else if (CurrRxType == ParallelPPMRx)
		RCParallelISR(TIM2);

} // TIM2_IRQHandler

void TIM3_IRQHandler(void) {

	if (CurrRxType == CPPMRx) {
		if (TIM_GetITStatus(TIM3, TIM_IT_CC1) == SET)
			RCSerialISR(TIM_GetCapture1(TIM3));
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	} else if (CurrRxType == ParallelPPMRx)
		RCParallelISR(TIM3);

} // TIM3_IRQHandler


//______________________________________________________________________________________________

// DMA


void DMA2_Stream2_IRQHandler(void) {

#if (defined(USE_WS2812) || defined(USE_WS2812B))
	// Half-Transfer completed
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_HTIF2)) {
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_HTIF2);
		UpdateWSLEDBuffer(WSLEDPWMBuffer);
	}

	// Transfer completed
	if (DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2)) {
		DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
		UpdateWSLEDBuffer(WSLEDPWMBuffer + (WSLEDBufferSize >> 1));
	}
#endif

} // DMA2_Stream2_IRQHandler


void DMA2_Stream7_IRQHandler(void) {

	DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
	DMA_Cmd(DMA2_Stream7, DISABLE);

	TxQHead[1] = TxQNewHead[1];
	if (TxQHead[1] != TxQTail[1])
		SerialTxDMA(1);

} // DMA2_Channel7_IRQHandler

void DMA1_Stream6_IRQHandler(void) {

	DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);
	DMA_Cmd(DMA1_Stream6, DISABLE);

	TxQHead[2] = TxQNewHead[1];
	if (TxQHead[2] != TxQTail[2])
		SerialTxDMA(2);

} // DMA1_Channel6_IRQHandler


void DMA1_Stream3_IRQHandler(void) {

	DMA_ClearITPendingBit(DMA1_Stream3, DMA_IT_TCIF3);
	DMA_Cmd(DMA1_Stream3, DISABLE);
	TxQHead[3] = TxQNewHead[3];
	if (TxQHead[3] != TxQTail[3])
		SerialTxDMA(3);

} // DMA1_Stream3_IRQHandler

void DMA1_Stream4_IRQHandler(void) {

	DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
	DMA_Cmd(DMA1_Stream4, DISABLE);

	TxQHead[4] = TxQNewHead[4];
	if (TxQHead[4] != TxQTail[4])
		SerialTxDMA(4);

} // DMA1_Stream4_IRQHandler

//______________________________________________________________________________________________

// Serial

void USART1_IRQHandler(void) {
	SerialISR(1);
} // USART1_IRQHandler


void USART2_IRQHandler(void) {
	SerialISR(2);
} // USART2_IRQHandler


void USART3_IRQHandler(void) {
#if defined (UAXF4V4)
	if (CurrComboPort2Config == RF_GPS_V4)
#endif
	SerialISR(3);
} // USART2_IRQHandler

void UART4_IRQHandler(void) {
	SerialISR(4);
} // UART4_IRQHandler


//______________________________________________________________________________________________

// I2C

void I2C1_ER_IRQHandler(void) {
	i2c_er_handler(1);
}

void I2C1_EV_IRQHandler(void) {
	i2c_ev_handler(1);
}

void I2C2_ER_IRQHandler(void) {
	i2c_er_handler(2);
}

void I2C2_EV_IRQHandler(void) {
	i2c_ev_handler(2);
}

void I2C3_ER_IRQHandler(void) {
	i2c_er_handler(3);
}

void I2C3_EV_IRQHandler(void) {
	i2c_ev_handler(3);
}

