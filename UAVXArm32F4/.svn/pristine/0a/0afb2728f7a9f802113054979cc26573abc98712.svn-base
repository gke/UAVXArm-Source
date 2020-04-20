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

///////////////////////////////////////////////////////////////////////////////
// Cycle Counter
///////////////////////////////////////////////////////////////////////////////

//******************************************************************************

// From http://forums.arm.com/index.php?showtopic=13949
//https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=https%3a%2f%2fmy.st.com%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fcortex_mx_stm32%2fcompensating%20latencies%20on%20STM32F4%20interrupts&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=2629
/*
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL = (volatile unsigned int *) 0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR = (volatile unsigned int *) 0xE000EDFC; //address of the register

void SysTick_HandlerOLD(void) {
	sysTickCycleCounter = *DWT_CYCCNT;
	sysTickUptime++;

	// sd card support
	if (sd_card_timer != 0x00)
		sd_card_timer--;

}

void EnableTiming(void) {
	static int enabled = 0;

	if (!enabled) {
		*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
		*DWT_CYCCNT = 0; // reset the counter
		*DWT_CONTROL = *DWT_CONTROL | 1; // enable the counter
		enabled = 1;
	}
}

void TimingDelay(unsigned int tick) {
	unsigned int start, current;

	start = *DWT_CYCCNT;
	do {
		current = *DWT_CYCCNT;
	} while ((current - start) < tick);
}

*/

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

volatile uint32 TicksuS;

void cycleCounterInit(void) {
	RCC_ClocksTypeDef clocks;

	RCC_GetClocksFreq(&clocks);
	TicksuS = clocks.SYSCLK_Frequency / 1000000;
	// enable DWT access
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// enable the CPU cycle counter
	DWT_CTRL |= CYCCNTENA;
} // cycleCounterInit


uint32 uSClock(void) { // wraps at 71 minutes
	volatile uint32 ms, cycle_cnt;

	do {
		ms = sysTickUptime;
		cycle_cnt = SysTick->VAL; //*DWT_CYCCNT; //
		asm volatile("\tnop\n");
	} while (ms != sysTickUptime);

	return (ms * 1000) + (TicksuS * 1000 - cycle_cnt) / TicksuS;
	//return (ms * 1000) + ((8000 - cycle_cnt) >> 3);

} // uSClock

void Delay1uS(uint16 d) {
	// TODO: needs round up
	uint32 TimeOut;

	TimeOut = uSClock() + d;
	while (uSClock() < TimeOut) {
	};

} // Delay1uS


__attribute__((always_inline))      inline uint32 mSClock(void) {
	return (sysTickUptime);
} // mSClock

void Delay1mS(uint16 d) {
	// TODO: needs round up
	uint32 TimeOut;

	TimeOut = mSClock() + d + 1; // clock may be rolling over
	while (mSClock() < TimeOut) {
	};

} // Delay1mS

real32 dTUpdate(uint32 NowuS, uint32 * LastUpdateuS) {
	real32 dT;

	NowuS = uSClock();
	dT = (NowuS - *LastUpdateuS) * 0.000001f;
	*LastUpdateuS = NowuS;

	return (dT);
} // dtUpdate


void mSTimer(uint32 NowmS, uint8 t, int32 TimePeriod) {
	mS[t] = NowmS + TimePeriod;
} // mSTimer

void uSTimer(uint32 NowuS, uint8 t, int32 TimePeriod) {
	uS[t] = NowuS + TimePeriod;
} // uSTimer


void InitClocks(void) {
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000); // 1mS
} // InitClocks

