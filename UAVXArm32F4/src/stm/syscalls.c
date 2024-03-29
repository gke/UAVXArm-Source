// based on a example-code from Keil for CS G++

//for caddr_t (typedef char * caddr_t;)
//#include <sys/types.h>

//gke

#include "UAVX.h"
#include <ctype.h>
#include <stdlib.h>
#include <sys/types.h>

void systemReset(boolean useBootLoader) {
	// generally due to Nesbitt and Ihlien

	if (useBootLoader)
		*((uint32_t *) 0x2001FFFC) = 0xDEADBEEF; // 128KB SRAM STM32F4XX

	__disable_irq();
	NVIC_SystemReset();
} // systemReset

caddr_t _sbrk(int incr) {
	/*
	 static unsigned char *heap = NULL;
	 unsigned char *prev_heap;

	 if (heap == NULL) {
	 heap = (unsigned char *)& __HEAP_START;
	 }
	 prev_heap = heap;

	 if (heap + incr > stack_ptr) {
	 //errno = ENOMEM;
	 return (caddr_t) -1;
	 }

	 heap += incr;
	 */
	return (caddr_t) - 1; //prev_heap;
}

// https://stackoverflow.com/questions/34196663/stm32-how-to-get-last-reset-status

/// @brief  Possible STM32 system reset causes

uint8 ResetCause = 0;

const char * ResetCauseNames[] = { "UNKNOWN_CAUSE", "LOW_POWER", "WINDOW_WATCHDOG",
		"INDEPENDENT_WATCHDOG", "SOFTWARE", "POWER_ON_POWER_DOWN",
		"EXTERNAL_RESET_PIN", "BROWNOUT" };

uint8 GetResetCause(void) {
	uint8 c;

	if (RCC_GetFlagStatus(RCC_FLAG_LPWRRST))
		c = LOW_POWER_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_WWDGRST))
		c = WINDOW_WATCHDOG_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST))
		c = INDEPENDENT_WATCHDOG_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_SFTRST))
		c = SOFTWARE_RESET; // induced by NVIC_SystemReset() call
	else if (RCC_GetFlagStatus(RCC_FLAG_PORRST))
		c = POWER_ON_POWER_DOWN_RESET;
	else if (RCC_GetFlagStatus(RCC_FLAG_PINRST))
		c = EXTERNAL_RESET_PIN_RESET;
	// Needs to come *after* checking the RCC_FLAG_PORRST flag in order to
	//ensure first that the reset cause is NOT a POR/PDR reset.
	else if (RCC_GetFlagStatus(RCC_FLAG_BORRST))
		c = BROWNOUT_RESET;
	else
		c = UNKNOWN_RESET;

	// Clear all the reset flags or else they will remain set during future
	// resets until system power is fully removed.
	RCC_ClearFlag();

	return (c);

} // CheckResetCause



