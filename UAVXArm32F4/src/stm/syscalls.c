

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
    	*((uint32_t *)0x2001FFFC) = 0xDEADBEEF; // 128KB SRAM STM32F4XX

    __disable_irq();
    NVIC_SystemReset();
} // systemReset

caddr_t _sbrk ( int incr )
{
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
  return (caddr_t) -1; //prev_heap;
}



