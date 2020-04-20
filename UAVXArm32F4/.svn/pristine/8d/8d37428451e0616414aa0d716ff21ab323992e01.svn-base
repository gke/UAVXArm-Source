// Copyright (c) 2012 John Ihlein.  All rights reserved.


#include "UAVX.h"

#include "usbd_cdc_core.h"
#include "usb_core.h"
#include "usbd_desc.h"
#include "usbd_usr.h"

#include "drv_usb.h"

#include <stdarg.h>

volatile bool usbDeviceConfigured = false;

bool previoususbDeviceConfigured = false;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;


//enum { expandEvr = 1 };
//
//void USBListenerCB(evr_t e)
//{
//    if (expandEvr)
//        USBPrintF("EVR-%s %8.3fs %s (%04X)\n", evrToSeverityStr(e.evr), (float)e.time/1000., evrToStr(e.evr), e.reason);
//    else
//        USBPrintF("EVR:%08X %04X %04X\n", e.time, e.evr, e.reason);
//}

/*
 uint8 USBDetectPin;

 //#define IOCFG_IN_FLOATING    IO_CONFIG(GPIO_MODE_INPUT,     GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
 // NONE initializer for ioTag_t variables
 //#define IO_TAG_NONE 0

 // NONE initializer for IO_t variable
 //#define IO_NONE ((IO_t)0)

 void USBCableDetectDeinit(void) {
 #if defined(USB_DETECT_PIN)
 //IOConfigGPIO(USBDetectPin, IOCFG_IN_FLOATING); // IO_CONFIG(GPIO_MODE_INPUT, GPIO_SPEED_FREQ_LOW,  GPIO_NOPULL)
 // USBDetectPin = 0;
 #endif
 }
 */
/*
 void USBCableDetectInit(void) {
 #if defined(USB_DETECT_PIN)
 USBDetectPin = digitalRead(&GPIOPins[USBDisconnectSel]);

 // IOInit(USBDetectPin, OWNER_USB_DETECT, 0);
 // IOConfigGPIO(USBDetectPin, IOCFG_OUT_PP);
 #endif
 } // USBCableDetectInit(

 bool USBCableIsInserted(void) {
 #if defined(USB_DETECT_PIN)
 return digitalRead(&GPIOPins[USBDisconnectSel]);
 #else
 return false;
 #endif
 } // USBCableIsInserted

 */
void usbGenerateDisconnectPulse(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_PinSource12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_PinSource12);
	Delay1mS(200);
	GPIO_SetBits(GPIOA, GPIO_PinSource12);

} // usbGenerateDisconnectPulse


uint8_t usbRxChar(void) {
	uint8_t data;
	if (usbDeviceConfigured) {
		CDC_Receive_DATA(&data);
		return data;
	} else
		return (0);
} // USBRxChar


void usbTxChar(char ch) {
	if (usbDeviceConfigured)
		CDC_Send_DATA((uint8 *)&ch, 1);
} // usbTxChar


void usbTxString(char* str) {
	if (usbDeviceConfigured)
		CDC_Send_DATA((uint8_t *) str, strlen(str));
} // USBTxString


/*
void usbPrintF(const char * fmt, ...) {
	char buf[512];

	va_list vlist;
	va_start(vlist, fmt);

	vsnprintf(buf, sizeof(buf), fmt, vlist);
	USBPrint(buf);
	va_end(vlist);
} // usbPrintF


void usbPrintBinary(const uint8_t *buf, uint16_t length) {
	int i;
	char *ptr = (char *) buf;
	if (USBDeviceConfigured)
		for (i = 0; i < length; i++) {
			CDC_Send_DATA(*ptr, 1);
			ptr++;
		}
} // usbPrintBinary
*/

void usbActive(boolean forceCheck) {

	//  if ((USBDeviceConfigured != previousUSBDeviceConfigured)  || forceCheck)
	//      if (USBDeviceConfigured)
	//      {
	//          cliPortAvailable       = &USBAvailable;
	//          cliPortPrint           = &USBPrint;
	//          cliPortPrintF          = &USBPrintF;
	//          cliPortRead            = &USBRead;
	//
	//          mavlinkPortPrintBinary = &USBPrintBinary;
	//      }
	//      else
	//      {
	//          cliPortAvailable       = &uart1Available;
	//          cliPortClearBuffer     = &uart1ClearBuffer;
	//            cliPortPrint           = &uart1Print;
	//          cliPortPrintF          = &uart1PrintF;
	//          cliPortRead            = &uart1Read;
	//
	//          mavlinkPortPrintBinary = &uart1PrintBinary;
	//
	//          cliPortClearBuffer();
	//      }
	//
	//  previousUSBDeviceConfigured = USBDeviceConfigured;

	if ((usbDeviceConfigured != previoususbDeviceConfigured) || forceCheck)
		if (usbDeviceConfigured) {
			//            cliPortAvailable = &USBAvailable;
			//            cliPortPrint = &USBPrint;
			//            cliPortPrintF = &USBPrintF;
			//            cliPortRead = &USBRead;

			//            mavlinkPortPrintBinary = &USBPrintBinary;
			//            mavlinkPortAvailable = &USBAvailable;
			//            mavlinkPortRead = &USBRead;
			//            mavlinkPortPrintBinary = &uart3PrintBinary;
			//            mavlinkPortAvailable = &uart3Available;
			//            mavlinkPortRead = &uart3Read;
		} else {
			//		    cliPortAvailable       = &uart3Available;
			//		    cliPortClearBuffer     = &uart3ClearBuffer;
			//            cliPortPrint           = &uart3Print;
			//		    cliPortPrintF          = &uart3PrintF;
			//		    cliPortRead            = &uart3Read;
			//
			//	 	    mavlinkPortPrintBinary = &uart3PrintBinary;
			//            mavlinkPortAvailable   = &uart3Available;
			//            mavlinkPortRead        = &uart3Read;
		}

	previoususbDeviceConfigured = usbDeviceConfigured;
} // usbActive


void usbConnect(void) {

	usbGenerateDisconnectPulse();

	Delay1mS(200);

	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb,
			&USR_cb);

	Delay1mS(200);
	usbActive(true);
	if (!usbDeviceConfigured) {
	}

} // usbConnect
