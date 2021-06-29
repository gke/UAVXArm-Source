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

#define BBQ_BUFFER_SIZE 2048
#define BBQ_BUFFER_MASK (BBQ_BUFFER_SIZE-1)

uint32 CurrBBMemAddr;

int8 BBQ[BBQ_BUFFER_SIZE];
volatile uint16 BBQHead;
volatile uint16 BBQTail;
volatile uint16 BBQEntries;
boolean BlackBoxEnabled = false;
uint8 BBReplaySpeed = 1;

void UpdateBlackBox(void) {
	uint32 i;

	if (F.HaveNVMem)
		if ((uSClock() >= uS[MemReadyuS]) && (BBQEntries >= NVMemBlockSize)) {

			for (i = 0; i < NVMemBlockSize; i++) {
				BBQHead = (BBQHead + 1) & BBQ_BUFFER_MASK;
				NVMemBuffer[i] = BBQ[BBQHead];
			}
			WriteNVMemBlock(CurrBBMemAddr, NVMemBlockSize, NVMemBuffer);
			BBQEntries -= NVMemBlockSize;

			CurrBBMemAddr += NVMemBlockSize;

			if (CurrBBMemAddr >= NVMemSize) // wrap around to capture end of flight
				CurrBBMemAddr = 0;

		}
} // UpdateBlackBox


void BlackBox(uint8 ch) {
	uint16 NewTail;

	if (BlackBoxEnabled) {
		NewTail = (BBQTail + 1) & BBQ_BUFFER_MASK;
		BBQ[NewTail] = ch;
		BBQTail = NewTail;
		BBQEntries++;
	}

} // BlackBox

void DumpBlackBox(uint8 s) {
	int32 seqNo;
	int32 a;
	uint32 MaxMemoryUsed;
	uint16 i;
	boolean Finish;

	F.DumpingBlackBox = true;

	seqNo = 0;

	if (F.HaveNVMem) {

		for (i = 0; i < 10; i++) {
			SendFlightPacket(s);
			Delay1mS(5);
		}

		do {
			ReadBlockNVMem(a, NVMemBlockSize, NVMemBuffer);
			Finish = true;
			for (i = 0; i < NVMemBlockSize; i++)
				Finish &= NVMemBuffer[i] == -1;
			if (!Finish) {
				if (NVMemBlockSize > 128) {
					SendBBPacket(s, seqNo++, 128, &NVMemBuffer[0]);
					SendBBPacket(s, seqNo++, NVMemBlockSize - 128, &NVMemBuffer[128]);
				} else
					SendBBPacket(s, seqNo++, (uint8) NVMemBlockSize, &NVMemBuffer[0]);
				a += NVMemBlockSize;
			}
		} while ((a < NVMemSize) && !Finish);

		NVMemBuffer[0] = 0;
		SendBBPacket(s, -1, 1, NVMemBuffer);
	}

	F.DumpingBlackBox = false;

} // DumpBlackBox


void InitBlackBox(void) {

	BBQHead = BBQTail = BBQEntries = 0;
	CurrBBMemAddr = 0;

} // InitBlackBox
