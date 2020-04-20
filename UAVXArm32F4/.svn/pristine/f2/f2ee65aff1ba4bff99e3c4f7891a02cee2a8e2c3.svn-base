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

__attribute__((__section__(".configflash")))     const int8 FlashNV[CONFIG_FLASH_SIZE];
__attribute__((__section__(".blackboxflash")))     const int8 BlackBoxNV[BLACKBOX_FLASH_SIZE];

void ReadBlockArmFlash(uint32 a, uint32 l, uint8 * v) {
	uint32 i;

	for (i = 0; i < l; i++)
		v[i] = *(int8 *) (a + i);

} // ReadBlockArmFlash

void EraseArmFlash(uint32 sector) {

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
			| FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	(FLASH_EraseSector(sector, VoltageRange_3) == FLASH_COMPLETE);

	FLASH_Lock();

} // EraseArmFlash

boolean WriteBlockArmFlash(boolean erase, uint32 sector, uint32 a, uint32 l,
		uint8 * v) {
	uint32 i;
	boolean r = true;

	if (erase)
		EraseArmFlash(sector);

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
			| FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	if (r) {
		for (i = 0; i < l; i += 4) {
			r = FLASH_ProgramWord(a + i, *(uint32 *) ((uint8 *) v + i))
					== FLASH_COMPLETE;
			if (!r)
				break;
		}
	}
	FLASH_Lock();

	return (r);

} // WriteBlockArmFlash


