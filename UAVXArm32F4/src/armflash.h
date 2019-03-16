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


#ifndef _armflash_h
#define _armflash_h


#define CONFIG_FLASH_ADDR (0x8004000)
#define CONFIG_FLASH_SIZE 0x4000 // 16K
#define CONFIG_FLASH_SECTOR	FLASH_Sector_1



#define BLACKBOX_FLASH_ADDR (0x8040000) // 128K
#define BLACKBOX_FLASH_SIZE 0x20000
#define BLACKBOX_FLASH_SECTOR	FLASH_Sector_6


void ReadBlockArmFlash(uint32 a, uint32 l, uint8 * v);
void EraseArmFlash(uint32 sector);
boolean WriteBlockArmFlash(boolean erase, uint32 sector, uint32 a, uint32 l, uint8 * F);

#endif

