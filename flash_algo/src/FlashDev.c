/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2010 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Device Description for New Device Flash               */
/*                                                                     */
/***********************************************************************/

#include "..\inc\FlashOS.H"        // FlashOS Structures

#if defined (CODE_FLASH)

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "GCT301S Code Flash v2",       // Device Name 
   ONCHIP,                     // Device Type
   0x00000000,                 // Device Start Address
   0x00020000,                 // Device Size in Bytes (128kB)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   3000,                       // Program Page Timeout 1000 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

   // Specify Size and Address of Sectors
   0x00000100, 0x00000000,     // Sector Size 256B
   SECTOR_END
};
	
#elif defined (DATA_FLASH)

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "GCT301S Data Flash v2",       // Device Name 
   ONCHIP,                     // Device Type
   0x10000000,                 // Device Start Address
   0x00008000,                 // Device Size in Bytes (128kB)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   3000,                       // Program Page Timeout 1000 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

   // Specify Size and Address of Sectors 
   0x00000100, 0x00000000,     // Sector Size 256B
   SECTOR_END
};
	
#elif defined (NVR_CODE_FLASH)

struct FlashDevice const FlashDevice  =  {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "GCT301S NVR Code Flash v2",   // Device Name 
   ONCHIP,                     // Device Type
   0x00100000,                 // Device Start Address
   0x00000100,                 // Device Size in Bytes (256B)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   3000,                       // Program Page Timeout 1000 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

   // Specify Size and Address of Sectors 
   0x00000100, 0x00000000,     // Sector Size 256B
   SECTOR_END
};
#endif
