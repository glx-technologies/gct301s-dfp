/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2010 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Flash Programming Functions adapted                   */
/*               for New Device 256kB Flash                            */
/*                                                                     */
/***********************************************************************/

/**
 * Modified by ngms 18 June 2015
 */
 
#include "..\inc\FlashOS.H"        // FlashOS Structures
#include "..\inc\gct301s_flash.h"

/* 
   Mandatory Flash Programming Functions (Called by FlashOS):
                int Init        (unsigned long adr,   // Initialize Flash
                                 unsigned long clk,
                                 unsigned long fnc);
                int UnInit      (unsigned long fnc);  // De-initialize Flash
                int EraseSector (unsigned long adr);  // Erase Sector Function
                int ProgramPage (unsigned long adr,   // Program Page Function
                                 unsigned long sz,
                                 unsigned char *buf);

   Optional  Flash Programming Functions (Called by FlashOS):
                int BlankCheck  (unsigned long adr,   // Blank Check
                                 unsigned long sz,
                                 unsigned char pat);
                int EraseChip   (void);               // Erase complete Device
      unsigned long Verify      (unsigned long adr,   // Verify Function
                                 unsigned long sz,
                                 unsigned char *buf);

       - BlanckCheck  is necessary if Flash space is not mapped into CPU memory space
       - Verify       is necessary if Flash space is not mapped into CPU memory space
       - if EraseChip is not provided than EraseSector for all sectors is called
*/

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */


#if   defined(CODE_FLASH)
FLASH_TypeDef *FLASH = CFLASH;
#elif defined(DATA_FLASH)
FLASH_TypeDef *FLASH = DFLASH;
#elif defined(NVR_CODE_FLASH)
FLASH_TypeDef *FLASH = CFLASH;
#endif

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) 
{
    /* stop LSRC (WDT clock) */
    SYSCTRL->LSRC_CTL.BITS.LSRC_ENB = 1;
  
    /* run at 8MHz HSRC */
    SYSCTRL->HSRC_CTL.BITS.PWRDN = 0;
    SYSCTRL->LSRC_CTL.BITS.VREF_ENB = 0;
    SYSCTRL->CLK_DIVSEL.VAL = 0;
    SYSCTRL->CLK_SRCSEL.VAL = 0;

    
#if defined(NVR_CODE_FLASH)
    FLASH->NVRP = FLASH_NVRP_READ; 
#endif
    FLASH->TIME = 0x0;
    
    GPIOA->ANDISSET = 0xFFFFF;
    GPIOB->ANDISSET = 0xFFFFF;
    GPIOC->ANDISSET = 0xFFFFF;
    
    GPIOA->PUDISSET = 0xFFFFF;
    GPIOB->PUDISSET = 0xFFFFF;
    GPIOC->PUDISSET = 0xFFFFF;
    
    return (0);                                  // Finished without Errors
}


/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) 
{
    FLASH->NVRP = 0; 
    FLASH->KEY = 0;
  
    return (0);                                  // Finished without Errors
}


/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */


int EraseChip (void) 
{
#if defined(CODE_FLASH) || defined(DATA_FLASH)
	FLASH->KEY = FLASH_KEY;
	FLASH->ERA = CERA;
	while( !FLASH->IF );  
	FLASH->KEY = 0;
#elif defined(NVR_CODE_FLASH)
	FLASH->KEY = FLASH_KEY;
	FLASH->NVRP = FLASH_NVRP_WRITE; 
    FLASH->ERA = NVRERA;
	while( !FLASH->IF );  
    FLASH->NVRP = 0; 
	FLASH->KEY = 0;
#endif  
    return (0);                                  // Finished without Errors
}


/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

volatile int32_t l_tmp;

int EraseSector (unsigned long adr) 
{
    GPIOA->BSCR = 0x1<<16 | 0x1;
    GPIOA->BSCR = 0x1<<16 | 0x1;
    
    
    FLASH->KEY = FLASH_KEY;
#if defined (NVR_CODE_FLASH)
    FLASH->NVRP = FLASH_NVRP_WRITE; 
#endif
    
#if defined (CODE_FLASH)  
	FLASH->ERASCTR = (adr >> 8);
    FLASH->ERA = MERA;
#elif defined (DATA_FLASH)
    FLASH->ERASCTR = ( (adr & 0x7FFFUL) >> 8 );
    FLASH->ERA = MERA;
#elif defined (NVR_CODE_FLASH)
    FLASH->ERA = NVRERA;            
#endif	
	while( !FLASH->IF ) {
        l_tmp = M32(0x4);
    }
  
#if defined (NVR_CODE_FLASH)
    FLASH->NVRP = 0;
#endif	
	FLASH->KEY = 0;
    
    return (0);                                  // Finished without Errors
}


/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) 
{
	uint32_t i;
	uint32_t l_adr;
  
    FLASH->KEY = FLASH_KEY;
#if defined (NVR_CODE_FLASH)
    FLASH->NVRP = FLASH_NVRP_WRITE;
#endif

#if defined (CODE_FLASH)
    l_adr = adr;
#elif defined (DATA_FLASH)
    l_adr = adr & 0x7FFFUL;
#elif defined (NVR_CODE_FLASH)
    l_adr = adr & 0xFFUL;
#endif
  
	for (i=0; i<sz; i++, l_adr++){
		FLASH->PROGDATA = buf[i];
        FLASH->PROGADDR = l_adr;

#if defined (NVR_CODE_FLASH)
        FLASH->PROG = NVRROG;
#else
        FLASH->PROG = MPROG;
#endif
        while ( !FLASH->IF );
	}
	
	FLASH->KEY = 0;
#if defined (NVR_CODE_FLASH)
    FLASH->NVRP = 0;
#endif	
  
    return (0);                                  // Finished without Errors
}

/*
 *  Verify
 *
 */
unsigned long Verify (unsigned long adr, unsigned long sz, unsigned char *buf)
{
    uint32_t i;
    uint8_t tmp;
    uint32_t l_adr;
  
    l_adr = adr;

#if defined (NVR_CODE_FLASH)
    FLASH->NVRP = FLASH_NVRP_READ;
#endif
	for (i=0; i<sz; i++, l_adr++){
        tmp = M8(l_adr);
		if ( buf[i] != tmp) {
            break;
        }
	}
#if defined (NVR_CODE_FLASH)
    FLASH->NVRP = 0;      
#endif  
    return (adr + i);
}
