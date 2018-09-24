
/****************************************************************************************************//**
 * @file     GCT301S.h
 *
 * @brief    CMSIS Cortex-M0 Peripheral Access Layer Header File for
 *           GCT301S from SZGC.
 *
 * @version  V1.2
 * @date     25. August 2018
 *
 * @note     Generated with SVDConv V2.87l 
 *           from CMSIS SVD File 'gct301s.svd' Version 1.2,
 *
 * @par      ARM Limited (ARM) is supplying this software for use with Cortex-M
 *           processor based microcontroller, but can be equally used for other
 *           suitable processor architectures. This file can be freely distributed.
 *           Modifications to this file shall be clearly marked.
 *           
 *           THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *           OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *           MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *           ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *           CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER. 
 *
 *******************************************************************************************************/



/** @addtogroup SZGC
  * @{
  */

/** @addtogroup GCT301S
  * @{
  */

#ifndef GCT301S_H
#define GCT301S_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum {
/* -------------------  Cortex-M0 Processor Exceptions Numbers  ------------------- */
  Reset_IRQn                    = -15,              /*!<   1  Reset Vector, invoked on Power up and warm reset                 */
  NonMaskableInt_IRQn           = -14,              /*!<   2  Non maskable Interrupt, cannot be stopped or preempted           */
  HardFault_IRQn                = -13,              /*!<   3  Hard Fault, all classes of Fault                                 */
  SVCall_IRQn                   =  -5,              /*!<  11  System Service Call via SVC instruction                          */
  PendSV_IRQn                   =  -2,              /*!<  14  Pendable request for system service                              */
  SysTick_IRQn                  =  -1,              /*!<  15  System Tick Timer                                                */
/* ---------------------  GCT301S Specific Interrupt Numbers  --------------------- */
  WDT_IRQn                      =   1,              /*!<   1  WDT                                                              */
  RTC_IRQn                      =   2,              /*!<   2  RTC                                                              */
  UART0_IRQn                    =   3,              /*!<   3  UART0                                                            */
  UART1_IRQn                    =   4,              /*!<   4  UART1                                                            */
  UART2_IRQn                    =   5,              /*!<   5  UART2                                                            */
  UART3_IRQn                    =   6,              /*!<   6  UART3                                                            */
  UART4_IRQn                    =   7,              /*!<   7  UART4                                                            */
  UART5_IRQn                    =   8,              /*!<   8  UART5                                                            */
  SPI_IRQn                      =   9,              /*!<   9  SPI                                                              */
  CFLASH_IRQn                   =  10,              /*!<  10  CFLASH                                                           */
  DFLASH_IRQn                   =  11,              /*!<  11  DFLASH                                                           */
  TIMER0_IRQn                   =  12,              /*!<  12  TIMER0                                                           */
  TIMER1_IRQn                   =  13,              /*!<  13  TIMER1                                                           */
  TIMER2_IRQn                   =  14,              /*!<  14  TIMER2                                                           */
  TIMER3_IRQn                   =  15,              /*!<  15  TIMER3                                                           */
  TIMER4_IRQn                   =  16,              /*!<  16  TIMER4                                                           */
  EXT0_IRQn                     =  17,              /*!<  17  GPIO 2 interrupt                                                 */
  EXT1_IRQn                     =  18,              /*!<  18  GPIO 3 interrupt                                                 */
  EXT2_IRQn                     =  19,              /*!<  19  GPIO 4 interrupt                                                 */
  EXT3_IRQn                     =  20,              /*!<  20  GPIO 5 interrupt                                                 */
  LCD_IRQn                      =  21,              /*!<  21  LCD                                                              */
  ADC_IRQn                      =  22,              /*!<  22  ADC                                                              */
  GPIOA_IRQn                    =  23,              /*!<  23  GPIOA                                                            */
  GPIOB_IRQn                    =  24,              /*!<  24  GPIOB                                                            */
  GPIOC_IRQn                    =  25,              /*!<  25  GPIOC                                                            */
  IRQ26_IRQn                    =  26,              /*!<  26  IRQ26                                                            */
  IRQ27_IRQn                    =  27,              /*!<  27  IRQ27                                                            */
  IRQ28_IRQn                    =  28,              /*!<  28  IRQ28                                                            */
  IRQ29_IRQn                    =  29,              /*!<  29  IRQ29                                                            */
  IRQ30_IRQn                    =  30,              /*!<  30  IRQ30                                                            */
  IRQ31_IRQn                    =  31               /*!<  31  IRQ31                                                            */
} IRQn_Type;


/** @addtogroup Configuration_of_CMSIS
  * @{
  */


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------Configuration of the Cortex-M0 Processor and Core Peripherals---------------- */
#define __CM0_REV                 0x0000            /*!< Cortex-M0 Core Revision                                               */
#define __MPU_PRESENT                  0            /*!< MPU present or not                                                    */
#define __NVIC_PRIO_BITS               2            /*!< Number of Bits used for Priority Levels                               */
#define __Vendor_SysTickConfig         0            /*!< Set to 1 if different SysTick Config is used                          */
/** @} */ /* End of group Configuration_of_CMSIS */

#include "core_cm0.h"                               /*!< Cortex-M0 processor and core peripherals                              */
#include "system_GCT301S.h"                         /*!< GCT301S System                                                        */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */


/** @addtogroup Device_Peripheral_Registers
  * @{
  */


/* -------------------  Start of section using anonymous unions  ------------------ */
#if defined(__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined(__ICCARM__)
  #pragma language=extended
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
/* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning 586
#else
  #warning Not supported compiler type
#endif



/* ================================================================================ */
/* ================                     SYSCTRL                    ================ */
/* ================================================================================ */


/**
  * @brief . (SYSCTRL)
  */

typedef struct {                                    /*!< SYSCTRL Structure                                                     */
  
  union {
    __IO uint32_t  SYS_RST_STA;                     /*!< System reset status                                                   */
    
    struct {
      __IO uint32_t  WDT_TIMEOUT:  1;               /*!< Watchdog timeout                                                      */
      __IO uint32_t  CMOLOCKUP  :  1;               /*!< Core lockup                                                           */
    } SYS_RST_STA_b;                                /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  SYS_CLK_STA;                     /*!< System clock status                                                   */
    
    struct {
      __IO uint32_t  DPLL_LOCK  :  1;               /*!< DPLL lock status                                                      */
    } SYS_CLK_STA_b;                                /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  SWD_ERA_KEY;                     /*!< SWD erase key (0xC3A1)                                                */
    
    struct {
      __IO uint32_t  ERA_KEY    : 32;               /*!< ERA_KEY                                                               */
    } SWD_ERA_KEY_b;                                /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  SWD_ERA_CMD;                     /*!< SWD erase command                                                     */
    
    struct {
      __IO uint32_t  CFERA      :  1;               /*!< CFERA                                                                 */
      __IO uint32_t  DFERA      :  1;               /*!< DFERA                                                                 */
    } SWD_ERA_CMD_b;                                /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  RST_CR;                          /*!< Reset configuration                                                   */
    
    struct {
      __IO uint32_t  RSTN_KEY   :  8;               /*!< (null)                                                                */
    } RST_CR_b;                                     /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CLK_DIVSEL;                      /*!< .                                                                     */
    
    struct {
      __IO uint32_t  DIVSEL     :  3;               /*!< Clock divider selection                                               */
    } CLK_DIVSEL_b;                                 /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CLK_SRCSEL;                      /*!< .                                                                     */
    
    struct {
      __IO uint32_t  LCDCLKSEL  :  1;               /*!< LCD clock selection                                                   */
      __IO uint32_t  LXTALSEL   :  1;               /*!< HSCLK/LXTAL selection                                                 */
      __IO uint32_t  HXTALSEL   :  1;               /*!< HSRC/HXTAL selection                                                  */
    } CLK_SRCSEL_b;                                 /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  NMI_CTL;                         /*!< NMI control                                                           */
    
    struct {
      __IO uint32_t  NMISEL     :  5;               /*!< (null)                                                                */
      __IO uint32_t  NMIEN      :  1;               /*!< (null)                                                                */
    } NMI_CTL_b;                                    /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  HSRC_CTL;                        /*!< HSRC control                                                          */
    
    struct {
           uint32_t             :  1;
      __IO uint32_t  PWRDN      :  1;               /*!< (null)                                                                */
      __IO uint32_t  HTRIM      :  5;               /*!< (null)                                                                */
      __IO uint32_t  HTRIM_EN   :  1;               /*!< (null)                                                                */
    } HSRC_CTL_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  LSRC_CTL;                        /*!< LSRC control                                                          */
    
    struct {
      __IO uint32_t  LSRC_ENB   :  1;               /*!< (null)                                                                */
      __IO uint32_t  VREF_ENB   :  1;               /*!< (null)                                                                */
      __IO uint32_t  LTRIM      :  4;               /*!< (null)                                                                */
      __IO uint32_t  LTRIM_EN   :  1;               /*!< (null)                                                                */
    } LSRC_CTL_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  PWR_CTL;                         /*!< Power control                                                         */
    
    struct {
      __IO uint32_t  VRACTIVE   :  1;               /*!< (null)                                                                */
      __IO uint32_t  SRAMPS_EN  :  1;               /*!< (null)                                                                */
      __IO uint32_t  ADCPS_EN   :  1;               /*!< (null)                                                                */
    } PWR_CTL_b;                                    /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  DPLL_CTL;                        /*!< DPLL control                                                          */
    
    struct {
      __IO uint32_t  DPLL_EN    :  1;               /*!< (null)                                                                */
    } DPLL_CTL_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __I  uint32_t  CHIP_CFG;                        /*!< Chip configuration                                                    */
    
    struct {
      __I  uint32_t  LXTAL_CFG  :  2;               /*!< (null)                                                                */
      __I  uint32_t  BOND_A_SEL :  1;               /*!< (null)                                                                */
      __I  uint32_t  BOND_B_SEL :  1;               /*!< (null)                                                                */
      __I  uint32_t  OSC32K_ADJ :  3;               /*!< (null)                                                                */
      __I  uint32_t  LSRC_TRIM  :  4;               /*!< (null)                                                                */
      __I  uint32_t  HSRC_TRIM  :  5;               /*!< (null)                                                                */
    } CHIP_CFG_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __I  uint32_t  SYS_CPL;                         /*!< System code protection                                                */
    
    struct {
      __I  uint32_t  CPL        :  2;               /*!< (null)                                                                */
    } SYS_CPL_b;                                    /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  HXTAL_CTL;                       /*!< HXTAL control                                                         */
    
    struct {
      __IO uint32_t  HXTAL_EN   :  1;               /*!< (null)                                                                */
    } HXTAL_CTL_b;                                  /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  RTC_CTL;                         /*!< RTC control                                                           */
    
    struct {
      __IO uint32_t  IGN_RSTN   :  1;               /*!< (null)                                                                */
      __IO uint32_t  SW_RESET_RTC:  1;              /*!< (null)                                                                */
      __IO uint32_t  PD_LXTAL   :  1;               /*!< (null)                                                                */
    } RTC_CTL_b;                                    /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  WDT_CTL;                         /*!< Watchdog trigger control                                              */
    
    struct {
      __IO uint32_t  WDT_TRIG   :  1;               /*!< (null)                                                                */
    } WDT_CTL_b;                                    /*!< BitSize                                                               */
  };
} SYSCTRL_Type;


/* ================================================================================ */
/* ================                       WDT                      ================ */
/* ================================================================================ */


/**
  * @brief Watchdog (WDT)
  */

typedef struct {                                    /*!< WDT Structure                                                         */
  
  union {
    __I  uint32_t  CFG;                             /*!< Watchdog configuration                                                */
    
    struct {
      __I  uint32_t  ON         :  1;               /*!< (null)                                                                */
      __I  uint32_t  LOAD_SEL   :  3;               /*!< (null)                                                                */
      __I  uint32_t  WARN_EN    :  1;               /*!< (null)                                                                */
      __I  uint32_t  LPM_EN     :  1;               /*!< (null)                                                                */
      __I  uint32_t  DBG_HALT   :  1;               /*!< (null)                                                                */
    } CFG_b;                                        /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  IF;                              /*!< Interrupt flag                                                        */
    
    struct {
      __IO uint32_t  WARN_IF    :  1;               /*!< (null)                                                                */
    } IF_b;                                         /*!< BitSize                                                               */
  };
  __O  uint32_t  FEED;                              /*!< Watchdog feed command, write 0xBB66AA55 to feed.                      */
  __I  uint32_t  LOAD;                              /*!< Watchdog timeout reload selection.                                    */
  __IO uint32_t  COUNT;                             /*!< Watchdog latched count value.                                         */
  
  union {
    __IO uint32_t  LATCH;                           /*!< Update COUNT register.                                                */
    
    struct {
      __IO uint32_t  LATCH_EN   :  1;               /*!< (null)                                                                */
    } LATCH_b;                                      /*!< BitSize                                                               */
  };
} WDT_Type;


/* ================================================================================ */
/* ================                       RTC                      ================ */
/* ================================================================================ */


/**
  * @brief RTC (RTC)
  */

typedef struct {                                    /*!< RTC Structure                                                         */
  
  union {
    __IO uint32_t  CLOCK;                           /*!< Clock                                                                 */
    
    struct {
      __IO uint32_t  SEC        :  7;               /*!< (null)                                                                */
           uint32_t             :  1;
      __IO uint32_t  MIN        :  7;               /*!< (null)                                                                */
           uint32_t             :  1;
      __IO uint32_t  HOUR       :  6;               /*!< (null)                                                                */
    } CLOCK_b;                                      /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CALENDAR;                        /*!< Calendar                                                              */
    
    struct {
      __IO uint32_t  DAY        :  6;               /*!< (null)                                                                */
           uint32_t             :  2;
      __IO uint32_t  MONTH      :  5;               /*!< (null)                                                                */
           uint32_t             :  3;
      __IO uint32_t  YEAR       :  8;               /*!< (null)                                                                */
      __IO uint32_t  WEEK       :  3;               /*!< (null)                                                                */
    } CALENDAR_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __I  uint32_t  STATUS;                          /*!< Status                                                                */
    
    struct {
      __I  uint32_t  VALIDITY   :  1;               /*!< (null)                                                                */
    } STATUS_b;                                     /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  LOAD;                            /*!< Load                                                                  */
    
    struct {
      __IO uint32_t  LOAD_EN    :  1;               /*!< (null)                                                                */
    } LOAD_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  LATCH;                           /*!< Latch                                                                 */
    
    struct {
      __IO uint32_t  LATCH_EN   :  1;               /*!< (null)                                                                */
    } LATCH_b;                                      /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  ACSEL;                           /*!< Adjust cycle selection                                                */
    
    struct {
      __IO uint32_t  AC         :  2;               /*!< (null)                                                                */
    } ACSEL_b;                                      /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  ADJUST;                          /*!< Adjustment amount                                                     */
    
    struct {
      __IO uint32_t  ADJ        :  9;               /*!< (null)                                                                */
    } ADJUST_b;                                     /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  ALARMCTL;                        /*!< Alarm control                                                         */
    
    struct {
      __IO uint32_t  AM         :  4;               /*!< (null)                                                                */
      __IO uint32_t  DW         :  1;               /*!< (null)                                                                */
    } ALARMCTL_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  ALARM;                           /*!< Alarm                                                                 */
    
    struct {
      __IO uint32_t  SEC        :  7;               /*!< (null)                                                                */
           uint32_t             :  1;
      __IO uint32_t  MIN        :  7;               /*!< (null)                                                                */
           uint32_t             :  1;
      __IO uint32_t  HOUR       :  6;               /*!< (null)                                                                */
           uint32_t             :  2;
      __IO uint32_t  DAY        :  7;               /*!< (null)                                                                */
    } ALARM_b;                                      /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  IE;                              /*!< Interrupt enable                                                      */
    
    struct {
      __IO uint32_t  SEC        :  1;               /*!< (null)                                                                */
      __IO uint32_t  MIN        :  1;               /*!< (null)                                                                */
      __IO uint32_t  HOUR       :  1;               /*!< (null)                                                                */
      __IO uint32_t  DAY        :  1;               /*!< (null)                                                                */
      __IO uint32_t  ALM        :  1;               /*!< (null)                                                                */
      __IO uint32_t  TM0        :  1;               /*!< (null)                                                                */
      __IO uint32_t  TM1        :  1;               /*!< (null)                                                                */
      __IO uint32_t  XSTP       :  1;               /*!< (null)                                                                */
      __IO uint32_t  ADJ        :  1;               /*!< (null)                                                                */
    } IE_b;                                         /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  IF;                              /*!< Interrupt flag                                                        */
    
    struct {
      __IO uint32_t  SEC        :  1;               /*!< (null)                                                                */
      __IO uint32_t  MIN        :  1;               /*!< (null)                                                                */
      __IO uint32_t  HOUR       :  1;               /*!< (null)                                                                */
      __IO uint32_t  DAY        :  1;               /*!< (null)                                                                */
      __IO uint32_t  ALM        :  1;               /*!< (null)                                                                */
      __IO uint32_t  TM0        :  1;               /*!< (null)                                                                */
      __IO uint32_t  TM1        :  1;               /*!< (null)                                                                */
      __IO uint32_t  XSTP       :  1;               /*!< (null)                                                                */
      __IO uint32_t  ADJ        :  1;               /*!< (null)                                                                */
    } IF_b;                                         /*!< BitSize                                                               */
  };
  __I  uint32_t  RESERVED;
  
  union {
    __IO uint32_t  CTL;                             /*!< Control                                                               */
    
    struct {
      __IO uint32_t  TM0EN      :  1;               /*!< (null)                                                                */
      __IO uint32_t  TM1EN      :  1;               /*!< (null)                                                                */
      __IO uint32_t  ALMEN      :  1;               /*!< (null)                                                                */
    } CTL_b;                                        /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  TMR0;                            /*!< Timer 0 initial value                                                 */
    
    struct {
      __IO uint32_t  INI        :  8;               /*!< (null)                                                                */
    } TMR0_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  TMR1;                            /*!< Timer 1 initial value                                                 */
    
    struct {
      __IO uint32_t  INI        :  8;               /*!< (null)                                                                */
    } TMR1_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  TMRSEL;                          /*!< Timer polarity                                                        */
    
    struct {
      __IO uint32_t  TMR0POL    :  1;               /*!< (null)                                                                */
      __IO uint32_t  TMR1POL    :  1;               /*!< (null)                                                                */
    } TMRSEL_b;                                     /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  TMRSRC;                          /*!< Timer clock source                                                    */
    
    struct {
      __IO uint32_t  TMR0SRC    :  4;               /*!< (null)                                                                */
      __IO uint32_t  TMR1SRC    :  4;               /*!< (null)                                                                */
    } TMRSRC_b;                                     /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  FOUT;                            /*!< FOUT source                                                           */
    
    struct {
      __IO uint32_t  FOUT0SRC   :  4;               /*!< (null)                                                                */
      __IO uint32_t  FOUT1SRC   :  4;               /*!< (null)                                                                */
    } FOUT_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  TRACK;                           /*!< Tracking control                                                      */
    
    struct {
      __IO uint32_t  TRACK      :  1;               /*!< (null)                                                                */
      __IO uint32_t  TDSEL      :  1;               /*!< (null)                                                                */
    } TRACK_b;                                      /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  TCNT;                            /*!< Track count                                                           */
    
    struct {
      __IO uint32_t  CNT        : 21;               /*!< (null)                                                                */
    } TCNT_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  FDIV;                            /*!< 16MHz clock adjustment                                                */
    
    struct {
      __IO uint32_t  DIV        : 12;               /*!< (null)                                                                */
    } FDIV_b;                                       /*!< BitSize                                                               */
  };
  __IO uint32_t  RAM[6];                            /*!< RAM                                                                   */
} RTC_Type;


/* ================================================================================ */
/* ================                       SPI                      ================ */
/* ================================================================================ */


/**
  * @brief SPI (SPI)
  */

typedef struct {                                    /*!< SPI Structure                                                         */
  __IO uint32_t  DATA;                              /*!< Data                                                                  */
  
  union {
    __I  uint32_t  STATE;                           /*!< State                                                                 */
    
    struct {
      __IO uint32_t  IF         :  1;               /*!< (null)                                                                */
      __I  uint32_t  TIP        :  1;               /*!< (null)                                                                */
    } STATE_b;                                      /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CTRL;                            /*!< Control                                                               */
    
    struct {
      __IO uint32_t  CPHA       :  1;               /*!< (null)                                                                */
      __IO uint32_t  CPOL       :  1;               /*!< (null)                                                                */
      __IO uint32_t  EN         :  1;               /*!< (null)                                                                */
      __IO uint32_t  IE         :  1;               /*!< (null)                                                                */
      __IO uint32_t  BC         :  2;               /*!< (null)                                                                */
      __IO uint32_t  FSB        :  1;               /*!< (null)                                                                */
      __IO uint32_t  SS         :  4;               /*!< (null)                                                                */
    } CTRL_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CRSEL;                           /*!< Clock rate selection                                                  */
    
    struct {
      __IO uint32_t  CR         :  3;               /*!< (null)                                                                */
    } CRSEL_b;                                      /*!< BitSize                                                               */
  };
} SPI_Type;


/* ================================================================================ */
/* ================                       ADC                      ================ */
/* ================================================================================ */


/**
  * @brief ADC (ADC)
  */

typedef struct {                                    /*!< ADC Structure                                                         */
  
  union {
    __IO uint32_t  CTL;                             /*!< Control                                                               */
    
    struct {
      __IO uint32_t  CHSEL      : 14;               /*!< (null)                                                                */
      __IO uint32_t  CLKDIV     :  2;               /*!< (null)                                                                */
      __IO uint32_t  EDGE       :  4;               /*!< (null)                                                                */
      __IO uint32_t  MOD        :  2;               /*!< (null)                                                                */
      __IO uint32_t  IE         :  1;               /*!< (null)                                                                */
    } CTL_b;                                        /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  EM;                              /*!< Mask bits for external events                                         */
    
    struct {
      __IO uint32_t  EM0        :  1;               /*!< (null)                                                                */
      __IO uint32_t  EM1        :  1;               /*!< (null)                                                                */
      __IO uint32_t  EM2        :  1;               /*!< (null)                                                                */
      __IO uint32_t  EM3        :  1;               /*!< (null)                                                                */
    } EM_b;                                         /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  START;                           /*!< Start ADC conversion                                                  */
    
    struct {
      __IO uint32_t  ON         :  1;               /*!< (null)                                                                */
    } START_b;                                      /*!< BitSize                                                               */
  };
  
  union {
    __I  uint32_t  STA;                             /*!< ADC conversion status                                                 */
    
    struct {
      __I  uint32_t  DONE       :  1;               /*!< (null)                                                                */
    } STA_b;                                        /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  TH;                              /*!< Threshold values                                                      */
    
    struct {
      __IO uint32_t  CHSEL      :  4;               /*!< (null)                                                                */
      __IO uint32_t  LOW        : 12;               /*!< (null)                                                                */
      __IO uint32_t  HIGH       : 12;               /*!< (null)                                                                */
    } TH_b;                                         /*!< BitSize                                                               */
  };
  __IO uint32_t  DR[14];                            /*!< ADC data                                                              */
} ADC_Type;


/* ================================================================================ */
/* ================                       LCD                      ================ */
/* ================================================================================ */


/**
  * @brief LCD controller (LCD)
  */

typedef struct {                                    /*!< LCD Structure                                                         */
  
  union {
    __IO uint32_t  CFG_FRQ;                         /*!< Clock prescaler and divider configuration                             */
    
    struct {
      __IO uint32_t  PS         :  2;               /*!< (null)                                                                */
      __IO uint32_t  DIV        :  4;               /*!< (null)                                                                */
      __IO uint32_t  BF         :  3;               /*!< (null)                                                                */
    } CFG_FRQ_b;                                    /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CFG_DRV;                         /*!< Driver configuration                                                  */
    
    struct {
      __IO uint32_t  BIAS       :  1;               /*!< (null)                                                                */
      __IO uint32_t  DUTY       :  1;               /*!< (null)                                                                */
      __IO uint32_t  CC         :  2;               /*!< (null)                                                                */
           uint32_t             :  1;
      __IO uint32_t  HD         :  1;               /*!< (null)                                                                */
      __IO uint32_t  PON        :  3;               /*!< (null)                                                                */
    } CFG_DRV_b;                                    /*!< BitSize                                                               */
  };
  
  union {
    __I  uint32_t  IRQ;                             /*!< Interrupt status flag                                                 */
    
    struct {
      __I  uint32_t  SOF        :  1;               /*!< (null)                                                                */
    } IRQ_b;                                        /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CTL;                             /*!< Control                                                               */
    
    struct {
      __IO uint32_t  LCDEN      :  1;               /*!< (null)                                                                */
      __IO uint32_t  SOFC       :  1;               /*!< (null)                                                                */
      __IO uint32_t  SOFIE      :  1;               /*!< (null)                                                                */
      __IO uint32_t  BLINKEN    :  1;               /*!< (null)                                                                */
    } CTL_b;                                        /*!< BitSize                                                               */
  };
  __IO uint32_t  SEGM;                              /*!< Segment mask                                                          */
  __IO uint32_t  BUF[8];                            /*!< Display buffers                                                       */
} LCD_Type;


/* ================================================================================ */
/* ================                      UART0                     ================ */
/* ================================================================================ */


/**
  * @brief UART 0 (UART0)
  */

typedef struct {                                    /*!< UART0 Structure                                                       */
  __IO uint32_t  DATA;                              /*!< Data buffer                                                           */
  
  union {
    __IO uint32_t  STATE;                           /*!< State                                                                 */
    
    struct {
      __I  uint32_t  TXE        :  1;               /*!< (null)                                                                */
      __I  uint32_t  RXF        :  1;               /*!< (null)                                                                */
      __IO uint32_t  RXO        :  1;               /*!< (null)                                                                */
      __IO uint32_t  FE         :  1;               /*!< (null)                                                                */
      __IO uint32_t  PE         :  1;               /*!< (null)                                                                */
      __I  uint32_t  TC         :  1;               /*!< (null)                                                                */
      __IO uint32_t  NACK       :  1;               /*!< (null)                                                                */
    } STATE_b;                                      /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  CTRL;                            /*!< Control                                                               */
    
    struct {
      __IO uint32_t  TX_IE      :  1;               /*!< (null)                                                                */
      __IO uint32_t  RX_IE      :  1;               /*!< (null)                                                                */
      __IO uint32_t  TC_IE      :  1;               /*!< (null)                                                                */
      __IO uint32_t  TXPOL      :  1;               /*!< (null)                                                                */
      __IO uint32_t  RXPOL      :  1;               /*!< (null)                                                                */
      __IO uint32_t  DL         :  1;               /*!< (null)                                                                */
      __IO uint32_t  SL         :  1;               /*!< (null)                                                                */
      __IO uint32_t  PTY        :  1;               /*!< (null)                                                                */
      __IO uint32_t  PEN        :  1;               /*!< (null)                                                                */
      __IO uint32_t  IRPOL      :  1;               /*!< (null)                                                                */
      __IO uint32_t  IREN       :  1;               /*!< (null)                                                                */
      __IO uint32_t  MODE       :  1;               /*!< (null)                                                                */
      __IO uint32_t  LB         :  1;               /*!< (null)                                                                */
    } CTRL_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  IF;                              /*!< Interrupt flag                                                        */
    
    struct {
      __IO uint32_t  TX_IF      :  1;               /*!< (null)                                                                */
      __IO uint32_t  RX_IF      :  1;               /*!< (null)                                                                */
      __IO uint32_t  TC_IF      :  1;               /*!< (null)                                                                */
    } IF_b;                                         /*!< BitSize                                                               */
  };
  __IO uint32_t  BAUD;                              /*!< Baud rate control                                                     */
  
  union {
    __IO uint32_t  SCCTRL;                          /*!< Smartcard mode control                                                */
    
    struct {
      __IO uint32_t  EGT        :  8;               /*!< (null)                                                                */
      __IO uint32_t  SCM        :  1;               /*!< (null)                                                                */
      __IO uint32_t  SCCLK_DIV  :  3;               /*!< (null)                                                                */
    } SCCTRL_b;                                     /*!< BitSize                                                               */
  };
  __IO uint32_t  SCCLKEN;                           /*!< Smartcard clock enable                                                */
  __IO uint32_t  CLKEN;                             /*!< UART clock enable                                                     */
  
  union {
    __IO uint32_t  ENABLE;                          /*!< UART enable                                                           */
    
    struct {
      __IO uint32_t  TXEN       :  1;               /*!< (null)                                                                */
      __IO uint32_t  RXEN       :  1;               /*!< (null)                                                                */
    } ENABLE_b;                                     /*!< BitSize                                                               */
  };
} UART0_Type;


/* ================================================================================ */
/* ================                     CFLASH                     ================ */
/* ================================================================================ */


/**
  * @brief Code Flash controller (CFLASH)
  */

typedef struct {                                    /*!< CFLASH Structure                                                      */
  __IO uint32_t  KEY;                               /*!< Erase/Program key                                                     */
  __IO uint32_t  NVRP;                              /*!< NVR protection                                                        */
  
  union {
    __IO uint32_t  ERASCTR;                         /*!< Sector erase index                                                    */
    
    struct {
      __IO uint32_t  SCTERAI    :  9;               /*!< (null)                                                                */
    } ERASCTR_b;                                    /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  ERA;                             /*!< Erase control                                                         */
    
    struct {
      __IO uint32_t  MERA       :  1;               /*!< (null)                                                                */
      __IO uint32_t  CERA       :  1;               /*!< (null)                                                                */
      __IO uint32_t  NVRERA     :  1;               /*!< (null)                                                                */
           uint32_t             :  5;
      __IO uint32_t  ERARETRY   :  1;               /*!< (null)                                                                */
    } ERA_b;                                        /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  PROGADDR;                        /*!< Byte address to program                                               */
    
    struct {
      __IO uint32_t  PROGRADDR  : 17;               /*!< (null)                                                                */
    } PROGADDR_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  PROGDATA;                        /*!< Byte data to program                                                  */
    
    struct {
      __IO uint32_t  PROGDATA   :  8;               /*!< (null)                                                                */
    } PROGDATA_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  PROG;                            /*!< Program control                                                       */
    
    struct {
      __IO uint32_t  MPROG      :  1;               /*!< (null)                                                                */
      __IO uint32_t  NVRPROG    :  1;               /*!< (null)                                                                */
    } PROG_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  IE;                              /*!< Interrupt enable                                                      */
    
    struct {
      __IO uint32_t  FIE        :  1;               /*!< (null)                                                                */
    } IE_b;                                         /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  IF;                              /*!< Status flag                                                           */
    
    struct {
      __IO uint32_t  FIF        :  1;               /*!< (null)                                                                */
    } IF_b;                                         /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  TIME;                            /*!< Time offset                                                           */
    
    struct {
      __IO uint32_t  OFFSET     :  2;               /*!< (null)                                                                */
      __IO uint32_t  MODE       :  1;               /*!< (null)                                                                */
    } TIME_b;                                       /*!< BitSize                                                               */
  };
} CFLASH_Type;


/* ================================================================================ */
/* ================                     TIMER0                     ================ */
/* ================================================================================ */


/**
  * @brief TIMER 0 (TIMER0)
  */

typedef struct {                                    /*!< TIMER0 Structure                                                      */
  
  union {
    __IO uint32_t  CTRL;                            /*!< Control                                                               */
    
    struct {
      __IO uint32_t  CEN        :  1;               /*!< (null)                                                                */
      __IO uint32_t  CRST       :  1;               /*!< (null)                                                                */
      __IO uint32_t  SRC        :  1;               /*!< (null)                                                                */
    } CTRL_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  IF;                              /*!< Interrupt status flag                                                 */
    
    struct {
      __IO uint32_t  COMR0      :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR1      :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR2      :  1;               /*!< (null)                                                                */
      __IO uint32_t  CAPR0      :  1;               /*!< (null)                                                                */
      __IO uint32_t  CAPR1      :  1;               /*!< (null)                                                                */
    } IF_b;                                         /*!< BitSize                                                               */
  };
  __IO uint32_t  CNT;                               /*!< Count                                                                 */
  __IO uint32_t  PR;                                /*!< Prescale                                                              */
  __IO uint32_t  PS;                                /*!< Prescale counter                                                      */
  
  union {
    __IO uint32_t  COMCR;                           /*!< Compare control                                                       */
    
    struct {
      __IO uint32_t  COMR0_IE   :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR0_RST  :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR0_STP  :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR1_IE   :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR1_RST  :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR1_STP  :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR2_IE   :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR2_RST  :  1;               /*!< (null)                                                                */
      __IO uint32_t  COMR2_STP  :  1;               /*!< (null)                                                                */
    } COMCR_b;                                      /*!< BitSize                                                               */
  };
  __IO uint32_t  COMR0;                             /*!< Compare 0                                                             */
  __IO uint32_t  COMR1;                             /*!< Compare 1                                                             */
  __IO uint32_t  COMR2;                             /*!< Compare 2                                                             */
  
  union {
    __IO uint32_t  CAPCR;                           /*!< Capture control                                                       */
    
    struct {
      __IO uint32_t  CAP0_RE    :  1;               /*!< (null)                                                                */
      __IO uint32_t  CAP0_FE    :  1;               /*!< (null)                                                                */
      __IO uint32_t  CAP0_IE    :  1;               /*!< (null)                                                                */
      __IO uint32_t  CAP1_RE    :  1;               /*!< (null)                                                                */
      __IO uint32_t  CAP1_FE    :  1;               /*!< (null)                                                                */
      __IO uint32_t  CAP1_IE    :  1;               /*!< (null)                                                                */
    } CAPCR_b;                                      /*!< BitSize                                                               */
  };
  __IO uint32_t  CAPR0;                             /*!< Capture 0                                                             */
  __IO uint32_t  CAPR1;                             /*!< Capture 1                                                             */
  
  union {
    __IO uint32_t  OCR;                             /*!< Output compare                                                        */
    
    struct {
      __IO uint32_t  OC0        :  1;               /*!< (null)                                                                */
      __IO uint32_t  OCC0       :  2;               /*!< (null)                                                                */
      __IO uint32_t  OC1        :  1;               /*!< (null)                                                                */
      __IO uint32_t  OCC1       :  2;               /*!< (null)                                                                */
      __IO uint32_t  OC2        :  1;               /*!< (null)                                                                */
      __IO uint32_t  OCC2       :  2;               /*!< (null)                                                                */
    } OCR_b;                                        /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  PWMC;                            /*!< PWM control                                                           */
    
    struct {
      __IO uint32_t  PWMEN0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  PWMEN1     :  1;               /*!< (null)                                                                */
    } PWMC_b;                                       /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  DBG;                             /*!< Debug halt                                                            */
    
    struct {
      __IO uint32_t  DBG_HALT   :  1;               /*!< (null)                                                                */
    } DBG_b;                                        /*!< BitSize                                                               */
  };
} TIMER0_Type;


/* ================================================================================ */
/* ================                      GPIOA                     ================ */
/* ================================================================================ */


/**
  * @brief GPIO A (GPIOA)
  */

typedef struct {                                    /*!< GPIOA Structure                                                       */
  
  union {
    __I  uint32_t  DATAIN;                          /*!< Input pin                                                             */
    
    struct {
      __I  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __I  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } DATAIN_b;                                     /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  DATAOUT;                         /*!< Output latch                                                          */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } DATAOUT_b;                                    /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  ANDISSET;                        /*!< Analog IO disable set                                                 */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } ANDISSET_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __O  uint32_t  ANDISCLR;                        /*!< Analog IO disable clear                                               */
    
    struct {
      __O  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } ANDISCLR_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  OUTENSET;                        /*!< Output enable set                                                     */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } OUTENSET_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __O  uint32_t  OUTENCLR;                        /*!< Output enable clear                                                   */
    
    struct {
      __O  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } OUTENCLR_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  ALTFUNCSET;                      /*!< Alternate function enable set                                         */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } ALTFUNCSET_b;                                 /*!< BitSize                                                               */
  };
  
  union {
    __O  uint32_t  ALTFUNCCLR;                      /*!< Alternate function enable clear                                       */
    
    struct {
      __O  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } ALTFUNCCLR_b;                                 /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  INTENSET;                        /*!< Interrupt enable set                                                  */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } INTENSET_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __O  uint32_t  INTENCLR;                        /*!< Interrupt enable clear                                                */
    
    struct {
      __O  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } INTENCLR_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  INTTYPESET;                      /*!< Interrupt type set                                                    */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } INTTYPESET_b;                                 /*!< BitSize                                                               */
  };
  
  union {
    __O  uint32_t  INTTYPECLR;                      /*!< Interrupt type clear                                                  */
    
    struct {
      __O  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } INTTYPECLR_b;                                 /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  INTPOLSET;                       /*!< Interrupt polarity set                                                */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } INTPOLSET_b;                                  /*!< BitSize                                                               */
  };
  
  union {
    __O  uint32_t  INTPOLCLR;                       /*!< Interrupt polarity clear                                              */
    
    struct {
      __O  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } INTPOLCLR_b;                                  /*!< BitSize                                                               */
  };
  
  union {
    union {
      __O  uint32_t  INTCLR;                        /*!< Interrupt status                                                      */
      
      struct {
        __O  uint32_t  port_0   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_1   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_2   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_3   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_4   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_5   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_6   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_7   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_8   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_9   :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_10  :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_11  :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_12  :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_13  :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_14  :  1;               /*!< (null)                                                                */
        __O  uint32_t  port_15  :  1;               /*!< (null)                                                                */
      } INTCLR_b;                                   /*!< BitSize                                                               */
    };
    
    union {
      __I  uint32_t  INTSTATUS;                     /*!< Interrupt status                                                      */
      
      struct {
        __I  uint32_t  port_0   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_1   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_2   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_3   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_4   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_5   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_6   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_7   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_8   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_9   :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_10  :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_11  :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_12  :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_13  :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_14  :  1;               /*!< (null)                                                                */
        __I  uint32_t  port_15  :  1;               /*!< (null)                                                                */
      } INTSTATUS_b;                                /*!< BitSize                                                               */
    };
  };
  __O  uint32_t  BSCR;                              /*!< Bit set/clear                                                         */
  
  union {
    __IO uint32_t  PUDISSET;                        /*!< Pullup disable set                                                    */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } PUDISSET_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __O  uint32_t  PUDISCLR;                        /*!< Pullup disable clear                                                  */
    
    struct {
      __O  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } PUDISCLR_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  PUSELSET;                        /*!< Pullup select set                                                     */
    
    struct {
      __IO uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } PUSELSET_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __O  uint32_t  PUSELCLR;                        /*!< Pullup select clear                                                   */
    
    struct {
      __O  uint32_t  port_0     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_1     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_2     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_3     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_4     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_5     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_6     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_7     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_8     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_9     :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_10    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_11    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_12    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_13    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_14    :  1;               /*!< (null)                                                                */
      __O  uint32_t  port_15    :  1;               /*!< (null)                                                                */
    } PUSELCLR_b;                                   /*!< BitSize                                                               */
  };
  
  union {
    __IO uint32_t  DSCTL;                           /*!< Drive strength control                                                */
    
    struct {
      __IO uint32_t  port_0     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_1     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_2     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_3     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_4     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_5     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_6     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_7     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_8     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_9     :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_10    :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_11    :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_12    :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_13    :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_14    :  2;               /*!< (null)                                                                */
      __IO uint32_t  port_15    :  2;               /*!< (null)                                                                */
    } DSCTL_b;                                      /*!< BitSize                                                               */
  };
  __I  uint32_t  RESERVED[235];
  __IO uint32_t  LB_MASKED[256];                    /*!< Lower byte mask access                                                */
  __IO uint32_t  UB_MASKED[256];                    /*!< Upper byte mask access                                                */
} GPIOA_Type;


/* --------------------  End of section using anonymous unions  ------------------- */
#if defined(__CC_ARM)
  #pragma pop
#elif defined(__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined(__TASKING__)
  #pragma warning restore
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define SYSCTRL_BASE                    0x4001D000UL
#define WDT_BASE                        0x40006000UL
#define RTC_BASE                        0x40000000UL
#define SPI_BASE                        0x4001A000UL
#define ADC_BASE                        0x4000A000UL
#define LCD_BASE                        0x40008000UL
#define UART0_BASE                      0x40010000UL
#define UART1_BASE                      0x40011000UL
#define UART2_BASE                      0x40012000UL
#define UART3_BASE                      0x40013000UL
#define UART4_BASE                      0x40014000UL
#define UART5_BASE                      0x40015000UL
#define CFLASH_BASE                     0x40017000UL
#define DFLASH_BASE                     0x40018000UL
#define TIMER0_BASE                     0x40001000UL
#define TIMER1_BASE                     0x40002000UL
#define TIMER2_BASE                     0x40003000UL
#define TIMER3_BASE                     0x40004000UL
#define TIMER4_BASE                     0x40005000UL
#define GPIOA_BASE                      0x40080000UL
#define GPIOB_BASE                      0x40081000UL
#define GPIOC_BASE                      0x40082000UL


/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define SYSCTRL                         ((SYSCTRL_Type            *) SYSCTRL_BASE)
#define WDT                             ((WDT_Type                *) WDT_BASE)
#define RTC                             ((RTC_Type                *) RTC_BASE)
#define SPI                             ((SPI_Type                *) SPI_BASE)
#define ADC                             ((ADC_Type                *) ADC_BASE)
#define LCD                             ((LCD_Type                *) LCD_BASE)
#define UART0                           ((UART0_Type              *) UART0_BASE)
#define UART1                           ((UART0_Type              *) UART1_BASE)
#define UART2                           ((UART0_Type              *) UART2_BASE)
#define UART3                           ((UART0_Type              *) UART3_BASE)
#define UART4                           ((UART0_Type              *) UART4_BASE)
#define UART5                           ((UART0_Type              *) UART5_BASE)
#define CFLASH                          ((CFLASH_Type             *) CFLASH_BASE)
#define DFLASH                          ((CFLASH_Type             *) DFLASH_BASE)
#define TIMER0                          ((TIMER0_Type             *) TIMER0_BASE)
#define TIMER1                          ((TIMER0_Type             *) TIMER1_BASE)
#define TIMER2                          ((TIMER0_Type             *) TIMER2_BASE)
#define TIMER3                          ((TIMER0_Type             *) TIMER3_BASE)
#define TIMER4                          ((TIMER0_Type             *) TIMER4_BASE)
#define GPIOA                           ((GPIOA_Type              *) GPIOA_BASE)
#define GPIOB                           ((GPIOA_Type              *) GPIOB_BASE)
#define GPIOC                           ((GPIOA_Type              *) GPIOC_BASE)


/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group GCT301S */
/** @} */ /* End of group SZGC */

#ifdef __cplusplus
}
#endif


#endif  /* GCT301S_H */

