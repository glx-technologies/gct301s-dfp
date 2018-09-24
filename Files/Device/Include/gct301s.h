
#ifndef __GCT301S_H
#define __GCT301S_H

#ifdef __cplusplus
 extern "C" {
#endif

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
  NonMaskableInt_IRQn           = -14,    /*   2 Non Maskable Interrupt                           */
  HardFault_IRQn                = -13,    /*   3 Cortex-M0 Hard Fault Interrupt                   */
  SVCall_IRQn                   = -5,     /*   11 Cortex-M0 SV Call Interrupt                     */
  PendSV_IRQn                   = -2,     /*   14 Cortex-M0 Pend SV Interrupt                     */
  SysTick_IRQn                  = -1,     /*   15 Cortex-M0 System Tick Interrupt                 */

/******  GCT301S Specific Interrupt Numbers *******************************************************/
  SYSCTRL_IRQn                  = 0,       /*   System Control Interrupt                        */
  WDT_IRQn                      = 1,       /*   WDT Warn Interrupt                              */
  RTC_IRQn                      = 2,       /*   RTC Interrupt                                   */
  UART0_IRQn                    = 3,       /*   UART 0 Interrupt                                */
  UART1_IRQn                    = 4,       /*   UART 1 Interrupt                                */
  UART2_IRQn                    = 5,       /*   UART 2 Interrupt                                */
  UART3_IRQn                    = 6,       /*   UART 3 Interrupt                                */
  UART4_IRQn                    = 7,       /*   UART 4 Interrupt                                */
  UART5_IRQn                    = 8,       /*   UART 5 Interrupt                                */
  SPI_IRQn                      = 9,       /*   SPI Interrupt                                   */
  CFLASH_IRQn                   = 10,      /*   Code Flash Interrupt                            */
  DFLASH_IRQn                   = 11,      /*   Data Flash Interrupt                            */
  TIMER0_IRQn                   = 12,      /*   Timer 0 Interrupt                               */
  TIMER1_IRQn                   = 13,      /*   Timer 1 Interrupt                               */
  TIMER2_IRQn                   = 14,      /*   Timer 2 Interrupt                               */
  TIMER3_IRQn                   = 15,      /*   Timer 3 Interrupt                               */
  TIMER4_IRQn                   = 16,      /*   Timer 4 Interrupt                               */
  EXT0_IRQn                     = 17,      /*   GPIO 2 Interrupt                                */
  EXT1_IRQn                     = 18,      /*   GPIO 3 Interrupt                                */
  EXT2_IRQn                     = 19,      /*   GPIO 4 Interrupt                                */
  EXT3_IRQn                     = 20,      /*   GPIO 5 Interrupt                                */  
  LCD_IRQn                      = 21,      /*   LCD Interrupt                                   */
  ADC_IRQn                      = 22,      /*   ADC Interrupt                                   */
  GPIOA_IRQn                    = 23,      /*   GPIO 1, 2, 5, 6, ..., 15 OR-ed Interrupt        */
  GPIOB_IRQn                    = 24,      /*   GPIO 16, ..., 31 OR-edInterrupt                 */
  GPIOC_IRQn                    = 25,      /*   GPIO 32, ..., 47 OR-edInterrupt                 */
  IRQ26_IRQn                    = 26,
  IRQ27_IRQn                    = 27,
  IRQ28_IRQn                    = 28,
  IRQ29_IRQn                    = 29,
  IRQ30_IRQn                    = 30,
  SINT_IRQn                     = 31,
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT             0         /*   MPU present or not                               */
#define __NVIC_PRIO_BITS          2         /*   Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*   Set to 1 if different SysTick Config is used     */

/*@}*/ /* end of group GCT301S_CMSIS */


#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_gct301s.h"                 /* System Header                                      */

/******************************************************************************/
/*                Device Specific Peripheral Registers structures             */
/******************************************************************************/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/*-------------------- System Control (SYSCTRL) -------------------*/
typedef struct
{
  __IO uint32_t SYS_RST_STA;
  __I  uint32_t SYS_CLK_STA;
  __IO uint32_t SWD_ERA_KEY;
  __IO uint32_t SWD_ERA_CMD;
  __IO uint32_t RST_CR;
  __IO uint32_t CLK_DIVSEL;
  __IO uint32_t CLK_SRCSEL;
  __IO uint32_t NMI_CTL;
  __IO uint32_t HSRC_CTL;
  __IO uint32_t LSRC_CTL;
  __IO uint32_t PWR_CTL;
  __IO uint32_t DPLL_CTL;
  __I  uint32_t CHIP_CFG;    
  __I  uint32_t SYS_CPL;
  __IO uint32_t HXTAL_CTL;
  __IO uint32_t RTC_CTL;
  __IO uint32_t WDT_CTL;
} SYSCTRL_TypeDef;

typedef struct
{
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t WDT_TIMEOUT : 1;
      __IO uint32_t CM0LOCKUP   : 1;
    } BITS;
  } SYS_RST_STA;
  union {
    __I   uint32_t  VAL;
    struct {
      __I  uint32_t DPLL_LOCK : 1;
    } BITS;
  } SYS_CLK_STA;
  __IO  uint32_t  SWD_ERA_KEY;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t CFERA : 1;
      __IO uint32_t DFERA : 1;
    } BITS;
  } SWD_ERA_CMD;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t RSTN_KEY : 8;
    } BIST;
  } RST_CR;
  union { 
    __IO  uint32_t VAL;
    struct {
      __IO uint32_t DIVSEL : 2;
    } BITS;
  } CLK_DIVSEL;
  union {
    __IO  uint32_t VAL;
    struct {
      __IO uint32_t LCDCLKSEL : 1;
      __IO uint32_t LXTALSEL  : 1;
      __IO uint32_t HXTALSEL  : 1;
    } BITS;
  } CLK_SRCSEL;
  union {
  __IO  uint32_t  VAL;
    struct {
      __IO uint32_t NMISEL : 5;
      __IO uint32_t NMIEN  : 1;
    } BITS;
  } NMI_CTL;
  union {
    __IO  uint32_t VAL;
    struct {
      __IO uint32_t RESERVED0 : 1;
      __IO uint32_t PWRDN     : 1;
      __IO uint32_t HTRIM     : 5;
      __IO uint32_t HTRIM_EN  : 1;
    } BITS;
  } HSRC_CTL;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t LSRC_ENB : 1;
      __IO uint32_t VREF_ENB : 1;
      __IO uint32_t LTRIM    : 4;
      __IO uint32_t LTRIM_EN : 1;
    } BITS;
  } LSRC_CTL;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t VRACTIVE  : 1;
      __IO uint32_t SRAMPS_EN : 1;
      __IO uint32_t ADCPS_EN : 1;
    } BITS;
  } PWR_CTL;
  union {
    __IO uint32_t  VAL;
    struct {
      __IO uint32_t DPLL_EN : 1;
    } BITS;
  } DPLL_CTL;
  union {
    __I  uint32_t VAL;
    struct {
      __I  uint32_t LXTAL_CFG  : 2;
      __I  uint32_t BOND_A_SEL : 1;
      __I  uint32_t BOND_B_SEL : 1;
      __I  uint32_t OSC32K_ADJ : 3;
      __I  uint32_t LSRC_TRIM  : 4;
      __I  uint32_t HSRC_TRIM  : 5;
    } BITS;  
  } CHIP_CFG;    
  union {
    __I   uint32_t  VAL;
    struct {
      __I  uint32_t CPL : 2;
    } BITS;
  } SYS_CPL;
  union {
    __IO  uint32_t VAL;
    struct {
      __IO uint32_t HXTAL_EN : 1;
    } BITS;
  } HXTAL_CTL;
  union {
    __IO  uint32_t VAL;
    struct {
      __IO uint32_t IGN_RSTN     : 1;
      __IO uint32_t SW_RESET_RTC : 1;
      __IO uint32_t PD_LXTAL     : 1;
    } BITS;
  } RTC_CTL;
  union {
    __IO  uint32_t VAL;
    struct {
      __IO uint32_t WDT_TRIG : 1;
    } BITS;
  } WDT_CTL;
} SYSCTRL_REG_TypeDef;

/*-------------------- Real Time Clock (RTC) -------------------*/
typedef struct {
  __IO uint32_t CLOCK;
  __IO uint32_t CALENDAR;
  __I  uint32_t STATUS;
  __IO uint32_t LOAD;
  __IO uint32_t LATCH;
  __IO uint32_t ACSEL;
  __IO uint32_t ADJUST;
  __IO uint32_t ALARMCTL;
  __IO uint32_t ALARM;
  __IO uint32_t IE;
  __IO uint32_t IF;
  __I  uint32_t RESERVED0;
  __IO uint32_t CTL;
  __IO uint32_t TMR0;
  __IO uint32_t TMR1;
  __IO uint32_t TMRSEL;
  __IO uint32_t TMRSRC;
  __IO uint32_t FOUT;
  __IO uint32_t TRACK;
  __I  uint32_t TCNT;
  __IO uint32_t FDIV;
  __IO  uint32_t  RAM[6];
} RTC_TypeDef;

typedef struct {
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t SECOND  : 7;
      __IO uint32_t         : 1;
      __IO uint32_t MINUTE  : 7;
      __IO uint32_t         : 1;
      __IO uint32_t HOUR    : 6;
    } BITS;
  } CLOCK;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t DAY   : 6;
      __IO uint32_t       : 2;
      __IO uint32_t MONTH : 5;
      __IO uint32_t       : 3;
      __IO uint32_t YEAR  : 8;
      __IO uint32_t WEEK  : 3;
    } BITS;
  } CALENDAR;
  union {
    __I  uint32_t VAL;
    struct {
      __I   uint32_t  VALIDITY : 1;
    } BITS;
  } STATUS;
  union {
    __IO uint32_t VAL;
    struct {
      __IO  uint32_t LOAD_EN : 1;
    } BITS;
  } LOAD;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t LATCH_EN;
    } BITS;
  } LATCH;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t AC;
    } BITS;
  } ACSEL;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t ADJ : 9;
    } BITS;
  } ADJUST;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t SEC_MASK  : 1;
      __IO uint32_t MIN_MASK  : 1;
      __IO uint32_t HOUR_MASK : 1;
      __IO uint32_t DAY_MASK  : 1;
      __IO uint32_t DAY_DATE  : 1;
    } BITS;
  } ALARMCTL;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t SEC : 7;
      __IO uint32_t RESERVED0 : 1;
      __IO uint32_t MIN : 7;
      __IO uint32_t RESERVED1 : 1;
      __IO uint32_t HOUR : 6;
    } BITS;
  } ALARM;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t SEC  : 1;
      __IO uint32_t MIN  : 1;
      __IO uint32_t HOUR : 1;
      __IO uint32_t DAY  : 1;
      __IO uint32_t ALM  : 1;
      __IO uint32_t TM0  : 1;
      __IO uint32_t TM1  : 1;
      __IO uint32_t XSTP : 1;
      __IO uint32_t ADJ  : 1;
    } BITS;
  } IE;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t SEC  : 1;
      __IO uint32_t MIN  : 1;
      __IO uint32_t HOUR : 1;
      __IO uint32_t DAY  : 1;
      __IO uint32_t ALM  : 1;
      __IO uint32_t TM0  : 1;
      __IO uint32_t TM1  : 1;
      __IO uint32_t XSTP : 1;
      __IO uint32_t ADJ  : 1;
    } BITS;
  } IF;
  __I  uint32_t  RESERVED0;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t TM0EN : 1;
      __IO uint32_t TM1EN : 1;
      __IO uint32_t ALMEN : 1;
    } BITS;
  } CTL;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t TMR0INI : 8;
    } BITS;
  } TMR0;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t TMR1INI : 8;
    } BITS;
  } TMR1;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t SEL : 2;
    } BITS;
  } TMRSEL;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t TMR0SRC : 4;
      __IO uint32_t TMR1SRC : 4;
    } BITS;
  } TMRSRC;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t FOUT0SRC : 4;
      __IO uint32_t FOUT1SRC : 4;
    } BITS;
  } FOUT;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t START : 1;
      __IO uint32_t TDSEL : 1;
    } BITS;
  } TRACK;
  union {
    __I  uint32_t VAL;
    struct {
      __I  uint32_t CNT : 21;
    } BITS;
  } TCNT;
  union {
    __IO uint32_t VAL;
    struct {
      __IO  uint32_t DIV;
    } BITS;
  } FDIV;
  __IO  uint32_t  RAM[6];
} RTC_REG_TypeDef;


/*-------------------- FLASH Control (FLASH) -------------------*/
typedef struct {
  __O   uint32_t  KEY;
  __IO  uint32_t  NVRP;
  __IO  uint32_t  ERASCTR;
  __IO  uint32_t  ERA;
  __IO  uint32_t  PROGADDR;
  __IO  uint32_t  PROGDATA;
  __IO  uint32_t  PROG;
  __IO  uint32_t  IE;
  __I   uint32_t  IF;
  __IO  uint32_t  TIME;
} FLASH_TypeDef;


/*-------------------- LCD driver (LCD) -------------------*/
typedef struct {
  __IO uint32_t CFG_FRQ;
  __IO uint32_t CFG_DRV;
  __I  uint32_t IRQ;   
  __IO uint32_t CTL;
  __IO uint32_t SEGM;
  __IO uint32_t BUF[8];
} LCD_TypeDef;

typedef struct {
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO  uint32_t PS  : 2;
      __IO  uint32_t DIV : 4;  
      __IO  uint32_t BF  : 3;
    } BITS;
  } CFG_FRQ;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO  uint32_t BIAS : 1;
      __IO  uint32_t DUTY : 1; 
      __IO  uint32_t CC   : 2;
      __IO  uint32_t RESERVED : 1; 
      __IO  uint32_t HD   : 1; 
      __IO  uint32_t PON  : 3;
    } BITS;
  } CFG_DRV;
  __I   uint32_t  IRQ;
  union {  
    __IO  uint32_t  VAL;
    struct {
      __IO  uint32_t LCDEN   : 1;
      __O   uint32_t SOFC    : 1;
      __IO  uint32_t SOFIE   : 1;
      __IO  uint32_t BLINKEN : 1;
    } BITS;
  } CTL;
  __IO  uint32_t  SEGM;
  __IO  uint32_t  BUF[8];
} LCD_REG_TypeDef;

/*-------------------- Analog-to-Digital Converter (ADC) ------*/
typedef struct {
  __IO uint32_t CTL;
  __IO uint32_t EM;
  __IO uint32_t START;
  __I  uint32_t STA;
  __IO uint32_t TH;
  __IO uint32_t DR[14];
} ADC_TypeDef;

typedef struct {
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t CHSEL   : 14;
      __IO uint32_t CLKDIV  : 2;
      __IO uint32_t EDGE    : 4;
      __IO uint32_t MOD     : 2;
      __IO uint32_t IE      : 1;
    } BITS;
  } CTL;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t EM0 : 1;
      __IO uint32_t EM1 : 1;
      __IO uint32_t EM2 : 1;
      __IO uint32_t EM3 : 1;
    } BITS;
  } EM;
  union {
    __IO uint32_t VAL;
    struct {
      __IO  uint32_t  ON : 1;
    } BITS;
  } START;
  union {
    __I   uint32_t  VAL;
    struct {
      __I  uint32_t DONE : 1;
    } BITS;
  } STA;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t CHSEL : 4;
      __IO uint32_t LOW   : 12;
      __IO uint32_t HIGH  : 12;
    } BITS;
  } TH;
  __IO  uint32_t  DR[14];
} ADC_REG_TypeDef;

/*-------------------- TIMER (TIMER) -------------------*/
typedef struct {
  __IO uint32_t CTRL;
  __IO uint32_t IF;
  __IO uint32_t CNT;
  __IO uint32_t PR;
  __IO uint32_t PS;
  __IO uint32_t COMCR;
  __IO uint32_t COMR0;
  __IO uint32_t COMR1;
  __IO uint32_t COMR2;
  __IO uint32_t CAPCR;
	__IO uint32_t CAPR0;
  __IO uint32_t CAPR1;
  __IO uint32_t OCR;
  __IO uint32_t PWMC;
  __IO uint32_t DBG; 
} TIMER_TypeDef;

typedef struct {
	union {
		__IO  uint32_t  VAL;
		struct {
			__IO uint32_t CEN   : 1;
			__IO uint32_t CRST 	: 1;
			__IO uint32_t SRC 	: 1;
		} BITS;
  } CTRL;
	union {
    __IO  uint32_t VAL;
		struct {
			__IO uint32_t COMR0_IF : 1;
			__IO uint32_t COMR1_IF : 1;
			__IO uint32_t COMR2_IF : 1;
			__IO uint32_t CAPR0_IF : 1;
			__IO uint32_t CAPR1_IF : 1;
		} BITS;
	} IF;
	__IO  uint32_t  CNT;
  __IO  uint32_t  PR;
  __IO  uint32_t  PS;
	union {
		__IO  uint32_t  VAL;
		struct {
			__IO uint32_t COMR0_IE	: 1;
			__IO uint32_t COMR0_RST	: 1;
			__IO uint32_t COMR0_STP : 1;
			__IO uint32_t COMR1_IE	: 1;
			__IO uint32_t COMR1_RST	: 1;
			__IO uint32_t COMR1_STP : 1;
			__IO uint32_t COMR2_IE	: 1;
			__IO uint32_t COMR2_RST	: 1;
			__IO uint32_t COMR2_STP : 1;
		} BITS;
	} COMCR;
	__IO  uint32_t  COMR0;
  __IO  uint32_t  COMR1;
  __IO  uint32_t  COMR2;
	union {
		__IO  uint32_t  VAL;
		struct {
			__IO uint32_t CAP0_RE : 1;
			__IO uint32_t CAP0_FE : 1;
			__IO uint32_t CAP0_IE : 1;
			__IO uint32_t CAP1_RE : 1;
			__IO uint32_t CAP1_FE : 1;
			__IO uint32_t CAP1_IE : 1;
		} ;
	} CAPCR;
	__IO  uint32_t  CAPR0;
  __IO  uint32_t  CAPR1;
	union {
		__IO  uint32_t  VAL;
		struct {
		  __IO uint32_t OC0		: 1;
			__IO uint32_t OCC0  : 2;
      __IO uint32_t OC1   : 1;
      __IO uint32_t OCC1  : 2;
      __IO uint32_t OC2   : 1;
      __IO uint32_t OCC2  : 2;
		} BITS;
	} OCR;
  union {
    __IO  uint32_t  VAL;    
    struct {
      __IO uint32_t PWMEN0 : 1;
      __IO uint32_t PWMEN1 : 1;
    } BITS;
  } PWMC;
  union {
    __IO  uint32_t  VAL; 
    struct {
      __IO uint32_t DBG_HALT : 1;
    } BITS;
  } DBG;
} TIMER_REG_TypeDef;

/*-------------------- Watchdog Timer (WDT) -------------------*/
typedef struct {
  __I  uint32_t CFG;
  __IO uint32_t IF;
  __O  uint32_t FEED;   
  __I  uint32_t LOAD;   
  __I  uint32_t COUNT;   
  __IO uint32_t LATCH;   
} WDT_TypeDef;

typedef struct {
  union {
    __I   uint32_t  VAL;
    struct {
      __I  uint32_t ON				:1;
      __I  uint32_t LOAD_SEL	:3;
      __I  uint32_t WARN_EN		:1;
      __I  uint32_t LPM_EN		:1;
      __I  uint32_t DBG_HALT	:1;
    } BITS;
  } CFG;
  __IO  uint32_t  IF;
  __O   uint32_t  FEED;   
  __I   uint32_t  LOAD;   
  __I   uint32_t  COUNT;   
  __IO  uint32_t  LATCH;   
} WDT_REG_TypeDef;


/*------------- Universal Asynchronous Receiver Transmitter (UART) -----------*/
typedef struct {
  __IO uint32_t DATA;
  __IO uint32_t STATE;
  __IO uint32_t CTRL;
  __IO uint32_t IF;
  __IO uint32_t BAUD;
  __IO uint32_t SCCTRL;
  __IO uint32_t SCCLKEN;
  __IO uint32_t CLKEN;
  __IO uint32_t ENABLE;
} UART_TypeDef;

typedef struct {
  __IO  uint32_t  DATA;
  union {
    __IO  uint32_t  VAL;
    struct {
      __I  uint32_t TXE :1;
      __I  uint32_t RXF :1;
      __IO uint32_t RXO :1;
      __IO uint32_t FE  :1;
      __IO uint32_t PE  :1;
      __I  uint32_t TC  :1;
      __I  uint32_t NACK:1;
    } BITS;
  } STATE;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t TX_IE :1;
      __IO uint32_t RX_IE :1;
      __IO uint32_t TC_IE :1;
      __IO uint32_t TXPOL :1;
      __IO uint32_t RXPOL :1;
      __IO uint32_t DL    :1;
      __IO uint32_t SL    :1;
      __IO uint32_t PTY   :1;
      __IO uint32_t PEN   :1;
      __IO uint32_t IRPOL :1;
      __IO uint32_t IREN  :1;
      __IO uint32_t MODE  :1;
      __IO uint32_t LB    :1;
    } BITS;
  } CTRL;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t TX_IF:1;
      __IO uint32_t RX_IF:1;
      __IO uint32_t TC_IF:1;
    } BITS;
  } IF;
  __IO  uint32_t  BAUD;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t EGT       :8;
      __IO uint32_t SCM       :1;
      __IO uint32_t SCCLK_DIV :3;
    } BITS;
  } SCCTRL;
  __IO  uint32_t  SCCLKEN;
  __IO  uint32_t  CLKEN;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t TXEN:1;
      __IO uint32_t RXEN:1;
    } BITS;
  } ENABLE;
} UART_REG_TypeDef;

/*-------------------- Serial Peripheral Interface (SPI) -------------------*/
typedef struct {
  __IO uint32_t DATA;
  __IO uint32_t STATE;
  __IO uint32_t CTRL;
  __IO uint32_t CRSEL;
} SPI_TypeDef;

typedef struct {
  __IO uint32_t DATA;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t IF  :1;
      __IO uint32_t TIP :1;
    } BITS;
  } STATE;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t CPHA:1;
      __IO uint32_t CPOL:1;
      __IO uint32_t EN  :1;
      __IO uint32_t BC  :2;
      __IO uint32_t FSB :1;
      __IO uint32_t SS  :4;
    } BITS;
  } CTRL;
  union {
    __IO uint32_t VAL;
    struct {
      __IO uint32_t CRSEL : 3;
    } BITS;
  } CRSEL;
} SPI_REG_TypeDef;


/*-------------------- General Purpose Input Output (GPIO) -------------------*/
typedef struct
{
  __I    uint32_t  DATAIN;           /*   Offset: 0x000 DATAIN Register (R/) */
  __IO   uint32_t  DATAOUT;          /*   Offset: 0x004 Data Output Latch Register (R/W) */
  __IO   uint32_t  ANDISSET;         /*   Offset: 0x008 Analog IO Disable Set Register (R/W) */
  __IO   uint32_t  ANDISCLR;         /*   Offset: 0x00C Analog IO Disable Clear Register (R/W) */
  __IO   uint32_t  OUTENSET;         /*   Offset: 0x010 Output Enable Set Register  (R/W) */
  __IO   uint32_t  OUTENCLR;         /*   Offset: 0x014 Output Enable Clear Register  (R/W) */
  __IO   uint32_t  ALTFUNCSET;       /*   Offset: 0x018 Alternate Function Set Register  (R/W) */
  __IO   uint32_t  ALTFUNCCLR;       /*   Offset: 0x01C Alternate Function Clear Register  (R/W) */
  __IO   uint32_t  INTENSET;         /*   Offset: 0x020 Interrupt Enable Set Register  (R/W) */
  __IO   uint32_t  INTENCLR;         /*   Offset: 0x024 Interrupt Enable Clear Register  (R/W) */
  __IO   uint32_t  INTTYPESET;       /*   Offset: 0x028 Interrupt Type Set Register  (R/W) */
  __IO   uint32_t  INTTYPECLR;       /*   Offset: 0x02C Interrupt Type Clear Register  (R/W) */
  __IO   uint32_t  INTPOLSET;        /*   Offset: 0x030 Interrupt Polarity Set Register  (R/W) */
  __IO   uint32_t  INTPOLCLR;        /*   Offset: 0x034 Interrupt Polarity Clear Register  (R/W) */
  union {
    __I    uint32_t  INTSTATUS;      /*   Offset: 0x038 Interrupt Status Register (R/ ) */
    __O    uint32_t  INTCLR;         /*   Offset: 0x038 Interrupt Clear Register ( /W) */
    };
  __O    uint32_t  BSCR;             /*   Offset: 0x03C Bit Set/Clear Register ( /W) */
  __IO   uint32_t  PUDISSET;         /*   Offset: 0x040 Pullup Disable Set Register (R/W) */
  __IO   uint32_t  PUDISCLR;         /*   Offset: 0x044 Pullup Disable Clear Register (R/W) */
  __IO   uint32_t  PUSELSET;         /*   Offset: 0x048 Pullup Select Set (R/W) */
  __IO   uint32_t  PUSELCLR;         /*   Offset: 0x04C Pullup Select Clear (R/W) */
  __IO   uint32_t  DSCTL;            /*   Offset: 0x050 Drive Strength Control (R/W) */
         uint32_t  RESERVED0[235];
  __IO   uint32_t  LB_MASKED[256];   /*   Offset: 0x400 - 0x7FC Lower byte Masked Access Register (R/W) */
  __IO   uint32_t  UB_MASKED[256];   /*   Offset: 0x800 - 0xBFC Upper byte Masked Access Register (R/W) */
} GPIO_TypeDef;


#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Base addresses                                                             */
#define CODEFLASH_BASE    (0x00000000UL)
#define DATAFLASH_BASE    (0x10000000UL)
#define RAM_BASE          (0x20000000UL)
#define APB_BASE          (0x40000000UL)
#define AHB_BASE          (0x40080000UL)

// APB peripherals
#define RTC_BASE          (APB_BASE + 0x00000UL)
#define TIMER0_BASE       (APB_BASE + 0x01000UL)
#define TIMER1_BASE       (APB_BASE + 0x02000UL)
#define TIMER2_BASE       (APB_BASE + 0x03000UL)
#define TIMER3_BASE       (APB_BASE + 0x04000UL)
#define TIMER4_BASE       (APB_BASE + 0x05000UL)
#define WDT_BASE          (APB_BASE + 0x06000UL)
#define LCD_BASE          (APB_BASE + 0x08000UL)
#define ADC_BASE          (APB_BASE + 0x0A000UL)
#define UART0_BASE        (APB_BASE + 0x10000UL)
#define UART1_BASE        (APB_BASE + 0x11000UL)
#define UART2_BASE        (APB_BASE + 0x12000UL)
#define UART3_BASE        (APB_BASE + 0x13000UL)
#define UART4_BASE        (APB_BASE + 0x14000UL)
#define UART5_BASE        (APB_BASE + 0x15000UL)
#define CFLASH_BASE       (APB_BASE + 0x17000UL)
#define DFLASH_BASE       (APB_BASE + 0x18000UL)
#define SPI_BASE          (APB_BASE + 0x1A000UL)
#define SYSCTRL_BASE      (APB_BASE + 0x1D000UL)

// AHB peripherals                                                           
#define GPIOA_BASE        (AHB_BASE + 0x00000UL)
#define GPIOB_BASE        (AHB_BASE + 0x01000UL)
#define GPIOC_BASE        (AHB_BASE + 0x02000UL)

/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
#define SPI                ((SPI_TypeDef     *) SPI_BASE     )
#define TIMER0             ((TIMER_TypeDef   *) TIMER0_BASE  )
#define TIMER1             ((TIMER_TypeDef   *) TIMER1_BASE  )
#define TIMER2             ((TIMER_TypeDef   *) TIMER2_BASE  )
#define TIMER3             ((TIMER_TypeDef   *) TIMER3_BASE  )
#define TIMER4             ((TIMER_TypeDef   *) TIMER4_BASE  )
#define SYSCTRL            ((SYSCTRL_TypeDef *) SYSCTRL_BASE )
#define WDT                ((WDT_TypeDef     *) WDT_BASE     )
#define LCD                ((LCD_TypeDef     *) LCD_BASE     )
#define ADC                ((ADC_TypeDef     *) ADC_BASE     )

#define CFLASH             ((FLASH_TypeDef   *) CFLASH_BASE  )
#define DFLASH             ((FLASH_TypeDef   *) DFLASH_BASE  )
#define RTC                ((RTC_TypeDef     *) RTC_BASE     )
#define UART0              ((UART_TypeDef    *) UART0_BASE   )
#define UART1              ((UART_TypeDef    *) UART1_BASE   )
#define UART2              ((UART_TypeDef    *) UART2_BASE   )
#define UART3              ((UART_TypeDef    *) UART3_BASE   )
#define UART4              ((UART_TypeDef    *) UART4_BASE   )
#define UART5              ((UART_TypeDef    *) UART5_BASE   )

#define GPIOA              ((GPIO_TypeDef    *) GPIOA_BASE   )
#define GPIOB              ((GPIO_TypeDef    *) GPIOB_BASE   )
#define GPIOC              ((GPIO_TypeDef    *) GPIOC_BASE   )

#define SYSCTRL_REG        ((SYSCTRL_REG_TypeDef *) SYSCTRL_BASE )
#define RTC_REG            ((RTC_REG_TypeDef     *) RTC_BASE     )
#define LCD_REG            ((LCD_REG_TypeDef     *) LCD_BASE     )

#define TIMER0_REG         ((TIMER_REG_TypeDef   *) TIMER0_BASE  )
#define TIMER1_REG         ((TIMER_REG_TypeDef   *) TIMER1_BASE  )
#define TIMER2_REG         ((TIMER_REG_TypeDef   *) TIMER2_BASE  )
#define TIMER3_REG         ((TIMER_REG_TypeDef   *) TIMER3_BASE  )
#define TIMER4_REG         ((TIMER_REG_TypeDef   *) TIMER4_BASE  )
#define UART0_REG          ((UART_REG_TypeDef    *) UART0_BASE   )
#define UART1_REG          ((UART_REG_TypeDef    *) UART1_BASE   )
#define UART2_REG          ((UART_REG_TypeDef    *) UART2_BASE   )
#define UART3_REG          ((UART_REG_TypeDef    *) UART3_BASE   )
#define UART4_REG          ((UART_REG_TypeDef    *) UART4_BASE   )
#define UART5_REG          ((UART_REG_TypeDef    *) UART5_BASE   )
#define ADC_REG            ((ADC_REG_TypeDef     *) ADC_BASE     )
#define WDT_REG            ((WDT_REG_TypeDef     *) WDT_BASE     )
#define SPI_REG            ((SPI_REG_TypeDef     *) SPI_BASE     )


/******************************************************************************/
/* Bit definitions */
#define _BV(bit)  (((uint32_t)1UL)<<(bit))
#define BV(bit) _BV((bit))
/******************************************************************************/


/******************************************************************************/
/*                         Peripheral Registers Bits Definition               */
/******************************************************************************/


/******************************************************************************/
/* SYSCTRL                                                                    */
/******************************************************************************/

/* SYS_RST_STA */
#define SYSCTRL_SYS_RST_STA_WDT_TIMEOUT _BV(0)
#define SYSCTRL_SYS_RST_STA_CM0LOCKUP   _BV(1)

/* SYS_CLK_STA */
#define SYSCTRL_SYS_CLK_STA_DPLL_CLOCK  _BV(0)

/* SWD_ERA_KEY */
#define SYSCTRL_SWD_ERA_KEY             ((uint32_t)0xFFFFFFFF)

/* SWD_ERA_CMD */
#define SYSCTRL_SWD_ERA_CMD_CFERA       _BV(0)
#define SYSCTRL_SWD_ERA_CMS_DFERA       _BV(1)

/* RST_CR */
#define SYSCTRL_RST_CR_RSTN_KEY        ((uint32_t)0x000000FF)

/* CLK_DIVSEL */
#define SYSCTRL_CLK_DIVSEL             (_BV(0) | _BV(1) | _BV(2))

/* CLK_SRCSEL */
#define SYSCTRL_CLK_SRCSEL_LCDCLKSEL    _BV(0)
#define SYSCTRL_CLK_SRCSEL_LXTALSEL     _BV(1)
#define SYSCTRL_CLK_SRCSEL_HXTALSEL     _BV(2)

/* NMI_CTL */
#define SYSCTRL_NMI_CTL_NMISEL          (_BV(0) | _BV(1) | _BV(2) | _BV(3) | _BV(4))
#define SYSCTRL_NIM_CTL_NMIEN           _BV(5)   

/* HSRC_CTL */
/* BIT 0 is reserved */
#define SYSCTRL_HSRC_CTL_PWRDN          _BV(1)
#define SYSCTRL_HSRC_CTL_HTRIM          (_BV(2) | _BV(3) | _BV(4) | _BV(5) | _BV(6))
#define SYSCTRL_HSRC_CTL_HTRIM_EN       _BV(7)

/* LSRC_CTL */
#define SYSCTRL_LSRC_CTL_LSRC_ENB       _BV(0)
#define SYSCTRL_LSRC_CTL_VREF_ENB       _BV(1)
#define SYSCTRL_LSRC_CTL_LTRIM          (_BV(2) | _BV(3) | _BV(4) | _BV(5))
#define SYSCTRL_LSRC_CTL_LTRIM_EN       _BV(6)

/* PWR_CTL */
#define SYSCTRL_PWR_CTL_VRACTIVE        _BV(0)
#define SYSCTRL_PWR_CTL_SRAMPS_EN       _BV(1)
#define SYSCTRL_PWR_CTL_ADCPS_EN        _BV(2)

/* DPLL_CTL */
#define SYSCTRL_DPLL_CTL_DPLL_EN        _BV(0)

/* CHIP_CFG */
#define SYSCTRL_CHIP_CFG_LXTAL_CFG      (_BV(0) | _BV(1))
#define SYSCTRL_CHIP_CFG_BOND_A_SEL     _BV(2)
#define SYSCTRL_CHIP_CFG_BOND_B_SEL     _BV(3)
#define SYSCTRL_CHIP_CFG_OSC32K_ADJ     (_BV(4) | _BV(5) | _BV(6))
#define SYSCTRL_CHIP_CFG_LSRC_TRIM      (_BV(7) | _BV(8) | _BV(9) | _BV(10))
#define SYSCTRL_CHIP_CFG_HSRC_TRIM      (_BV(11) | _BV(12) | BV(13) | _BV(14) | _BV(15))

/* SYS_CPL */
#define SYSCTRL_SYS_CPL                 (_BV(0) | _BV(1))

/* HXTAL_CTL */
#define SYSCTRL_HXTAL_CTL_HXTAL_EN      _BV(0)

/* RTC_CTL */
#define SYSCTRL_RTC_CTL_IGN_RSTN        _BV(0)
#define SYSCTRL_RTC_CTL_SW_RESET_RTC    _BV(1)
#define SYSCTRL_RTC_CTL_PD_LXTAL        _BV(2)

/* WDT_CTL */
#define SYSCTRL_WDT_CTL_WDT_TRIG        _BV(0)


/******************************************************************************/
/* TIMER                                                                      */
/******************************************************************************/
/* TM_CTRL */
#define TIMER_CTRL_CEN  _BV(0)
#define TIMER_CTRL_CRST _BV(1)
#define TIMER_CTRL_SRC  _BV(2)

/* TM_IF */
#define TIMER_IF_COM0 _BV(0)
#define TIMER_IF_COM1 _BV(1)
#define TIMER_IF_COM2 _BV(2)
#define TIMER_IF_CAP0 _BV(3)
#define TIMER_IF_CAP1 _BV(4)

/* TM_CNT */

/* TM_PR */

/* TM_PS */

/* TM_COMCR */
#define TIMER_COM0_IE    _BV(0)
#define TIMER_COM0_RST   _BV(1)
#define TIMER_COM0_STP   _BV(2)
#define TIMER_COM1_IE    _BV(3)
#define TIMER_COM1_RST   _BV(4)
#define TIMER_COM1_STP   _BV(5)
#define TIMER_COM2_IE    _BV(6)
#define TIMER_COM2_RST   _BV(7)
#define TIMER_COM2_STP   _BV(8)

/* TM_COMR0 */

/* TM_COMR1 */

/* TM_COMR2 */

/* TM_CAPCR */
#define TIMER_CAP0_RE   _BV(0)
#define TIMER_CAP0_FE   _BV(1)
#define TIMER_CAP0_IE   _BV(2)
#define TIMER_CAP1_RE   _BV(3)
#define TIMER_CAP1_FE   _BV(4)
#define TIMER_CAP1_IE   _BV(5)

/* TM_CAP0 */

/* TM_CAP1 */

/* TM_OCR */
#define TIMER_OCR_OC0   _BV(0)
#define TIMER_OCR_OCC0  (_BV(2) | _BV(1))
#define TIMER_OCR_OC1   _BV(3)
#define TIMER_OCR_OCC1  (_BV(4) | _BV(5))
#define TIMER_OCR_OC2   _BV(6)
#define TIMER_OCR_OCC2  (_BV(7) | _BV(8))

/* TM_PWMC */
#define TIMER_PWMEN0    _BV(0)
#define TIMER_PWNEN1    _BV(1)

/* TM_DBG */
#define TIMER_DBG_HALT  _BV(0)

/******************************************************************************/
/* UART                                                                       */
/******************************************************************************/
/* U_DATA */
#define UART_DATA                       ((uint32)0x000000FF)

/* U_STATE */
#define UART_STATE_TXE                  _BV(0)
#define UART_STATE_RXF                  _BV(1)
#define UART_STATE_RXO                  _BV(2)
#define UART_STATE_FE                   _BV(3)           
#define UART_STATE_PE                   _BV(4)
#define UART_STATE_TC                   _BV(5)
#define UART_STATE_NACK                 _BV(6)

/* U_CTRL */
#define UART_CTRL_TX_IE                 _BV(0)
#define UART_CTRL_RX_IE                 _BV(1)
#define UART_CTRL_TC_IE                 _BV(2)
#define UART_CTRL_TXPOL                 _BV(3)
#define UART_CTRL_RXPOL                 _BV(4)
#define UART_CTRL_DL                    _BV(5)
#define UART_CTRL_SL                    _BV(6)
#define UART_CTRL_PTY                   _BV(7)
#define UART_CTRL_PEN                   _BV(8)
#define UART_CTRL_IRPOL                 _BV(8)
#define UART_CTRL_IREN                  _BV(10)
#define UART_CTRL_MODE                  _BV(11)
#define UART_CTRL_LB                    _BV(12)

/* U_IF */
#define UART_IF_TX_IF                   _BV(0)
#define UART_IF_RX_IF                   _BV(1)
#define UART_IF_TC_IF                   _BV(2)

/* UART_BAUD */
#define UART_BAUD                       ((uint32_t)0x00000FFF)

/* UART_SCCTRL */
#define UART_SCCTRL_EGT                 ((uint32_t)0x000000FF)
#define UART_SCCTRL_SCM                 _BV(8)
#define UART_SCCTRL_SCCLK_DIV           (_BV(9) | _BV(10) | _BV(11))

/* UART_SCCLKEN */
#define UART_SCCLKEN                    _BV(0)

/* UART_CLKEN */
#define UART_CLKEN                      _BV(0)

/* UART_ENABLE */
#define UART_ENABLE_TXEN                _BV(0)
#define UART_ENABLE_RXEN                _BV(1)


#define CM0_IRQ_PRIO_HIGH       0x0UL
#define CM0_IRQ_PRIO_MID_HIGH   0x1UL
#define CM0_IRQ_PRIO_MID_LOW    0x2UL
#define CM0_IRQ_PRIO_LOW        0x3UL


// Macro for absolute memory access
#define M8(adr)		(*((volatile uint8_t  *) (adr)))
#define M16(adr)	(*((volatile uint16_t *) (adr)))
#define M32(adr)	(*((volatile uint32_t *) (adr)))

// Bitband Access to SRAM
#define _RAM_BITBAND_BASE       0x22000000UL
#define _RAM_BASE               0x20000000UL
#define RAM_BITBAND(addr,bit)   M32((_RAM_BITBAND_BASE) + (((addr) -(_RAM_BASE))<<5) + ((bit)<<2))

// Bitband Access to APB
#define _APB_BITBAND_BASE       0x42000000UL
#define _APB_BASE               0x40000000UL
#define APB_BITBAND(addr,bit)   M32((_APB_BITBAND_BASE) + (((addr) -(_APB_BASE))<<5) + ((bit)<<2))

// Bitband Access to APB
#define _AHB_BITBAND_BASE       0x43000000UL
#define _AHB_BASE               0x40080000UL
#define AHB_BITBAND(addr,bit)   M32((_AHB_BITBAND_BASE) + (((addr) -(_AHB_BASE))<<5) + ((bit)<<2))


#ifdef USE_PERIPH_DRIVER
  #include "gct301s_conf.h"
#endif

#ifdef __cplusplus
}
#endif

#endif  /* __GCT301S_H */
