#ifndef __GCT301S_FLASH_H
#define __GCT301S_FLASH_H

#include <stdint.h>

#define __I  volatile const     /*!< defines 'read only' permissions                 */
#define __O  volatile           /*!< defines 'write only' permissions                */
#define __IO volatile           /*!< defines 'read / write' permissions              */
 
#define _BV(bit)  (((uint32_t)1UL)<<(bit))
#define BV(bit) _BV((bit))

/**
 * Memory access
 */
#define M8(adr)		(*((volatile uint8_t  *) (adr)))
#define M16(adr)	(*((volatile uint16_t *) (adr)))
#define M32(adr)	(*((volatile uint32_t *) (adr)))

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
  __IO   uint32_t  INTSTATUS;      /*   Offset: 0x038 Interrupt Status Register (R/ ) */
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

/*-------------------- FLASH Controller -------------------*/
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


/*-------------------- System Control (SYSCTRL) -------------------*/
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
      __IO uint32_t           : 1;
      __IO uint32_t PWRDN     : 1;
      __IO uint32_t HTRIM     : 5;
      __IO uint32_t HTRIM_EN  : 1;
    } BITS;
  } HSRC_CTL;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t LSRC_ENB : 1;
      __IO uint32_t VREF_ENB : 1;
      __IO uint32_t LTRIM    : 4;
      __IO uint32_t LTRIM_EN : 1;
    } BITS;
  } LSRC_CTL;
  union {
    __IO  uint32_t VAL;
    struct {
      __IO uint32_t VRACTIVE  : 1;
      __IO uint32_t SRAMPS_EN : 1;
      __IO uint32_t ADCPS_EN  : 1;
    } BITS;
  } PWR_CTL;
  union {
    __IO  uint32_t  VAL;
    struct {
      __IO uint32_t DPLL_EN : 1;
    } BITS;
  } DPLL_CTL;
  union {
    __I   uint32_t VAL;
    struct {
      __I  uint32_t LXTAL_CFG  : 2;
      __I  uint32_t BOND_A_SEL : 1;
      __I  uint32_t BOND_B_SEL : 1;
      __I  uint32_t OSC32K_ADJ : 3;
      __I  uint32_t LSRC_TRIM  : 4;
      __I  uint32_t HSRC_TRUM  : 5;
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
} SYSCTRL_TypeDef;

#define AHB_BASE          (0x40080000UL)

#define GPIOA_BASE        (AHB_BASE + 0x00000UL)
#define GPIOB_BASE        (AHB_BASE + 0x01000UL)
#define GPIOC_BASE        (AHB_BASE + 0x02000UL)


#define GPIOA              ((GPIO_TypeDef    *) GPIOA_BASE   )
#define GPIOB              ((GPIO_TypeDef    *) GPIOB_BASE   )
#define GPIOC              ((GPIO_TypeDef    *) GPIOC_BASE   )

#define CODEFLASH_BASE    (0x00000000UL)
#define DATAFLASH_BASE    (0x10000000UL)

#define APB_BASE          (0x40000000UL)

#define SYSCTRL_BASE      (APB_BASE + 0x1D000UL)

#define CFLASH_BASE       (APB_BASE + 0x17000UL)
#define DFLASH_BASE       (APB_BASE + 0x18000UL)

#define SYSCTRL           ((SYSCTRL_TypeDef *) SYSCTRL_BASE )
#define CFLASH            ((FLASH_TypeDef   *) CFLASH_BASE  )
#define DFLASH            ((FLASH_TypeDef   *) DFLASH_BASE  )

#define FLASH_KEY 	0xC6A5

#define FLASH_NVRP_WRITE    0x5AA5
#define FLASH_NVRP_READ 	0xA55A

#define MERA        0x1
#define CERA        0x2
#define NVRERA      0x4

#define MPROG       0x1
#define NVRROG      0x2
#endif
