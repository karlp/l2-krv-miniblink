/*
 * File:        twr-k70f120m.h
 * Purpose:     Kinetis TWR-K70F120M tower CPU card definitions
 *
 * Notes:
 */

#ifndef __TWR_K70F120M_H__
#define __TWR_K70F120M_H__

#include "mcg.h"

/********************************************************************/

/* Global defines to use for all tower boards */
#define DEBUG_PRINT

#define CPU_MK70F120

  /*
   * Input Clock Info
   */
  #define CLK0_FREQ_HZ        50000000
  #define CLK0_TYPE           CANNED_OSC

  #define CLK1_FREQ_HZ        12000000
  #define CLK1_TYPE           CRYSTAL

  /*
   * PLL Configuration Info
   */
/* If the project options define SYNCH_MODE or NO_PLL_INIT, then
 * use that as the clock otherwise the default mode is ASYNCH_MODE.
 */
#if !(defined(SYNCH_MODE) | defined(NO_PLL_INIT))
  #define ASYNCH_MODE    /* PLL1 is source for MCGCLKOUT and DDR controller */
#endif

/* The expected PLL output frequency is:
 * PLL out = (((CLKIN/PRDIV) x VDIV) / 2)
 * where the CLKIN can be either CLK0_FREQ_HZ or CLK1_FREQ_HZ.
 * 
 * For more info on PLL initialization refer to the mcg driver files.
 */

  #define PLL0_PRDIV      5
  #define PLL0_VDIV       24

  #define PLL1_PRDIV      5
  #define PLL1_VDIV       30


/* Serial Port Info */
  #define TERM_PORT           UART2_BASE_PTR
  #define TERMINAL_BAUD       115200
  #undef  HW_FLOW_CONTROL

/* tower module LED defines */
  #define LED0_EN (PORTA_PCR11 = PORT_PCR_MUX(1))
  #define LED1_EN (PORTA_PCR28 = PORT_PCR_MUX(1))
  #define LED2_EN (PORTA_PCR29 = PORT_PCR_MUX(1))
  #define LED3_EN (PORTA_PCR10 = PORT_PCR_MUX(1))

  #define LED0_TOGGLE (GPIOA_PTOR = (1<<11))
  #define LED1_TOGGLE (GPIOA_PTOR = (1<<28))
  #define LED2_TOGGLE (GPIOA_PTOR = (1<<29))
  #define LED3_TOGGLE (GPIOA_PTOR = (1<<10))

  #define LED0_OFF (GPIOA_PSOR = (1<<11))
  #define LED1_OFF (GPIOA_PSOR = (1<<28))
  #define LED2_OFF (GPIOA_PSOR = (1<<29))
  #define LED3_OFF (GPIOA_PSOR = (1<<10))

  #define LED0_ON (GPIOA_PCOR = (1<<11))
  #define LED1_ON (GPIOA_PCOR = (1<<28))
  #define LED2_ON (GPIOA_PCOR = (1<<29))
  #define LED3_ON (GPIOA_PCOR = (1<<10))

#endif /* __TWR_K70F120M_H__ */
