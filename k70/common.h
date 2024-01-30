/*
 * File:        common.h
 * Purpose:     File to be included by all project files
 *
 * Notes:
 */

#ifndef _COMMON_H_
#define _COMMON_H_

/********************************************************************/

/*
 * Debug prints ON (#define) or OFF (#undef)
 */
#define DEBUG
#define DEBUG_PRINT

/* 
 * Include the generic CPU header file 
 */
//#include "arm_cm4.h"
#include <stdint.h>

#define EnableInterrupts asm(" CPSIE i");

  /*!< Macro to disable all interrupts. */
#define DisableInterrupts asm(" CPSID i");

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;


/* 
 * Include the platform specific header file 
 */
#if (defined(TWR_K70F120M))
  #include "twr-k70f120m.h"
#elif (defined(TWR_K60F120M))
  #include "twr-k60f120m.h"
#else
  #error "No valid platform defined"
#endif

/* 
 * Include the cpu specific header file 
 */
#if (defined(CPU_MK40D100))
  #include "MK40DZ10.h"
#elif (defined(CPU_MK60D100))
  #include "MK60DZ10.h"
#elif (defined(CPU_MK53D100))
  #include "MK53DZ10.h"
#elif (defined(CPU_MK70F120))
  #include "MK70F15.h"
#elif (defined(CPU_MK60F120))
  #include "MK70F15.h"
#else
  #error "No valid CPU defined"
#endif

/* 
 * Include any toolchain specfic header files 
 */
#if (defined(CW))
	#include "cw.h"
	#define printf printf_kinetis
	#define sprintf sprintf_kinetis
#elif (defined(IAR))
  #include "iar.h"
#else
//#warning "No toolchain specific header included"
#endif

/* 
 * Include common utilities
 */
#include <stdlib.h>

#if (defined(IAR))
	#include "intrinsics.h"
	#include "assert.h"
	#include "io.h"
#endif

/********************************************************************/

#endif /* _COMMON_H_ */
