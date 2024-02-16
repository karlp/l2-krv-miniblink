/*
 * Marel hf 2024.
 * Because NXP doesn't see fit to support K70...
 * We've faking a stub of just enough fsl_device_registers.h to give us the USB peripherals
 * to keep tinyusb happy.
 * All the _rest_ of the system, we use through laks.  (in this project at least)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __FSL_DEVICE_REGISTERS_H__
#define __FSL_DEVICE_REGISTERS_H__

#include <stdint.h>

#ifdef __cplusplus
#define __I volatile /*!< Defines 'read only' permissions */
#else
#define __I volatile const /*!< Defines 'read only' permissions */
#endif
#define __O volatile  /*!< Defines 'write only' permissions */
#define __IO volatile /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define __IM volatile const /*! Defines 'read only' structure member permissions */
#define __OM volatile       /*! Defines 'write only' structure member permissions */
#define __IOM volatile      /*! Defines 'read / write' structure member permissions */

// #define NVIC_SetPriorityGrouping __NVIC_SetPriorityGrouping
// #define NVIC_GetPriorityGrouping __NVIC_GetPriorityGrouping
// #define NVIC_EnableIRQ __NVIC_EnableIRQ
// #define NVIC_GetEnableIRQ __NVIC_GetEnableIRQ
// #define NVIC_DisableIRQ __NVIC_DisableIRQ
// #define NVIC_GetPendingIRQ __NVIC_GetPendingIRQ
// #define NVIC_SetPendingIRQ __NVIC_SetPendingIRQ
// #define NVIC_ClearPendingIRQ __NVIC_ClearPendingIRQ
// #define NVIC_GetActive __NVIC_GetActive
// #define NVIC_SetPriority __NVIC_SetPriority
// #define NVIC_GetPriority __NVIC_GetPriority
// #define NVIC_SystemReset __NVIC_SystemReset

#ifndef __STATIC_INLINE
#define __STATIC_INLINE static inline
#endif
#ifndef __STATIC_FORCEINLINE
#define __STATIC_FORCEINLINE __attribute__((always_inline)) static inline
#endif
#define __ASM __asm__

__STATIC_FORCEINLINE void __DSB(void)
{
     __ASM volatile("dsb 0xF" ::: "memory");
}
__STATIC_FORCEINLINE void __ISB(void)
{
     __ASM volatile("isb 0xF" ::: "memory");
}

#define __NOP() __ASM volatile("nop")

#if (defined(CPU_MK70FX512VMJ12) || defined(CPU_MK70FN1M0VMJ12) || \
     defined(CPU_MK70FX512VMJ15) || defined(CPU_MK70FN1M0VMJ15))
#define K70F12_SERIES

/* CMSIS-style register definitions */
#include "MK70F12.h"
/* CPU specific feature definitions */
// Nope, we're not supporting any of that.
// #include "MK70F12_features.h"

#elif (defined(CPU_MK64FN1M0CAJ12) || defined(CPU_MK64FN1M0VDC12) || defined(CPU_MK64FN1M0VLL12) || \
       defined(CPU_MK64FN1M0VLQ12) || defined(CPU_MK64FN1M0VMD12) || defined(CPU_MK64FX512VDC12) || \
       defined(CPU_MK64FX512VLL12) || defined(CPU_MK64FX512VLQ12) || defined(CPU_MK64FX512VMD12))
#define K64F12_SERIES
#include "MK64F12.h"

#else

#error "No valid CPU defined!"
#endif

#endif
