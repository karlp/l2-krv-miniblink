
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include <interrupt/interrupt.h>
#include <nxp_kx/rcm.h>
#include <nxp_kx/sim.h>
#include <nxp_kx/wdog.h>
#include <gpio/gpio.h>

// Increase stack size when debug log is enabled
#define USBH_STACK_SIZE (3 * configMINIMAL_STACK_SIZE / 2) * (CFG_TUSB_DEBUG ? 2 : 1)

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTOTYPES
//--------------------------------------------------------------------+
/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

// static timer & task
#if configSUPPORT_STATIC_ALLOCATION
StaticTimer_t blinky_tmdef;

StackType_t usb_host_stack[USBH_STACK_SIZE];
StaticTask_t usb_host_taskdef;
#endif

TimerHandle_t blinky_tm;

static void led_blinky_cb(TimerHandle_t xTimer);
static void usb_host_task(void *param);

extern "C"
{
  extern void cdc_app_init(void);
  extern void hid_app_init(void);
  extern void msc_app_init(void);
  uint32_t SystemCoreClock;
}

#if CFG_TUH_ENABLED && CFG_TUH_MAX3421
// API to read/rite MAX3421's register. Implemented by TinyUSB
extern uint8_t tuh_max3421_reg_read(uint8_t rhport, uint8_t reg, bool in_isr);
extern bool tuh_max3421_reg_write(uint8_t rhport, uint8_t reg, uint8_t data, bool in_isr);
#endif

Pin led1 = GPIO_LED1;

void board_init()
{

#ifdef RCC_ENABLE1
  SIM.enable(RCC_ENABLE1);
#endif

  // FIXME - this is gross.  want to use a ?friend? class
  // we need to assign the right "PRCx" from the led1 pin?
  PCRB.mux(led1.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);

  led1.set_out();

  SystemCoreClock = 120000000;
}

void board_led_write(bool on)
{
  led1.set(on);
}

int board_getchar(void)
{
  return 0;
}

int board_uart_read(uint8_t *buf, int len)
{
#if 0 /*                                                             \
  Use this version if want the LED to blink during BOARD=board_test, \
  without having to hit a key.                                       \
      */
  if( 0U != (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags( UART_PORT )) )
    {
      LPUART_ReadBlocking(UART_PORT, buf, len);
      return len;
    }

  return( 0 );
#else /* Wait for 'len' characters to come in */

  //  LPUART_ReadBlocking(UART_PORT, buf, len);
  return len;

#endif
}

int board_uart_write(void const *buf, int len)
{
  // LOL LPUART_WriteBlocking(UART_PORT, (uint8_t const*) buf, len);
  return len;
}

extern void
laks_entry(void);

void entry(void)
{
  // MUST be early!
  WDOG.unlock();
  WDOG.disable();

  laks_entry();
}

int main()
{
  board_init();

  printf("TinyUSB Host CDC MSC HID with FreeRTOS Example\n");
  printf("lol, it won't _work_ untilyou finish the clocks!\n");

  // Create soft timer for blinky, task for tinyusb stack
#if configSUPPORT_STATIC_ALLOCATION
  blinky_tm = xTimerCreateStatic(NULL, pdMS_TO_TICKS(BLINK_MOUNTED), true, NULL, led_blinky_cb, &blinky_tmdef);
  xTaskCreateStatic(usb_host_task, "usbh", USBH_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, usb_host_stack, &usb_host_taskdef);
#else
  blinky_tm = xTimerCreate(NULL, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), true, NULL, led_blinky_cb);
  xTaskCreate(usb_host_task, "usbd", USBH_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
#endif

  xTimerStart(blinky_tm, 0);

  vTaskStartScheduler();
}

// USB Host task
// This top level thread process all usb events and invoke callbacks
static void usb_host_task(void *param)
{
  (void)param;

  // init host stack on configured roothub port
  tuh_init(BOARD_TUH_RHPORT);

  if (board_init_after_tusb)
  {
    board_init_after_tusb();
  }

#if CFG_TUH_ENABLED && CFG_TUH_MAX3421
  // FeatherWing MAX3421E use MAX3421E's GPIO0 for VBUS enable
  enum
  {
    IOPINS1_ADDR = 20u << 3,
    /* 0xA0 */
  };
  tuh_max3421_reg_write(BOARD_TUH_RHPORT, IOPINS1_ADDR, 0x01, false);
#endif

  cdc_app_init();
  hid_app_init();
  msc_app_init();

  // RTOS forever loop
  while (1)
  {
    // put this thread to waiting state until there is new events
    tuh_task();

    // following code only run if tuh_task() process at least 1 event
  }
}

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+

void tuh_mount_cb(uint8_t dev_addr)
{
  // application set-up
  printf("A device with address %d is mounted\r\n", dev_addr);
}

void tuh_umount_cb(uint8_t dev_addr)
{
  // application tear-down
  printf("A device with address %d is unmounted \r\n", dev_addr);
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
static void led_blinky_cb(TimerHandle_t xTimer)
{
  (void)xTimer;
  static bool led_state = false;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}

// TODO -figure out how to give this to freertosconfig?
// #define vPortSVCHandler SVC_Handler
// #define xPortPendSVHandler PendSV_Handler
// #define xPortSysTickHandler SysTick_Handler
extern "C"
{
  void vPortSVCHandler(void);
  void xPortPendSVHandler(void);
  void xPortSysTickHandler(void);

// This should work with laks interrupts... but going to have the include paths..
#define __COMPILER_BARRIER() __asm__ volatile("" ::: "memory")

  void NVIC_EnableIRQ(IRQn_Type IRQn)
  {
    if ((int32_t)(IRQn) >= 0)
    {
      __COMPILER_BARRIER();
      NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
      __COMPILER_BARRIER();
    }
  }
  void NVIC_DisableIRQ(IRQn_Type IRQn)
  {
    if ((int32_t)(IRQn) >= 0)
    {
      NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
      __DSB();
      __ISB();
    }
  }

  void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
  {
    if ((int32_t)(IRQn) >= 0)
    {
      NVIC->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    }
  }

  uint32_t NVIC_GetEnableIRQ(IRQn_Type IRQn)
  {
    if ((int32_t)(IRQn) >= 0)
    {
      return ((uint32_t)(((NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
    }
    else
    {
      return (0U);
    }
  }
}
template <>
void interrupt::handler<interrupt::exception::SVCall>()
{
  vPortSVCHandler();
}
template <>
void interrupt::handler<interrupt::exception::PendSV>()
{
  xPortPendSVHandler();
}
template <>
void interrupt::handler<interrupt::exception::SysTick>()
{
  xPortSysTickHandler();
}