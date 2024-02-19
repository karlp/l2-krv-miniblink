
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

#include "bsp/board_api.h"
#include "tusb.h"

#include <interrupt/interrupt.h>
#include <nxp_kx/mcg.h>
#include <nxp_kx/mpu.h>
#include <nxp_kx/osc.h>
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

#if 0
const mcg_config_t mcgConfig_BOARD_BootClockRUN =
	{
		.mcgMode = kMCG_ModePEE,			 /* PEE - PLL Engaged External */
		.irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
		.ircs = kMCG_IrcSlow,				 /* Slow internal reference clock selected */
		.fcrdiv = 0x0U,						 /* Fast IRC divider: divided by 1 */
		.frdiv = 0x0U,						 /* FLL reference clock divider: divided by 32 */
		.drs = kMCG_DrsLow,					 /* Low frequency range */
		.dmx32 = kMCG_Dmx32Default,			 /* DCO has a default range of 25% */
		.oscsel = kMCG_OscselOsc,			 /* Selects System Oscillator (OSCCLK) */
		.pll0Config =
			{
				.enableMode = MCG_PLL_DISABLE, /* MCGPLLCLK disabled */
				.prdiv = 0x13U,				   /* PLL Reference divider: divided by 20 */
				.vdiv = 0x18U,				   /* VCO divider: multiplied by 48 */
			},
};
const sim_clock_config_t simConfig_BOARD_BootClockRUN =
	{
		.pllFllSel = SIM_PLLFLLSEL_MCGPLLCLK_CLK, /* PLLFLL select: MCGPLLCLK clock */
		.er32kSrc = SIM_OSC32KSEL_RTC32KCLK_CLK,  /* OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
		.clkdiv1 = 0x1240000U,					  /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /2, OUTDIV3: /3, OUTDIV4: /5 */
};
const osc_config_t oscConfig_BOARD_BootClockRUN =
	{
		.freq = 50000000U,		  /* Oscillator frequency: 50000000Hz */
		.capLoad = (OSC_CAP0P),	  /* Oscillator capacity load: 0pF */
		.workMode = kOSC_ModeExt, /* Use external clock */
		.oscerConfig =
			{
				.enableMode = kOSC_ErClkEnable, /* Enable external reference clock, disable external reference clock in STOP mode */
			}};
#endif
void laks_clock_config_for_usb_k64()
{
	// Here, we copy just the necessary parts of our tinyusb clock_config.c file...

	// "SetSimSafeDivs()"
	SIM->CLKDIV1 = 0x00040000U;

	// CLOCK_InitOsc0()
	int capload = 0;
	OSC0->CR = (OSC0->CR & ~(0xf)) | capload;
	int range = 2;								  // "very high"
	MCG->C2 = (1 << 7) | (range << 4) | (0 << 2); // loss of clock, range, external reference, don't care about others.
	OSC0->CR |= (1 << 7);						  // extrefenable  // ref code set this separately, after configuring C2...
	// only meant to wait here if we're usign oscillator, not external!
	// while (!(MCG->S & (1 << 1)))				  // ok, stuck here already. boooo
	// {
	// 	// Wait for stable
	// 	;
	// }

	// CLOCK_SetXtal0Freq(oscConfig_BOARD_BootClockRUN.freq); // just sets a global

	// CLOCK_SetInternalRefClkConfig
	// I'm not sure we even _care_ about this, it just sets the MCGIRCLK
	// that's the 30/40khz or 4Mhz clock.  Can be used by LPTMR, and also as a low speed VLPR system clock..
	// skip it for now!

	// CLOCK_CONFIG_SetFllExtRefDiv(mcgConfig_BOARD_BootClockRUN.frdiv);
	// again, only useful for setting fll divider, which we're not using...
	// skip it for now!

	/* Set MCG to PEE mode. */
	// CLOCK_BootToPeeMode(mcgConfig_BOARD_BootClockRUN.oscsel,
	//                 kMCG_PllClkSelPll0,
	//                 &mcgConfig_BOARD_BootClockRUN.pll0Config);
	MCG->C7 = 0; // osc0. whee.

	// Clear CLKS and IREFS, select CLKS=external
	MCG->C1 = (MCG->C1 & ~((0x3 << 6) | (1 << 2))) | (2 << 6);
	// while IREF and CLKST aren't what we want  yet.
	while ((MCG->S & ((1 << 4) | (0x3 << 2))) != ((0 << 4) | (2 << 2)))
	{
		;
	}
	// disable PLL, then configure.
	MCG->C6 &= ~(1 << 6);
	while (MCG->S & (1 << 5))
	{ // yes, bit 5 status for bit 6 selection. nxp ftw
		;
	}
	// Disable, but configure
	// k70 wants 8-16Mhz for pll ref,a nd only has 3 bits anyway..
#ifdef CPU_MK70FN1M0VMJ12
	int prdiv0 = 5; // = 10Mhz pllref  divider range is 1..8
	int vdiv0 = 24; // => 240MHz output  there's a /2 afterwards on k70.... range is 16--47times.
	int vdiv_correction = 16;
#else
	int prdiv0 = 20; // this is for k64, which needs 2-4Mhz pll ref.
	int vdiv0 = 48;
	int vdiv_correction = 24;
#endif
	MCG->C5 = 0 | ((prdiv0 - 1) << 0);
	MCG->C6 = (MCG->C6 & ~(0x1f << 0)) | ((vdiv0 - vdiv_correction) << 0);
	// Enable pll
	MCG->C5 |= (1 << 6);
	while (!(MCG->S & (1 << 6)))
	{
		; // wait for lock
	}

	// Change to pll mode. (PLLS)
	MCG->C6 |= (1 << 6);
	while (!(MCG->S & (1 << 5)))
	{
		; // wait for selection to switch
	}

	// now _use_ it
	int clks_pll = 0;
	MCG->C1 = (MCG->C1 & ~(3 << 6)) | (clks_pll << 6);
	while ((MCG->S & (3 << 2)) != (3 << 2))
	{
		; // wait for this switch to pll...
	}

	// CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
	SIM->CLKDIV1 = 0x01240000; // div1=/1, div2=/2,div3=/div3,div4=/5
	// CLOCK_SetPllFllSelClock(config->pllFllSel);
	int pllfllsel = 1; // MCGPLLCLK
	SIM->SOPT2 = (SIM->SOPT2 & ~(0x3 << 16)) | (pllfllsel << 16);
	// CLOCK_SetEr32kClock(config->er32kSrc);
	// don't think I care honestly, but whatever.
	// OSC32KSEL = RTC.
	SIM->SOPT1 = (SIM->SOPT1 & ~(3 << 18)) | (2 << 18);

	// CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0, SIM_USB_CLK_120000000HZ);
	// kCLOCK_UsbSrcPll0   = SIM_SOPT2_USBSRC(1U) | SIM_SOPT2_PLLFLLSEL(1U), /*!< Use PLL0.      */
	SIM.disable(sim::USBFS);
	// 120 * 2 / 5 == 48
	int udiv = 5;
	int ufrac = 2;
	SIM->CLKDIV2 = ((udiv - 1) << 1) | ((ufrac - 1) << 0);
	// USBSRC=fll|pll|irc48 + pllfllsel = pll
	// ALSO; FOR K70, USBFSRC is extra bits[22,23] but at zero, should be identical, ie, use pllfllsel.
	SIM->SOPT2 = (SIM->SOPT2 & ~((1 << 18) | (3 << 16))) | (1 << 18) | (1 << 16);
	SIM.enable(sim::USBFS);

	// Turn off the MPU so that the usb peripheral can access transfer buffers!
	// SYSMPU->CESR = 0;
	MPU.disable();
}

void board_init()
{

#ifdef RCC_ENABLE1
	SIM.enable(RCC_ENABLE1);
#endif

	// FIXME - this is gross.  want to use a ?friend? class
	// we need to assign the right "PRCx" from the led1 pin?
	// HACK HACK HACK
#ifdef CPU_MK70FN1M0VMJ12
	PCRA.mux(led1.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
#else
	PCRB.mux(led1.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
#endif

	led1.set_out();
	uint32_t sdid = SIM->SDID;
	int fam = (sdid >> 4) & 0x7;
	int pinid = sdid & 0xf;
	int revid = (sdid >> 12) & 0xf;
	switch (fam)
	{
	case 0x5:
		printf("alive on K70 family, pins: %d, revid: %d\n", pinid, revid);
		break;
	case 0x4:
		printf("alive on K6x family, pins: %d, revid: %d\n", pinid, revid);
		break;
	default:
		printf("unimplemented, not decoding sdid: %lx\n", sdid);
		break;
	}

	laks_clock_config_for_usb_k64();
	SystemCoreClock = 120000000;
	printf("Running sys clock: %lu\n", SystemCoreClock);
	printf("SIM sopt2: %lx, scgc4: %lx, clkdiv2: %lx\n", SIM->SOPT2, SIM->SCGC4, SIM->CLKDIV2);
	printf("MCG c1: %x, c2: %x, c5: %x, c6: %x\n", MCG->C1, MCG->C2, MCG->C5, MCG->C6);
	// Mine, now at least: check against another one?
	// Running sys clock: 120000000
	// SIM sopt2: 51000, scgc4: f0140030, clkdiv2: 9
	// MCG c1: 0, c2: a0, c5: 53, c6: 58

	// tu demo working:
	//	Running sys clock: 120000000
	// SIM sopt2: 51000, scgc4: f0140430, clkdiv2: 9
	// MCG c1: 2, c2: e0, c5: 53, c6: 58
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

	printf("laks+TinyUSB Host CDC MSC HID with FreeRTOS Example\n");

	// Create soft timer for blinky, task for tinyusb stack
#if configSUPPORT_STATIC_ALLOCATION
	blinky_tm = xTimerCreateStatic(NULL, pdMS_TO_TICKS(BLINK_MOUNTED), true, NULL, led_blinky_cb, &blinky_tmdef);
	xTaskCreateStatic(usb_host_task, "usbh", USBH_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, usb_host_stack, &usb_host_taskdef);
#else
	blinky_tm = xTimerCreate(NULL, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), true, NULL, led_blinky_cb);
	xTaskCreate(usb_host_task, "usbd", USBH_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
#endif

	xTimerStart(blinky_tm, 0);

	// NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	// Required to use FreeRTOS ISR methods!
	NVIC.set_priority(interrupt::irq::USB_OTG, 6 << configPRIO_BITS);

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

#if CFG_TUD_ENABLED && defined(BOARD_TUD_RHPORT)
#define PORT_SUPPORT_DEVICE(_n) (BOARD_TUD_RHPORT == _n)
#else
#define PORT_SUPPORT_DEVICE(_n) 0
#endif

#if CFG_TUH_ENABLED && defined(BOARD_TUH_RHPORT)
#define PORT_SUPPORT_HOST(_n) (BOARD_TUH_RHPORT == _n)
#else
#define PORT_SUPPORT_HOST(_n) 0
#endif
template <>
void interrupt::handler<interrupt::irq::USB_OTG>()
{
#if PORT_SUPPORT_HOST(0)
	tuh_int_handler(0, true);
#endif

#if PORT_SUPPORT_DEVICE(0)
	tud_int_handler(0);
#endif
}
