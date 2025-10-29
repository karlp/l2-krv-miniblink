
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

// #include "bsp/board_api.h"
// #include "tusb.h"

#include <interrupt/interrupt.h>
#include <nxp_kx/mcg.h>
#include <nxp_kx/mpu.h>
#include <nxp_kx/osc.h>
#include <nxp_kx/rcm.h>
#include <nxp_kx/sim.h>
#include <nxp_kx/wdog.h>
#include <gpio/gpio.h>

// Increase stack size when debug log is enabled
//#define USBH_STACK_SIZE (3 * configMINIMAL_STACK_SIZE / 2) * (CFG_TUSB_DEBUG ? 2 : 1)
#define USBH_STACK_SIZE (3 * configMINIMAL_STACK_SIZE / 2)

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
StackType_t task_late_stack[configMINIMAL_STACK_SIZE];
StaticTask_t task_late_handle;
#endif

TimerHandle_t blinky_tm;

static void led_blinky_cb(TimerHandle_t xTimer);
//static void usb_host_task(void *param);

extern "C"
{
	uint32_t SystemCoreClock;
	extern int fsl_app_main(void);
}


Pin led1 = GPIO_LED1;
PinMux led1_mux = GPIO_LED1_MUX[led1.n];
#if defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FN1M0VMJ15)
Pin usb_reset = GPIOD[11]; // only valid on m2400
PinMux usb_reset_mux = PCRD[usb_reset.n]; // only valid on m2400
Pin pswitch = GPIOB[4];	   // only on m2400
PinMux pswitch_mux = PCRB[pswitch.n];
#else
// hope you're on a frdmk64 board!
Pin usb_reset = GPIOB[22]; // use the red led as a standin for the hub reset
PinMux usb_reset_mux = PCRB[usb_reset.n];
Pin pswitch = GPIOC[6];	   // SW2
PinMux pswitch_mux = PCRC[pswitch.n];

// NXP_PinPair psw2 = NXP_PinPair(GPIOC[6], PCRC[6]);
#endif

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
	// no, it doesn't fix anything, as it shouldn't.
	MCG->C2 |= (1<<6); // do the damn fcftrim, it's the only bit that is different vs the factory exampples
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
	int ircs = 0;
	if (MCG->C1 & (1<<1)) {
		// meh, this path ~never happens on our codebase)
		// IRCLKEN, must go to slow turn off before changing..
		MCG->C2 &= ~(1<<0);
		while ((MCG->S & (1<<0)) != (ircs<<0)) {
			; // while not on slow
		}
		MCG.set_fcr_div(0);
	}
	MCG->C2 &= ~(1<<0);  // select slow
	MCG->C2 |= (ircs << 0);
	MCG->C1 = (MCG->C1 & ~(3<<0)) | (1<<1); // IRREFCLK EN, not in stop mode.
	if (MCG.clock_source() == 1) {
		while ((MCG->S & (1<<0)) != (ircs<<0)) {
			;
		}
	}



	// CLOCK_CONFIG_SetFllExtRefDiv(mcgConfig_BOARD_BootClockRUN.frdiv);
	// again, only useful for setting fll divider, which we're not using...
	// skip it for now!
	MCG.set_fr_div(0);



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
#if defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FN1M0VMJ15)
	int prdiv0 = 5; // = 10Mhz pllref  divider range is 1..8
	int vdiv0 = 24; // => 240MHz output  there's a /2 afterwards on k70.... range is 16--47times.
	int vdiv_correction = 16;
#else
// k64 has 50M ext from etherphy, so output is 50 / 20 * 48 == 120
	int prdiv0 = 20; // this is for k64, which needs 2-4Mhz pll ref.
	int vdiv0 = 48;
	int vdiv_correction = 24;
#endif
	MCG->C5 = 0 | ((prdiv0 - 1) << 0);
	MCG->C6 = (MCG->C6 & ~(0x1f << 0)) | ((vdiv0 - vdiv_correction) << 0);
	// Enable pll
	MCG->C5 |= (1 << 6);
//	while (!(MCG->S & (1 << 6)))
	while (!MCG.pll_is_locked())
	{
		; // wait for lock
	}

	// Change to pll mode. (PLLS)
	MCG->C6 |= (1 << 6);
	// while (!(MCG->S & (1 << 5)))
	while (!MCG.plls_is_pll())
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
#if defined(NONO_CPU_MK70FN1M0VMJ15)
	// LOL, 48Mhz oscillator on board as well...
	// usbclk = external + pllfllsel = pll. (leave it alone?)
	SIM->SOPT2 = (SIM->SOPT2 & ~((1 << 18) | (3 << 16))) | (0 << 18) | (1 << 16);
	// this... _shouldn't_ matter, as we don't _have_ to be on the "same" 48Mhz clock as the external hub, but... maybe?

#else

	// 120 * 2 / 5 == 48
	int udiv = 5;
	int ufrac = 2;
	SIM->CLKDIV2 = ((udiv - 1) << 1) | ((ufrac - 1) << 0);
	// USBSRC=fll|pll|irc48 + pllfllsel = pll
	// ALSO; FOR K70, USBFSRC is extra bits[22,23] but at zero, should be ident	ical, ie, use pllfllsel.
	SIM->SOPT2 = (SIM->SOPT2 & ~((1 << 18) | (3 << 16))) | (1 << 18) | (1 << 16);
#endif
	SIM.enable(sim::USBFS);

	// Turn off the MPU so that the usb peripheral can access transfer buffers!
	// SYSMPU->CESR = 0;
	MPU.disable();
}

void board_init()
{
	// just make sure, we have a few thigns we're poking, it's not all tied to the miniblink repo now.
	SIM.enable(sim::PORTA);
	SIM.enable(sim::PORTB);
	SIM.enable(sim::PORTC);
	SIM.enable(sim::PORTD);
	SIM.enable(sim::PORTE);
	//	SIM.enable(sim::PORTF); only k70 has portf, and none of our k70 boards use it anyway.

#ifdef RCC_ENABLE1
	SIM.enable(RCC_ENABLE1);
#endif

	// FIXME - this is gross.  want to use a ?friend? class
	// we need to assign the right "PRCx" from the led1 pin?
	// HACK HACK HACK
	led1_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
#ifdef CPU_MK70FN1M0VMJ12
	// superseded PCRA.mux(led1.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
#elif defined(CPU_MK70FN1M0VMJ15)
	// superseded PCRE.mux(led1.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);

	// M2400 has a 48MHz external oscillator feeding us on USB_CLKIN, _if we wish_
	// PCRE.mux(26, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt7);

	// Set up a pin to control the onboard hub's reset pin.
	SIM.enable(sim::PORTD);
	//PCRD.mux(usb_reset.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	usb_reset_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	usb_reset.set_out();
	// try just leaving it as is, on by default...
	usb_reset.on();
	// usb_reset.off();  // all it to be on out of the box, and we just stall ourselves

	// The config switch, for a basic UI...
	//PCRB.mux(pswitch.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	pswitch_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	pswitch.set_in();

#else
	// PCRB.mux(led1.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	// PCRB.mux(usb_reset.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	// superseded led1_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	usb_reset_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	usb_reset.set_out();
	// try just leaving it as is, on by default...
	usb_reset.on();
	// PCRC.mux(pswitch.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	pswitch_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	// PCRC->PCR[pswitch.n] |= 3; // pullup on switch pleas..
	pswitch_mux.pull(true, true);
	pswitch.set_in();
	// psw2.mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	// psw2.mux.pull(true, true);
	// psw2.pin.set_in();

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
// alive on K6x family, pins: 8, revid: 1
// Running sys clock: 120000000
// SIM sopt2: 51000, scgc4: f0140030, clkdiv2: 9
// MCG c1: 0, c2: a0, c5: 53, c6: 58
// so c2 and c1 are "wrong" ?
// c1 == 2 means "irclken", are we just not using it?
// c2=a0 vs e0 is bit 6, fcftrim...?
// scgc4extra! what am I missing, 0x400.. << uart2, no drama, we use RTT instead...


	// tu demo working:
	//	Running sys clock: 120000000
	// SIM sopt2: 51000, scgc4: f0140430, clkdiv2: 9
	// MCG c1: 2, c2: e0, c5: 53, c6: 58
	// later tu demo working has this, same as we have, so yeah, I knew it was ok.
	// SIM sopt2: 51000, scgc4: f0140030, clkdiv2: 9
	// MCG c1: 0, c2: a0, c5: 53, c6: 58


	// nxp demo running
// 	lol, starting nxp demo
// Running sys clock: 120000000
// SIM sopt2: 51000, scgc4: f0140430, clkdiv2: 9
// MCG c1: 2, c2: e0, c5: 53, c6: 58


	// good
	/*
UHD:nS=1 diS:2 stat:0
UHD:nS=1 diS:3 stat:0
UHD:nS=1 diS:4 stat:0
UHD:nS=1 diS:5 stat:0
UHD:nS=1 diS:6 stat:0
Hevt: 1
UHD:nS=1 diS:7 stat:0
Hevt: 3


(gdb) p /x bdt
$10 = {0x68, 0x1fff0220, 0x68, 0x1fff0220, 0x48, 0x0, 0x80008, 0x1fff8bf8, 0x0 <repeats 120 times>}

bad
UHD:nS=2 diS:2 stat:11   kStatus_USB_TransferFailed
UHD:nS=2 diS:2 stat:11
UHD:nS=2 diS:2 stat:11
UHD:nS=2 diS:2 stat:11
Hevt: b0005

bdt... (this might not mean much, as we didnt' finish enumeration, so... nothing to show?)
$8 = {0x0, 0x0, 0x0, 0x0, 0x80000, 0x1fff0200, 0x80000, 0x1fff0200, 0x0 <repeats 120 times>}


nvic configs are identical... what else is it...
enables ISR before starting the task? ok, try that. (nope, not that)

MCG c1: 2, c2: e0, c5: 53, c6: 58
	[4][4][64][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][8][4][8][8][4]UHD:nS=1[4] diS:2 s[4]tat:0
[4][8][8][4]UHD:n[4]S=1 diS:[4]3 stat:0[4]
[4][4][8][8][4][8][4][8][8][4]UHD:nS[4]=1 diS:4[4] stat:0
   [8][4][8][8][4][8][4]UHD:nS=1[4] diS:5 s[4]tat:0
[4][8][8][4][8][4][8][8][4][8][4][8]UHD[4]:nS=1 di[4]S:6 stat[4]:0
Hev[4]t: 1
[4][8][8][4]UHD:n[4]S=1 diS:[4]7 stat:0[4]
Hevt: [4]3
hid [4]mouse at[4]tached:p[4]id=0x34[4]vid=0x1c[4]4f addre[4]ss=1
[4]mouse [4]attach[4]ed

vs me:
(we're never getting the first SOF? (may just be rtt line buffering)
configpriobits is 4, usb hirq prio 6, lib max syscall: 2
[4][4][64][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][8][F-4096]
64 is attach, then lots more sof, then 8 is token done, just like they do, but... we don't continue..
UHD:nS=2 diS:2 stat:11
[8][F-4096]
UHD:nS=2 diS:2 stat:11
[8][F-4096]
UHD:nS=2 diS:2 stat:11
[4][8][F-4096]
UHD:nS=2 diS:2 stat:11
Hevt: b0005


	*/
}

void board_led_write(bool on)
{
	led1.set(on);
	//usb_reset.set(on);
}

// int board_getchar(void)
// {
// 	return 0;
// }

#if 0 // this is tusb stuff
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
#endif

extern void
laks_entry(void);

void entry(void)
{
	// MUST be early!
	WDOG.unlock();
	WDOG.disable();

	laks_entry();
}

static void task_late_start(void *pvParameters)
{
	// usb_reset.off();
	(void)pvParameters;
	// for (int i = 0; i < 10; i++)
	// {
	// 	vTaskDelay(pdMS_TO_TICKS(1000));
	// 	printf("Waiting to start USB: %d\n", i);
	// }
	// Allow pushing the button to turn on/off the onboard hub.
	bool reset = true;
	bool last = pswitch.get();
	while (1)
	{
		bool now = pswitch.get();
		//printf("switch is now %d\n", now);
		if (now != last)
		{
			last = now;
			reset = !reset;
			usb_reset.set(reset);
			printf("toggle usb_reset to: %s\n", reset ? "true" : "false");
		}
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

#if defined(__NVIC_PRIO_BITS) && (__NVIC_PRIO_BITS >= 3)
// #warning "karl nvic prio >= than 3..."
#define USB_HOST_INTERRUPT_PRIORITY (6U)
#else
// #warning "host prior 3"
#define USB_HOST_INTERRUPT_PRIORITY (3U)
#endif



int main()
{
	board_init();

	printf("laks+MCUX USB Middleware Host HID with FreeRTOS Example\n");
	// doesn't seemto make any difference when this is enabled, the handler hcecks for being init'd
	NVIC.set_priority(interrupt::irq::USB_OTG, USB_HOST_INTERRUPT_PRIORITY << configPRIO_BITS);
	NVIC.enable(interrupt::irq::USB_OTG);

	// Create soft timer for blinky, task for tinyusb stack
#if configSUPPORT_STATIC_ALLOCATION
	// blinky_tm = xTimerCreateStatic(NULL, pdMS_TO_TICKS(BLINK_MOUNTED), true, NULL, led_blinky_cb, &blinky_tmdef);
	// xTaskCreateStatic(usb_host_task, "usbh", USBH_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, usb_host_stack, &usb_host_taskdef);
	// This is for the v2400 board....
	// xTaskCreateStatic(task_late_start, "late", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, task_late_stack, &task_late_handle);
	fsl_app_main();
#else
#error "This path isn't actually used"
	blinky_tm = xTimerCreate(NULL, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), true, NULL, led_blinky_cb);
	xTaskCreate(usb_host_task, "usbd", USBH_STACK_SIZE, NULL, configMAX_PRIORITIES - 1, NULL);
	xTaskCreate(task_late_start, "xlate", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
#endif

	// xTimerStart(blinky_tm, 0);

	// not entirely convinced this is right!
	// NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	// Required to use FreeRTOS ISR methods!

	printf("configpriobits is %d, usb hirq prio %d, lib max syscall: %d\n", configPRIO_BITS, USB_HOST_INTERRUPT_PRIORITY, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	// _this _should_ be the same as
    // NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
    // EnableIRQ((IRQn_Type)irqNumber);


	vTaskStartScheduler();
}

#if 0  // at the moment, fsl is in app.c
// USB Host task
// This top level thread process all usb events and invoke callbacks
static void usb_host_task(void *param)
{

	// Just wait for bit before starting.  This makes the
	for (int i = 0; i < 5; i++)
	{
		vTaskDelay(pdMS_TO_TICKS(200));
		printf("TU WAIT USB: %d\n", i);
	}

	// // init host stack on configured roothub port
	// tusb_rhport_init_t host_init = {
	// 	.role = TUSB_ROLE_HOST,
	// 	.speed = TUSB_SPEED_AUTO
	// };

	// if (!tusb_init(BOARD_TUH_RHPORT, &host_init)) {
	// 	printf("Failed to init USB Host Stack\r\n");
	// 	vTaskSuspend(NULL);
	// }

	//board_init_after_tusb();

	// RTOS forever loop
	while (1)
	{
		USB_HostTaskFn(param);
	}
}
#endif

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
	void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
	void USB0_IRQHandler(void);


// This should work with laks interrupts... but going to have the include paths..
#define __COMPILER_BARRIER() __asm__ volatile("" ::: "memory")

// 	void NVIC_EnableIRQ(IRQn_Type IRQn)
// 	{
// 		if ((int32_t)(IRQn) >= 0)
// 		{
// 			__COMPILER_BARRIER();
// 			NVIC.enable(IRQn);
// //			NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
// 			__COMPILER_BARRIER();
// 		}
// 	}
// 	void NVIC_DisableIRQ(IRQn_Type IRQn)
// 	{
// 		if ((int32_t)(IRQn) >= 0)
// 		{
// 			//NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
// 			NVIC.disable(IRQn);
// 			__DSB();
// 			__ISB();
// 		}
// 	}

// 	void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
// 	{
// 		if ((int32_t)(IRQn) >= 0)
// 		{
// 			NVIC->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
// 		}
// 	}

// 	uint32_t NVIC_GetEnableIRQ(IRQn_Type IRQn)
// 	{
// 		if ((int32_t)(IRQn) >= 0)
// 		{
// 			return ((uint32_t)(((NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
// 		}
// 		else
// 		{
// 			return (0U);
// 		}
// 	}

	void vAssertCalled(const char *const pcFileName, unsigned long ulLine)
	{
		volatile unsigned long ulSetToNonZeroInDebuggerToContinue = 0;

		/* Parameters are not used. */
		(void)ulLine;
		(void)pcFileName;

		taskENTER_CRITICAL();
		{
			while (ulSetToNonZeroInDebuggerToContinue == 0)
			{
				/* Use the debugger to set ulSetToNonZeroInDebuggerToContinue to a
				non zero value to step out of this function to the point that raised
				this assert(). */
				__asm volatile("NOP");
				__asm volatile("NOP");
			}
		}
		taskEXIT_CRITICAL();
	}

	void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
		(void) pxTask;
		(void) pcTaskName;
		printf("lol, you crashed in %s", pcTaskName);
		while(1) {
			;
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

template <>
void interrupt::handler<interrupt::irq::USB_OTG>()
{
	// FIXME - call fsl layer.
	USB0_IRQHandler();
}
