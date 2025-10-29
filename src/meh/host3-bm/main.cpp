
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <cortex_m/debug.h>
#include <interrupt/interrupt.h>
#include <nxp_kx/mcg.h>
#include <nxp_kx/mpu.h>
#include <nxp_kx/osc.h>
#include <nxp_kx/rcm.h>
#include <nxp_kx/sim.h>
#include <nxp_kx/wdog.h>
#include <gpio/gpio.h>

// You can't have either of these, as it defines NVIC as well...
// well, if you let the the mkXX.h keep including all of cmsis, which you probably should
// #include "fsl_device_registers.h"
//#include "core_cm4.h"
#define __NVIC_PRIO_BITS 4 /**< Number of priority bits implemented in the NVIC */


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
Pin pswitch = GPIOB[4];	   // CONF button
PinMux pswitch_mux = PCRB[pswitch.n];
#else
// hope you're on a frdmk64 board!
Pin usb_reset = GPIOB[22]; // use the red led as a standin for the hub reset
PinMux usb_reset_mux = PCRB[usb_reset.n];
Pin pswitch = GPIOC[6];	   // SW2
PinMux pswitch_mux = PCRC[pswitch.n];

#endif


void laks_clock_config_for_boot(void)
{
		// redone from scratch, copying the mcux- example, _NOT_ our old tinyusb example, even though both end up with "same" clocks..
	SIM->CLKDIV1 = 0x01240000U;  // CLOCK_SetSimSafeDivs();

#if 0
const osc_config_t oscConfig_BOARD_BootClockRUN =
    {
        .freq = 50000000U,                        /* Oscillator frequency: 50000000Hz */
        .capLoad = (OSC_CAP0P),                   /* Oscillator capacity load: 0pF */
        .workMode = kOSC_ModeExt,                 /* Use external clock */
        .oscerConfig =
            {
                .enableMode = kOSC_ErClkEnable,   /* Enable external reference clock, disable external reference clock in STOP mode */
            }
    };
	    CLOCK_InitOsc0(&oscConfig_BOARD_BootClockRUN);
#endif
	OSC0.set_cap_load(0);
	MCG.set_gain_high(false);
	MCG.set_ext_ref_osc(false); // this means ext ref is ext ref, not osc..
	MCG.set_range(2); // "very high" for 120MHz
	OSC0.enable_ext_ref(true);
	if (OSC0->CR & (1<<7)) {
		while (MCG->S & (1<<1)) {
			; // wait for oscinit0
		}
	}

#if 0
const mcg_config_t mcgConfig_BOARD_BootClockRUN =
    {
        .mcgMode = kMCG_ModePEE,                  /* PEE - PLL Engaged External */
        .irclkEnableMode = kMCG_IrclkEnable,      /* MCGIRCLK enabled, MCGIRCLK disabled in STOP mode */
        .ircs = kMCG_IrcSlow,                     /* Slow internal reference clock selected */
        .fcrdiv = 0x0U,                           /* Fast IRC divider: divided by 1 */
        .frdiv = 0x0U,                            /* FLL reference clock divider: divided by 32 */
        .drs = kMCG_DrsLow,                       /* Low frequency range */
        .dmx32 = kMCG_Dmx32Default,               /* DCO has a default range of 25% */
        .oscsel = kMCG_OscselOsc,                 /* Selects System Oscillator (OSCCLK) */
        .pll0Config =
            {
                .enableMode = MCG_PLL_DISABLE,    /* MCGPLLCLK disabled */
                .prdiv = 0x13U,                   /* PLL Reference divider: divided by 20 */
                .vdiv = 0x18U,                    /* VCO divider: multiplied by 48 */
            },
    };
    CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_BootClockRUN.irclkEnableMode,
                                  mcgConfig_BOARD_BootClockRUN.ircs,
                                  mcgConfig_BOARD_BootClockRUN.fcrdiv);
#endif
	// I don't believe we care, as we don't use it for very long

	/* Configure FLL external reference divider (FRDIV). */
	// CLOCK_CONFIG_SetFllExtRefDiv(mcgConfig_BOARD_BootClockRUN.frdiv);
	// also don't care, we don't use the fll, but that's standard "all combinations clock support"

    // CLOCK_BootToPeeMode(mcgConfig_BOARD_BootClockRUN.oscsel,
    //                     kMCG_PllClkSelPll0,
    //                     &mcgConfig_BOARD_BootClockRUN.pll0Config);
	MCG->C7 = 0; // oscsel = osc0 // simpfliifed, as we're not changing dynamically
	//
	MCG->C1 = (MCG->C1 & ~((0x3<<6) | (0x1<<2))) | (2<<6);
    while ((MCG->S & ((1<<4) | (3<<2))) != ((0<<4) | (2<<2)))
    {
    }
    /* Disable PLL first, then configure PLL. */
    MCG->C6 &= (uint8_t)(~(1<<6));
    //while ((MCG->S & (1<<5)) != 0U)
	while (MCG.plls_is_pll())
    {
    }

    /* Configure the PLL. */
    {
        // CLOCK_EnablePll0(config);
		// FIXME - karl - go and recheck this for how you did it in the tueh examples...
		// ie, to make it more _useful_ to people
		// you can't put in real dividers, as the k64/k70 impls are wildly different bit sizes and factors here..
		MCG.config_pll(0x13, 0x18, 0);
    }

    /* Change to PLL mode. */
    MCG->C6 |= (1<<6);

    /* Wait for PLL mode changed. */
    //while (((MCG->S & (1<<5))) == 0U)
	while (!MCG.plls_is_pll())
    {
    }

	// Now, finally, change to use PLL output clock
	MCG.clock_source(0);  // yeah, 0 doesn't make sense to me, but ok.
	while (MCG.clock_source() != 3) {
		;
	}


// ok, back tot top level
#if 0
const sim_clock_config_t simConfig_BOARD_BootClockRUN =
    {
        .pllFllSel = SIM_PLLFLLSEL_MCGPLLCLK_CLK, /* PLLFLL select: MCGPLLCLK clock */
        .er32kSrc = SIM_OSC32KSEL_RTC32KCLK_CLK,  /* OSC32KSEL select: RTC32KCLK clock (32.768kHz) */
        .clkdiv1 = 0x1240000U,                    /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /2, OUTDIV3: /3, OUTDIV4: /5 */
    };
    CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
#endif
	SIM->CLKDIV1 = 0x1240000;

	SIM->SOPT2 = (SIM->SOPT2 & ~(0x3<<16)) | (1<<16); // PLLFLLSEL = MCGPLL
	SIM->SOPT1 = (SIM->SOPT1 & ~(0x3<<18)) | (2<<18); // ER32clk = rtc
}

void laks_usb_host_clock_init(void) {
//    CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0, CLOCK_GetFreq(kCLOCK_PllFllSelClk));
	// we skip that clock lookup by just knowing
	SIM.disable(sim::USBFS);

	// 120 * 2 / 5 == 48
	int udiv = 5;
	int ufrac = 2;
	SIM->CLKDIV2 = ((udiv - 1) << 1) | ((ufrac - 1) << 0);
	// usb src from "somethign internal" + pll from pll.
	SIM->SOPT2 = (SIM->SOPT2 & ~(0x7<<16)) | (1<<18) | (1<<16);

	SIM.enable(sim::USBFS);
}

void board_init_pins(void)
{
	// XXX: this assumes the SIM:PORTx have all been turned on...
	led1_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	led1.set_out();
	// usb_reset is faked on frdm-k64, but resets the hub on v2400
	usb_reset_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	usb_reset.set_out();
	usb_reset.on();
	// pswitch is SW2 or the conf button
	pswitch_mux.mux(NXP_PCR_KX_t::Alt1_GPIO);
	pswitch_mux.pull(true, true);
	pswitch.set_in();
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

	// NXP is "secure" by default blocking things like the usb peripheral from accessing memory
	MPU.disable();

	board_init_pins();

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

	laks_clock_config_for_boot();

	SystemCoreClock = 120000000;
	printf("Running sys clock: %lu\n", SystemCoreClock);
	printf("SIM sopt2: %lx, scgc4: %lx, clkdiv2: %lx\n", SIM->SOPT2, SIM->SCGC4, SIM->CLKDIV2);
	printf("MCG c1: %x, c2: %x, c5: %x, c6: %x\n", MCG->C1, MCG->C2, MCG->C5, MCG->C6);

	// now, separately, the app usb clock en
	laks_usb_host_clock_init();
}

void board_led_write(bool on)
{
	led1.set(on);
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


#define configPRIO_BITS 4
#if defined(__NVIC_PRIO_BITS) && (__NVIC_PRIO_BITS >= 3)
#define USB_HOST_INTERRUPT_PRIORITY (6U)
#else
#warning "You don't have access to __NVIC_PRIO_BITS like you expect!"
#define USB_HOST_INTERRUPT_PRIORITY (3U)
#endif



int main()
{
	board_init();

	setvbuf(stdout, NULL, _IONBF, 0);
	printf("laks+MCUX USB Middleware Host HID with bare metal\n");

	// I have other code that uses syscall, but should be ok as long as it's higher prior
	// NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC.set_priority(interrupt::irq::USB_OTG, USB_HOST_INTERRUPT_PRIORITY << configPRIO_BITS);
	NVIC.enable(interrupt::irq::USB_OTG);
	fsl_app_main();
}


extern "C"
{
	void USB0_IRQHandler(void);
	// extern void* g_HostHandle;

    // extern void USB_HostKhciIsrFunction(void* handle);

// This should work with laks interrupts... but going to have the include paths..
#define __COMPILER_BARRIER() __asm__ volatile("" ::: "memory")

	void EnableGlobalIRQ(uint32_t primask)
	{
		//__set_PRIMASK(primask);
		__asm volatile ("MSR primask, %0" : : "r" (primask) : "memory");

	}

	uint32_t DisableGlobalIRQ(void)
	{
		uint32_t result;

		__asm volatile ("MRS %0, primask" : "=r" (result) :: "memory");
		__asm volatile ("cpsid i" : : : "memory");

		return result;
	}

}

template <>
void interrupt::handler<interrupt::irq::USB_OTG>()
{
	// call fsl layer.
	USB0_IRQHandler();
	// this didn't help.
	// USB_HostKhciIsrFunction(g_HostHandle);
}

