
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>


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
//#include "fsl_device_registers.h"
//#include "core_cm4.h"
#define __NVIC_PRIO_BITS 4 /**< Number of priority bits implemented in the NVIC */



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

TimerHandle_t blinky_tm;

static void led_blinky_cb(TimerHandle_t xTimer);

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
	printf("laks+MCUX USB Middleware Host HID with FreeRTOS Example2\n");

	fsl_app_main();
	// I have other code that uses syscall, but should be ok as long as it's higher prior
	// NVIC_SetPriority(USB0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	NVIC.set_priority(interrupt::irq::USB_OTG, USB_HOST_INTERRUPT_PRIORITY << configPRIO_BITS);
	NVIC.enable(interrupt::irq::USB_OTG);
	blinky_tm = xTimerCreate(NULL, pdMS_TO_TICKS(BLINK_NOT_MOUNTED), true, NULL, led_blinky_cb);
	xTimerStart(blinky_tm, 0);
	printf("configpriobits is %d, usb hirq prio %d, lib max syscall: %d\n", configPRIO_BITS, USB_HOST_INTERRUPT_PRIORITY, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	vTaskStartScheduler();
	while(1) {
		;
	}
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
	void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
	void USB0_IRQHandler(void);
	// extern void* g_HostHandle;

    // extern void USB_HostKhciIsrFunction(void* handle);

// This should work with laks interrupts... but going to have the include paths..
#define __COMPILER_BARRIER() __asm__ volatile("" ::: "memory")

	// void NVIC_EnableIRQ(IRQn_Type IRQn)
	// {
	// 	if ((int32_t)(IRQn) >= 0)
	// 	{
	// 		__COMPILER_BARRIER();
	// 		NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
	// 		__COMPILER_BARRIER();
	// 	}
	// }
	// void NVIC_DisableIRQ(IRQn_Type IRQn)
	// {
	// 	if ((int32_t)(IRQn) >= 0)
	// 	{
	// 		NVIC->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
	// 		__DSB();
	// 		__ISB();
	// 	}
	// }

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

	int32_t lol_cyccnt(void) {
		return DWT->CYCCNT;
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
	// call fsl layer.
	USB0_IRQHandler();
	// this didn't help.
	// USB_HostKhciIsrFunction(g_HostHandle);
}


/*
## before plug, us:
(gdb) x /2wx 0xE000E100
0xe000e100:	0x00000000	0x00200000
 Ok, usb irq on in nvic
gdb) x /xb 0x40072084
0x40072084:	0x44
  softok andattach are enabled inINTEN
  erren is 0.
istat is 0. (which is why we get no irq...)
no. this was a red herring. we needed setvbuf to turn off line buffering.
We were just not getting prints in rtt. fucking noise.


So it's just...
we get:

[64]h1[4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4]h28[8]h4[F-4096]
UHD:nS=2 diS:2 stat:11
h20[8]h4[F-[4]4096]
UHD:nS=2 diS:2 stat:11
h28[8]h4[F-4096]
UHD:nS=2 diS:2 stat:11
h20[8][4]h4[F-4096]
UHD:nS=2 diS:2 stat:11
Hevt: b0005
enumeration failed

(64 is attach, h1 is attach posted, h28 is "msg"+"sof_tok")
8 is tokndone h4 is token done.


They get:

[4]h8S[64]h1[4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4]h2[8]8[4]hc[8][4]hc[8][4]h4UHD:[4]4]=1 diS:2[4] stat:0
   h28[4][8]hc[8][4]hcUHD[4]:nS=1 di[4]S:3 stat[4]:0
[4][4]h28[8][4]hc[8][4]hc[8][4]hc[8][4]hc[8][4]h4UHD:nS[4]=1 diS:4[4] stat:0
   h28[4][8]hc[8][4]hc[8][4]hc[8][4]hcUHD:[4]nS=1 diS[4]:5 stat:[4]0
[4][4]h8[8]h4[4]h8[8][4]h4h8[8][4]hc[8][4]hc[8][4]h4h8[8][4]hc[8][4]h4UHD:nS[4]=1 diS:6[4] stat:0
   Hevt:[4] 1
[4][4]h8[8]h4[4]h8[8][4]h4UHD:nS[4]=1 diS:7[4] stat:0
   Hevt: [4]3
hid m[4]ouse at[4]tached:p[4]id=0x34[4]vid=0x1c4[4]f addre[4]ss=1
h8[4]h8mous[4]h8[4]h8e atta[4]h8[4]h8ched


So that's an extra h8S up front...
h8 is token done, with S being no devices attached?


FUCK THIS NOISE

laks+MCUX USB Middleware Host HID with FreeRTOS Example2
host init done
configpriobits is 4, usb hirq prio 6, lib max syscall: 2
[4]h8S

[4]h8S[64]h1[4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4]h28[8]h4[F-4096]

them:
lol, starting nxp demo
Running sys clock: 120000000
SIM sopt2: 51000, scgc4: f0140030, clkdiv2: 9
MCG c1: 2, c2: e0, c5: 53, c6: 58
configpriobits is 4, usb hirq prio 6, lib max syscall: 2
[4]h8S

[64]h1[4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4][4]h28[8]h4[8]h4[8]h4UHD:nS=1[4] diS:2 stat:0
h28[8]h4[8]h4UHD:nS=1 diS:3 stat:0
[4][4]h28[8]h4[8]h4[8]h4[4][8]h8h4[8]h4UHD:nS=1 diS:4 stat:0
h20[8]h4[4][8]h8h4[8]h4[8]h4UHD:nS=1 diS:5 stat:0
h20[8][4]h4h8[8]h4[8]h4[8]h4[8][4]h4h8[8]h4[8]h4UHD:nS=1 diS:6 stat:0
Hevt: 1
h20[4]h8[8]h4[8]h4UHD:nS=1 diS:7 stat:0
Hevt: 3
hid mouse attached:pid=0x34vid=0x1c4f addres[4]]=1



Them....
lol, starting nxp demo
Running sys clock: 120000000
SIM sopt2: 51000, scgc4: f0140030, clkdiv2: 9
MCG c1: 2, c2: e0, c5: 53, c6: 58
configpriobits is 4, usb hirq prio 6, lib max syscall: 2
<h8>S

<h1><h28><h4><h4><h4>UHD:nS=1 diS:2 stat:0
<h28><h4><h4>UHD:nS=1 diS:3 stat:0
<h28><h4><h4><h4><h4><h8><h4>UHD:nS=1 diS:4 stat:0
<h20><h4><h4><h8><h4><h4>UHD:nS=1 diS:5 stat:0
<h20><h4><h8><h4><h4><h4><h4><h8><h4><h4>UHD:nS=1 diS:6 stat:0
Hevt: 1
<h20><h4><h8><h4>UHD:nS=1 diS:7 stat:0
Hevt: 3
hid mouse attached:pid=0x34vid=0x1c4f address=1
<h8>mouse attached

Us...
laks+MCUX USB Middleware Host HID with FreeRTOS Example2
host init done
configpriobits is 4, usb hirq prio 6, lib max syscall: 2
<h8>S   SOF with S means SOF with no devices attached.

<h1><h28><h4>[F-4096]      attach, msg+sof, tokdone. then a USB_KHCI_ATOM_TR_BUS_TIMEOUT
UHD:nS=2 diS:2 stat:11
<h20><h4>[F-4096]
UHD:nS=2 diS:2 stat:11
<h28><h4>[F-4096]
UHD:nS=2 diS:2 stat:11

h1 is attach
h2 is reset
h4 is tok done
h8 is SOF
h20 is msg

I have
(gdb) p /x &bdt
$3 = 0x1fff0600
(gdb)
and after failed attach
(gdb) p /x bdt
$9 = {0x0, 0x0, 0x0, 0x0, 0x80000, 0x1fff0200, 0x80000, 0x1fff0200, 0x0 <repeats 120 times>}
(gdb)
Why is my endpoint 0 all just empty.
NO not ep 0! we're fucking host, it's "pipes"


Thave have:
(gdb) p /x &bdt
$4 = 0x1fff0a00
(gdb)
and after successful attach

each direction needs 2x 8byte BDs (even/odd)

so first _eight_: [ep0+even+in|ep0+odd+in|ep0+even+out|ep0+odd+out]...
(each entry is 7 bytes, 2 entries, )
(gdb) p /x bdt
$6 = {0x40, 0x1fff0220, 0x68, 0x1fff0220, 0x48, 0x0, 0x80008, 0x1fff91f8, 0x0 <repeats 120 times>}
(gdb)

*/