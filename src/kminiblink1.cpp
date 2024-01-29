
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

// #include <gpio/gpio.h>
// #include <interrupt/interrupt.h>
// #include <rcc/flash.h>
// #include <rcc/rcc.h>
// #include <syscfg/syscfg.h>
// #include <uart/uart.h>

#include <nxp_kx/rcm.h>
#include <nxp_kx/sim.h>
#include <nxp_kx/wdog.h>
#include <gpio/gpio.h>

//#if defined(TWRK70)
Pin led0 = GPIOA[11]; // orange
Pin led1 = GPIOA[28]; // yellow
Pin led2 = GPIOA[29]; // green
Pin led3 = GPIOA[10];  // Blue

// #else
// #warning "unspecifed board, defaulting led to PA0"
// #endif

extern "C" {
	extern int plain_main(void);
}

extern void laks_entry(void);

void entry(void) {
	// MUST be early!
	WDOG.unlock();
	WDOG.disable();

	laks_entry();	
}

int main()
{
	// Too late to turn off watchdog here!

	uint8_t reason0 = RCM->SRS0;
	uint8_t reason1 = RCM->SRS1;
	if (reason0 == 0x42 && reason1 == 0x69) {
		while (1)
		;
	}
	printf("boot reset reasons: %x %x\n", reason0, reason1);
	// lol
	SIM.enable(sim::PORTA);
	SIM.enable(sim::PORTB);
	SIM.enable(sim::PORTC);
	SIM.enable(sim::PORTD);
	SIM.enable(sim::PORTE);
	SIM.enable(sim::PORTF);

	// FIXME - this is gross.  want to use a ?friend? class
	// or something to get the right info out of the "Pin"?
	PCRA.mux(10, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	PCRA.mux(28, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	PCRA.mux(29, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	PCRA.mux(11, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);

	led0.set_out();
	led1.set_out();
	led2.set_out();
	led3.set_out();
	led0.off();
	led1.on();
	led2.off();
	led3.on();

	int qq = 0;
	while (1) {
		qq++;
		if (qq % 800000 == 0) {
			printf("tick: %d\n", qq);
			led0.toggle();
			led1.toggle();
			led2.toggle();
			led3.toggle();
		}
	}

	// int q = 0;
	// while (1) {
	// 	q++;
	// }

//	plain_main();
}

