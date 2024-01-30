
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

// #include <gpio/gpio.h>
#include <interrupt/interrupt.h>
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
	extern void kk_main(void);

	void enable_irq(int irq) {
		NVIC.enable((interrupt::irq)irq);
	}
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
	WDOG.unlock();
	WDOG.disable();
	uint8_t reason0 = RCM->SRS0;
	uint8_t reason1 = RCM->SRS1;
	if (reason0 == 0x42 && reason1 == 0x69) {
		while (1)
		;
	}
	// lol
	SIM.enable(sim::PORTA);
	SIM.enable(sim::PORTB);
	SIM.enable(sim::PORTC);
	SIM.enable(sim::PORTD);
	SIM.enable(sim::PORTE);
	SIM.enable(sim::PORTF);

	printf("jumping to original\n");
	kk_main();
}

extern "C" {
	void Timer_ISR(void);
	void USB_ISR(void);
}

template <>
void interrupt::handler<interrupt::irq::LPTimer>()
{
	Timer_ISR();
}

template <>
void interrupt::handler<interrupt::irq::USB_OTG>()
{
	USB_ISR();
}
