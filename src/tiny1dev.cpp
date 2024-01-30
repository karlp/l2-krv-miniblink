
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

#include <interrupt/interrupt.h>

#include <nxp_kx/mcgx.h>
#include <nxp_kx/osc.h>
#include <nxp_kx/rcm.h>
#include <nxp_kx/sim.h>
#include <nxp_kx/wdog.h>
#include <gpio/gpio.h>

#include "tusb.h"
#include "bsp/board_api.h"

// #if defined(TWRK70)
Pin led0 = GPIOA[11]; // orange
Pin led1 = GPIOA[28]; // yellow
Pin led2 = GPIOA[29]; // green
Pin led3 = GPIOA[10]; // Blue

// #else
// #warning "unspecifed board, defaulting led to PA0"
// #endif

extern "C"
{
	extern void sysinit(void);
	extern void tu_init(void);

	void enable_irq(int irq)
	{
		NVIC.enable((interrupt::irq)irq);
	}

	void board_led_write(bool state)
	{
		led0.set(state);
	}

	uint32_t board_button_read(void)
	{
		return true; // FIXME
	}

	void board_init(void)
	{
		tu_init(); // my init.
	}
}

extern void laks_entry(void);

void entry(void)
{
	// MUST be early!
	WDOG.unlock();
	WDOG.disable();

	laks_entry();
}

int main()
{
	uint8_t reason0 = RCM->SRS0;
	uint8_t reason1 = RCM->SRS1;
	if (reason0 == 0x42 && reason1 == 0x69)
	{
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
	PCRA.mux(10, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	PCRA.mux(28, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	PCRA.mux(29, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);
	PCRA.mux(11, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);

	led0.set_out();
	led1.set_out();
	led2.set_out();
	led3.set_out();

	printf("tinyusb lol1\n");
	board_init();
	tud_init(BOARD_TUD_RHPORT);

	// tusb_init(); // stack init

	while (1)
	{
		tud_task();
	}
}

template <>
void interrupt::handler<interrupt::irq::USB_OTG>()
{
	// USB_ISR();
	// tud_int_handler(0);
	dcd_int_handler(0);
}
