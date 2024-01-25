

#include "MK70F15.h"
#include "twr-k70f120m.h"
#define GPIO_PIN(x)	(1<<x)

int toggle_all(void) {
	LED0_TOGGLE;
	LED1_TOGGLE;
	LED2_TOGGLE;
	LED3_TOGGLE;
//     GPIOA_PTOR|=GPIO_PDOR_PDO(GPIO_PIN(29));
	return 0;
}


void wdog_disable(void)
{
	WDOG_UNLOCK = 0xC520;
	WDOG_UNLOCK = 0xD928;

	WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN_MASK;
}


int plain_main(void)
{
	// LOL, watchdog in kinetis defaults to ON!
	wdog_disable();

	/*
	 * Enable all of the port clocks. These have to be enabled to configure
	 * pin muxing options, so most code will need all of these on anyway.
	 */
	SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTF_MASK);

	LED0_EN;
	LED1_EN;
	LED2_EN;
	LED3_EN;

	GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );  // this was missing!

	LED2_ON;
	LED3_ON;

	const int LITTLE_BIT = 0x600000;
	while (1)
	{
		for (int i = 0; i < LITTLE_BIT; i++)
		{
			__asm__("nop");
		}
		toggle_all();
	}
}