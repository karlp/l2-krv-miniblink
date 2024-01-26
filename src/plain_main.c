

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


int plain_main(void)
{

	// LED0_EN;
	// LED1_EN;
	// LED2_EN;
	// LED3_EN;

	GPIOA_PDDR=GPIO_PDDR_PDD(GPIO_PIN(10) | GPIO_PIN(11) | GPIO_PIN(28) | GPIO_PIN(29) );  // this was missing!

	LED0_OFF;
	LED1_ON;
	LED2_ON;
	LED3_ON;

	const int LITTLE_BIT = 0x200000;
	while (1)
	{
		for (int i = 0; i < LITTLE_BIT; i++)
		{
			__asm__("nop");
		}
		toggle_all();
	}
}