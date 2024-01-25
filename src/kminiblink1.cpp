
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



#if defined(TWRK70)
// Pin led = GPIOA[10];  // Blue
// Pin led2 = GPIOA[11]; // orange
// Pin led3 = GPIOA[29]; // green

#else
#warning "unspecifed board, defaulting led to PA0"
#endif

extern "C" {
	extern int plain_main(void);
}

int main()
{
	// lol
	plain_main();
}

