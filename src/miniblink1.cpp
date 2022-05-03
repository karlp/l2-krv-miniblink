// zyp's "I think this is my gd32v miniblink?" https://paste.jvnv.net/raw/Z30Xj
// plus zyp's "you'll need this too..." https://paste.jvnv.net/raw/14lMz


#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

#include <adc/adc.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <syscfg/syscfg.h>
#include <uart/uart.h>

//// add to laks!!! 

void entry();
extern int _ram_end;

[[gnu::naked]]
[[gnu::section(".vectors")]]
void _reset_handler()
{
	// Initialize stack pointer.
	asm volatile("lui sp, %%hi(%0); add sp, sp, %%lo(%0)" ::"i"(&_ram_end));
	// Absolute jump to entry function.
	asm volatile("jr %0" ::"m"(entry));
}
//// add to laks end....


#if defined(CH32V23)
Pin led = GPIOA[0];
Pin utx = GPIOA[9]; // USART1 (connected to wch-link on black eval board)
Pin urx = GPIOA[10];
auto rcc_uart = rcc::USART1;
auto my_uart = USART1;
#elif defined(CH58x)
Pin led = GPIO[(0 << 8) | 0]; // A0.... we might need some macros to fiddle this...
Pin utx = GPIO[(0 << 8) | 9]; // A9
Pin urx = GPIO[(0 << 8) | 8]; // A8
auto rcc_uart = rcc::UART1;
auto my_uart = UART1; // connected to P4 on the CH582M-R0-1v0 board
#elif defined(GD32V)
Pin led = GPIOA[7];
#else
#warning "unspecifed board, defaulting led to PA0"
Pin led = GPIOA[0];
#endif


extern "C" int _write(int file, char* ptr, int len) {
        int i;

        if (file == STDOUT_FILENO || file == STDERR_FILENO) {
                for (i = 0; i < len; i++) {
                        if (ptr[i] == '\n') {
                                my_uart.write_blocking('\r');
                        }
                        my_uart.write_blocking(ptr[i]);
                }
                return i;
        }
        errno = EIO;
        return -1;
}

#if defined(CH32V23)

void uart_enable(void)
{
	RCC.enable(rcc_uart);
	uint32_t pclk = 8000000; // default is 8MHz HSI
	uint32_t baud = 115200;
	USART1->BRR = (pclk + baud / 2) / baud;
	USART1->CR1 = (1 << 3) | (1 << 2) | (1 << 13); // TE, RE, UE
	USART1->CR1 |= (1 << 5); // RXNEIE
	interrupt_ctl.enable(interrupt::irq::USART1);
}
#elif defined(CH58x)

void uart_enable(uint32_t sys)
{
	//	SYSCFG.unlock_safe();
	//	RCC.enable(rcc_uart);  // FIXME - this is inverted on ch58x!

	auto rx_fifo = 0x3; // 7 bytes
	// Flush and enable fifos. (ie, 16550 mode, not 16450 mode)
	my_uart->FCR = (rx_fifo << 6) | 0x7;
	my_uart->LCR = 0x3; // 8N1
	my_uart->IER = (1 << 6); // enable TXD output
	my_uart->DIV = 1; // "standard" prescaler

	auto dl = sys * 2 / 1 / 16 / 115200;
	my_uart->DL = dl;
	my_uart->IER |= (1 << 0); // RECV_RDY irqs
	my_uart->MCR |= (1 << 3); // peripheral IRQ enable..
	interrupt_ctl.enable(interrupt::irq::UART1);
}
#else
#warning "Unsupported UART platform!"

void uart_enable(void)
{
}
#endif


void rcc_init();

volatile uint16_t lol_char;
uint8_t ss;

#if defined(CH58x)

uint32_t rcc_set_pll(uint8_t div)
{
	int real_div = div;
	if (div > 31) {
		div = 0;
		real_div = 32;
	}
	// From recent SDK updates!
	uint8_t flash_cfg = 0x52;
	if (div < 8) {
		flash_cfg = 0x02;
	}
	// this is madness...
	SYSCFG.unlock_safe();
	RCC->PLL_CONFIG &= ~(1 << 5); // undocumented
	RCC->CLK_SYS_CFG = (1 << 6) | (div);
//	asm volatile("nop");
//	asm volatile("nop");
//	asm volatile("nop");
//	asm volatile("nop");
	ss = SYSCFG->SAFE_ACCESS_SIG;  // just looking at timing. can be removed.
	SYSCFG.unlock_safe();  // you _probably_ still need this one!
	FLASH->CFG = flash_cfg; // undocumented...
	RCC->PLL_CONFIG |= (1 << 7); // undocumented...  yey...

	return 480000000u / real_div;
}
#endif

int main()
{
	rcc_init();

#if defined(CH58x)
#if 0
#if defined(KSANE)
	SYSCFG.unlock_safe();
	// PLL480 / 12 == 40MHz clock
	//RCC->CLK_SYS_CFG = (1<<6) | 12;
	RCC->CLK_SYS_CFG = (1 << 6) | 20; // 480/32 => 15MHz
#else
	// this is madness...
	SYSCFG.unlock_safe();
	RCC->PLL_CONFIG &= ~(1 << 5); // undocumented
	SYSCFG.lock_safe();
	SYSCFG.unlock_safe();
	//RCC->CLK_SYS_CFG = (1<<6) | 16; // 30Mhz, either ~4times as fast as specced, or ~double what we see...?
	RCC->CLK_SYS_CFG = (1 << 6) | 8; // 60Mhz, either ~4times as fast as specced, or ~double what we see...?
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	SYSCFG.lock_safe();
	SYSCFG.unlock_safe();
	FLASH->CFG = 0x52; // undocumented...
	SYSCFG.lock_safe();
	SYSCFG.unlock_safe();
	RCC->PLL_CONFIG |= (1 << 7); // undocumented...  yey...
	SYSCFG.lock_safe();
#endif
#endif

	// GPIOS are always clocked?
#else
	RCC.enable(rcc::GPIOA);
#endif
	//    RCC.enable(rcc::USBFS);
	//
	//    usb.init();

	//sleep(10);

	led.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);

#if defined(CH58x)
	utx.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);
	urx.set_mode(Pin::Input, Pin::Pull::Up, Pin::Drive::Low5);
#else
	utx.set_mode(Pin::AF);
	urx.set_mode(Pin::AF);
#endif
	uint32_t bspeed = 16000000u;
	uart_enable(bspeed);
	printf("Booted at %lu\n", bspeed);

	int i = 0;
	int qq = 0;
	while (1) {
		// Will actually crash and restart when div > 24/25!
		for (auto div = 6; div < 20; div+=2) {
			uint32_t sys_speed = rcc_set_pll(div);
			uart_enable(sys_speed);
			printf("zzz div to %d and sysclock to %lu (ss: %x)\n", div, sys_speed, ss);
			SYSCFG.unlock_safe();
			uint8_t ss1 = SYSCFG->SAFE_ACCESS_SIG;
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			uint8_t ss2 = SYSCFG->SAFE_ACCESS_SIG;
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			uint8_t ss3 = SYSCFG->SAFE_ACCESS_SIG;
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			asm volatile ("nop");
			uint8_t ss4 = SYSCFG->SAFE_ACCESS_SIG;
			printf("slow? %02x:%02x:%02x:%02x\n", ss1, ss2, ss3, ss4);
			i = 0;
			while (i < 30) {
				qq++;
				if (qq % 80000 == 0) {
					led.toggle();
					my_uart.write_blocking('A' + i % 26);
					i++;
				}
				if (lol_char) {
					my_uart.write_blocking('[');
					my_uart.write_blocking(lol_char);
					my_uart.write_blocking(']');
					lol_char = 0;
				}
			}

			ADC->TEMP |= (1<<7); // enable temperature sensor
			// ~arbitrary clock selection,
			uint8_t clk_div = 2;
			uint8_t pga_gain = 3;
			// Set RB_ADC_POWER_ON to 1 to enable the ADC,
			// set RB_ADC_DIFF_EN to 1, 
			// set RB_ADC_CLK_DIV,
			// set RB_ADC_BUF_EN to 1,
			// and set RB_ADC_PGA_GAIN to 11;
			ADC->CFG = (clk_div << 6) | (pga_gain << 4) | (1<<2) | (1<<1) | (1<<0);

			// Set ADC_START to ... start ;)
			uint16_t samps[4];
			for (auto i = 0; i < 4; i++) {
				ADC->CONVERT = 1;
				while (ADC->CONVERT);
				// or, apparently also,
				//while (!ADC->INT_FLAG & (1<<7));
				uint16_t data = ADC->DATA;
				samps[i] = data;
			}

			printf("ADC temp sensor = %d %d %d %d\n", samps[0], samps[1], samps[2], samps[3]);


			

		}
#if 0 // Polled works just fine...
		// lol, poll that shit...
		if (my_uart.rxne()) {
			uint16_t cc = my_uart.read();
			my_uart.write_blocking('<');
			my_uart.write_blocking(cc);
			my_uart.write_blocking('>');
		}
#endif
	}

}


#if defined(CH32V23)

template <>
void interrupt::handler<interrupt::irq::USART1>()
{
	if (USART1->SR & (1 << 5)) {
		lol_char = USART1->DR;
	}
}
#elif defined(CH58x)

template <>
void interrupt::handler<interrupt::irq::UART1>()
{
	volatile uint8_t flags = my_uart->IIR & 0xf;
	if (flags == my_uart.RxData || flags == my_uart.RxTimeOut) {
		lol_char = my_uart.read();
	} else {
		// unhandled, might need to read LSR, IIR or MSR!
		while (1) {
			asm volatile ("nop");
		}
	}
}

#endif

