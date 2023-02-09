
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <spi/spi.h>
#include <syscfg/syscfg.h>
#include <timer/timer.h>
#include <uart/uart.h>



#if defined(CH32V23)
#error "this example doesn't support that yet..."
#elif defined(CH58x)
#if defined(CH582_DEV_BOARD)
Pin led = GPIO[(0 << 8) | 5]; // A5 // needs wiring externally to led1/led2
Pin utx = GPIO[(0 << 8) | 9]; // A9
Pin urx = GPIO[(0 << 8) | 8]; // A8
auto rcc_uart = rcc::UART1;
auto my_uart = UART1; // connected to P4 on the CH582M-R0-1v0 board
auto my_uart_irq = interrupt::irq::UART1;

// PB2/3 == PWM8/9 _alternate_
Pin pz1 = GPIO[(1<<8) | 2]; // B2
Pin pz2 = GPIO[(1<<8) | 3]; // B3
auto my_pwm = PWM;
auto my_pwm_ch1 = 8;
auto my_pwm_ch2 = 9;
#else
#error "unknown ch58x board"
#endif

#else
#error "gotta pick a board boyo"
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

	auto dl = sys * 2 / 1 / 16 / 115200; // this probably doesn't round perfectly?
	my_uart->DL = dl;
	my_uart->IER |= (1 << 0); // RECV_RDY irqs
	my_uart->MCR |= (1 << 3); // peripheral IRQ enable..
	interrupt_ctl.enable(my_uart_irq);
}
#else
#warning "Unsupported UART platform!"

void uart_enable(void)
{
}
#endif


void rcc_init();
volatile uint16_t lol_char;


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
	SYSCFG.unlock_safe();
	FLASH->CFG = flash_cfg; // undocumented...
	RCC->PLL_CONFIG |= (1 << 7); // turns on "flash rom control mode" ?

	return 480000000u / real_div;
}
#endif




int main()
{
	rcc_init();

#if defined(CH58x)
	// GPIOS are always clocked?
	// yes, and we need to wrangle the templates to invert!
#else
	RCC.enable(rcc::GPIOA);
#endif

	led.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);

#if defined(CH58x)
	utx.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);
	urx.set_mode(Pin::Input, Pin::Pull::Up, Pin::Drive::Low5);
#else
	utx.set_mode(Pin::AF);
	urx.set_mode(Pin::AF);
#endif
	uint32_t sys_speed = rcc_set_pll(6); // 480/6 => 80MHz...
	uart_enable(sys_speed);
	printf("zzz div set and sysclock to %lu\n", sys_speed);


	pz1.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);
	pz2.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);
	GPIO->ALTERNATE |= (1<<10); // remaps pwm4,5,7,8,9!
	// This is actually 1MHZ for the whole set, so with 256steps
	// you get 3.91kHz.  You can use 5..8 bits of step data.
	int full = 0;
	//full = 256;
	full = 32;
	my_pwm->CLOCK_DIV = 80;  // 1MHz seems like plenty for right now
	my_pwm.set(my_pwm_ch1, full/4); // let's got for 25/75 for starters?
	my_pwm.enable(my_pwm_ch1);
	my_pwm.set(my_pwm_ch2, full/4);
	my_pwm.enable(my_pwm_ch2);
	my_pwm.polarity_active_low(my_pwm_ch2, false);
	// "Interleaved" "staggered"
	// appears to halve the clock rate,
	// without interleaved, 1mhz = 256, div 4, you get
	// 256usecs period, with 64usecs high, both channels phase locked
	// with interleaved, you _still_ get 64usec pulses on each channel,
	// but now they're twice as far apart, 512usecs period, with the
	// other channel firing at 256usecs.  
	//my_pwm->CONFIG |= (1<<6);
	my_pwm.data_bits(PWM.Five);
	//my_pwm->CONFIG |= 1; // n-1 cycles... if you need that sort of thing.


	int i = 0;
	int qq = 0;
	while (1) {
		qq++;
		if (qq % 800000 == 0) {
			led.toggle();
//			uint8_t x = 'a' + i % 26;
//			my_uart.write_blocking(x);
			printf("tick: %d\n", i);
			i++;
			my_pwm.set(my_pwm_ch, 256/(i%8));// lol
		}

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
#if defined(K_VEITTUR)
template <>
void interrupt::handler<interrupt::irq::UART0>()
#elif defined(CH582_DEV_BOARD)
template <>
void interrupt::handler<interrupt::irq::UART1>()
#else
#error "unsupported ch58x board"
#endif
{
	volatile uint8_t flags = my_uart->IIR & 0xf;
	if (flags == my_uart.RxData || flags == my_uart.RxTimeOut) {
#if 0 // poll LSR data ready
		while (my_uart->LSR & 1) {
			// consume all available chars to properly handle the irq?
			lol_char = my_uart.read();
		}
#endif
#if 1 // poll fifo count?
		while (my_uart->RFC) {
			// consume all available chars to properly handle the irq?
			lol_char = my_uart.read();
		}
#endif
	} else {
		// unhandled, might need to read LSR, IIR or MSR!
		while (1) {
			asm volatile ("nop");
		}
	}
}

#endif

