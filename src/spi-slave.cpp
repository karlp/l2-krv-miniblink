
#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

#include <etl/circular_buffer.h>

#include <cal/cal.h>
#include <adc/adc.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <spi/spi.h>
#include <syscfg/syscfg.h>
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

// Default mappings for SPI0
auto my_spi = SPI0;
auto my_spi_irq = interrupt::irq::SPI0;
// CS is hardware assisted for slave mode, manual (free gpio) for master
// use the "same" pin, for easier wiring...
Pin spi_cs = GPIO[(0<<8) | 12]; // A12
Pin spi_clk = GPIO[(0<<8) | 13]; // A13
Pin spi_mosi = GPIO[(0<<8) | 14]; // A14
Pin spi_miso = GPIO[(0<<8) | 15]; // A15
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
	SYSCFG.unlock_safe();
	FLASH->CFG = flash_cfg; // undocumented...
	RCC->PLL_CONFIG |= (1 << 7); // turns on "flash rom control mode" ?

	return 480000000u / real_div;
}
#endif



// lol, you think I can parameterize this?!
void spi_init_master(void) {
	// 80e6 / 80 = 1Mhz spi for starters?
	my_spi->CLOCK_DIV = 80; // clock == fsys / this 8 bit value
	bool mode0 = true;
	my_spi->CTRL_MOD = (1 << 6) // MOSI OE
		| (1<<5) // SCK OE
		| ((mode0 ? 0 : 1) << 3) // mode 0/3 (sck idle low/high)
		| 0; // master mode, 3 wire.

	// gpio config.
	spi_cs.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);
	spi_clk.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);
	spi_mosi.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);
	spi_miso.set_mode(Pin::Input, Pin::Pull::Up, Pin::Drive::Low5);

	// that's it for config, unless you want DMA
}

void spi_init_slave(void)
{
	// clear fifo and all flags first
	my_spi->CTRL_MOD = (1<<1);
	// We don't know what cmd vs data stream really means yet...
	uint8_t fifo_input = 1;  // we want to receive initially.
	uint8_t cmd_mode = 1; // 0 == stream mode, whatever that is..
	my_spi->CTRL_MOD = (1<<7) // MISO OE
		| (fifo_input << 4)
		| (cmd_mode << 3)
		| 1; // slave mode, 3 wire.
	// vendor sdk enables "auto" in ctrl-cfg, which auto clears byte
	// complete flags when data reg is read...
	
	// gpio config. All inputs.  miso switches automatically
	// based on MISO_OE, and this supports single and multi device.
	spi_cs.set_mode(Pin::Input, Pin::Pull::Up, Pin::Drive::Low5);
	spi_clk.set_mode(Pin::Input, Pin::Pull::Up, Pin::Drive::Low5);
	spi_mosi.set_mode(Pin::Input, Pin::Pull::Up, Pin::Drive::Low5);
	spi_miso.set_mode(Pin::Input, Pin::Pull::Up, Pin::Drive::Low5);

	my_spi->PERIPH_PRE = 0xbe;
	// no, put nothing in fifo ourselves.
	// my_spi->FIFO = 0x99; // do we need one byte for the fifo as well?
	// for us, for now, we're using fifo for tx... (register read style)
	//my_spi.fifo_out(true);
	//my_spi.command_mode(true); // vs datastream mode?!
	// cmd mode vs datastream mode (CTRL_MOD
}

volatile uint8_t my_spi_req;
volatile uint16_t spi_periph_regs[10];
auto spi_rxb = etl::circular_buffer<uint8_t, 10>();
volatile bool my_spi_busy_tx = false;
volatile bool my_spi_busy_rx = false;

int main()
{
	rcc_init();
	for (auto i = 0; i < 10; i++) {
		// Spi regs == 0x6606 for register 6...
		spi_periph_regs[i] = 0xa << 12 | i << 8 | i << 4 | 0x5;
	}

#if defined(CH58x)
	// GPIOS are always clocked?
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
	printf("zzz div set and sysclock to %lu (ss: %x)\n", sys_speed, ss);

	//spi_init_master();
	spi_init_slave();
	my_spi->INT_EN |= (1<<7); // slave first byte irq
	interrupt_ctl.enable(my_spi_irq);


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
		}
		if (my_spi_busy_tx) {
			while (my_spi->FIFO_COUNT != 0)
				;
			my_spi_busy_tx = false;
			// return to input mode
			my_spi.fifo_out(false);
			interrupt_ctl.enable(my_spi_irq);
		}
		if (my_spi_busy_rx) {
			// ideally should validate nothing remainining in fifo?
			// ideally should reset on every CS release?
			if (my_spi->FIFO_COUNT == 2) {
				// lol, is this legit? just run fast and wait til we have what we expect?
				uint8_t a = my_spi->FIFO;
				uint8_t b = my_spi->FIFO;
				spi_periph_regs[my_spi_req] = a << 8 | b;
				my_spi_busy_rx = false;
				interrupt_ctl.enable(my_spi_irq);
			}
		}

	}

}


template <>
void interrupt::handler<interrupt::irq::SPI0>()
{
	my_spi->INT_FLAG = (1<<7); // clear FST_BYTE
	// turn off interrupts event? (here, or in periph?)
	interrupt_ctl.disable(my_spi_irq);
	// must read from fifo, not BUFFER to properly count
	uint8_t incmd = my_spi->FIFO;
	uint8_t reg = incmd & 0x3f;
	if (incmd & 0x80) {
		// leave the fifo in IN mode, mark us as busy, and let
		// task space read it out and take action
		my_spi_busy_rx = true;
		my_spi_req = reg;
	} else {
		my_spi_busy_tx = true;
		my_spi.fifo_out(true);
		my_spi->FIFO = spi_periph_regs[reg] >> 8;
		my_spi->FIFO = spi_periph_regs[reg] & 0xff;
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

