// zyp's "I think this is my gd32v miniblink?" https://paste.jvnv.net/raw/Z30Xj
// plus zyp's "you'll need this too..." https://paste.jvnv.net/raw/14lMz


#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <syscfg/syscfg.h>
#include <uart/uart.h>
#include <usb/usb.h>
#include <usb/generic.h>
#include <usb/descriptor.h>

//// add to laks!!! 

void entry();
extern int _ram_end;

[[gnu::naked]]
[[gnu::section(".vectors")]]
void _reset_handler() {
    // Initialize stack pointer.
	asm volatile("lui sp, %%hi(%0); add sp, sp, %%lo(%0)" :: "i"(&_ram_end));
    // Absolute jump to entry function.
    asm volatile("jr %0" :: "m"(entry));
}
//// add to laks end....


#if defined(CH32V23)
Pin led = GPIOA[0];
Pin utx = GPIOA[9]; // USART1 (connected to wch-link on black eval board)
Pin urx = GPIOA[10];
auto rcc_uart = rcc::USART1;
auto my_uart = USART1;
#elif defined(CH58x)
Pin led = GPIO[(0<<8)|0];  // A0.... we might need some macros to fiddle this...
Pin utx = GPIO[(0<<8)|9]; // A9
Pin urx = GPIO[(0<<8)|8]; // A8
auto rcc_uart = rcc::UART1;
auto my_uart = UART1; // connected to P4 on the CH582M-R0-1v0 board
#elif defined(GD32V)
Pin led = GPIOA[7];
#else
#warning "unspecifed board, defaulting led to PA0"
Pin led = GPIOA[0];
#endif

auto dev_desc = device_desc(0x200, 0, 0, 0, 64, 0x1234, 0x5678, 0x110, 1, 2, 3, 1);
auto conf_desc = configuration_desc(1, 1, 0, 0xc0, 0,
	// HID interface.
	interface_desc(0, 0, 1, 0xff, 0x00, 0x00, 0,
		endpoint_desc(0x81, 0x03, 16, 1)
	)
);

desc_t dev_desc_p = {sizeof(dev_desc), (void*)&dev_desc};
desc_t conf_desc_p = {sizeof(conf_desc), (void*)&conf_desc};

//USB_otg usb(OTG_FS, dev_desc_p, conf_desc_p);

void sleep(int cnt) {
    while(cnt--) {
        asm volatile("nop");
    }
}

#if defined(CH32V23)
void uart_enable(void)
{
    RCC.enable(rcc_uart);
    uint32_t pclk = 8000000; // default is 8MHz HSI
    uint32_t baud = 115200;
    USART1->BRR = (pclk + baud / 2) / baud;
    USART1->CR1 = (1 << 3) | (1 << 2) | (1 << 13); // TE, RE, UE
    USART1->CR1 |= (1<<5); // RXNEIE
    interrupt_ctl.enable(interrupt::irq::USART1);
}
#elif defined(CH58x)
void uart_enable(void)
{
//	SYSCFG.unlock_safe();
//	RCC.enable(rcc_uart);  // FIXME - this is inverted on ch58x!

	auto rx_fifo = 0x3; // 7 bytes
	// Flush and enable fifos. (ie, 16550 mode, not 16450 mode)
	my_uart->FCR = (rx_fifo << 6) | 0x7;
	my_uart->LCR = 0x3; // 8N1
	my_uart->IER = (1<<6); // enable TXD output
	my_uart->DIV = 1;  // "standard" prescaler

	// TODOD - baud calculation depends heavily on sysclock!
	// you really need to keep on there...
	// my_uart->DL =
	auto sys = 60000000u;
	auto dl = sys * 2 / 1 / 16 / 115200;
	my_uart->DL = dl;  // 65 for 115200? (with 60Mhz in)
	

}
#else
#warning "Unsupported UART platform!"
void uart_enable(void) {}
#endif


void rcc_init();

volatile uint16_t lol_char;

int main() {
    rcc_init();

#if defined(CH58x)
#if 1
#if defined(KSANE)
    SYSCFG.unlock_safe();
    // PLL480 / 12 == 40MHz clock
    //RCC->CLK_SYS_CFG = (1<<6) | 12;
    RCC->CLK_SYS_CFG = (1<<6) | 20; // 480/32 => 15MHz
#else
    // this is madness...
    SYSCFG.unlock_safe();
    RCC->PLL_CONFIG &= ~(1<<5); // undocumented
    SYSCFG.lock_safe();
    SYSCFG.unlock_safe();
    //RCC->CLK_SYS_CFG = (1<<6) | 16; // 30Mhz, either ~4times as fast as specced, or ~double what we see...?
    RCC->CLK_SYS_CFG = (1<<6) | 8; // 60Mhz, either ~4times as fast as specced, or ~double what we see...?
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	asm volatile("nop");
	SYSCFG.lock_safe();
    SYSCFG.unlock_safe();
    FLASH->CFG = 0x52; // undocumented...
    SYSCFG.lock_safe();
    SYSCFG.unlock_safe();
    RCC->PLL_CONFIG |= (1<<7); // undocumented...  yey...
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

    uart_enable();
    
    
    int i = 0;
    int qq = 0;
    while(1) {
//        usb.process();
	qq++;
	if (qq % 80000 == 0) {
	    led.toggle();
#if defined(CH32V23)
	    my_uart.write_blocking('a' + i % 26);
#elif defined(CH58x)
	    my_uart.write_blocking('a' + i % 26);
#else
#warning "unsupported platform"
#endif
	    i++;
	}
	if (lol_char) {
		my_uart.write_blocking('[');
		my_uart.write_blocking(lol_char);
		my_uart.write_blocking(']');
		lol_char = 0;
	}
#if 0 // Polled works just fine...
	// lol, poll that shit...
	if (USART1->SR & (1<<5)) {
		uint16_t cc = USART1->DR;
		USART1.write_blocking('<');
		USART1.write_blocking(cc);
		USART1.write_blocking('>');
	}
#endif
    }

}


#if defined(CH32V23)
template <>
void interrupt::handler<interrupt::irq::USART1>() {
	if (USART1->SR & (1<<5)) {
		lol_char = USART1->DR;
	}
}
#endif

