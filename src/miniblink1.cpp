// zyp's "I think this is my gd32v miniblink?" https://paste.jvnv.net/raw/Z30Xj
// plus zyp's "you'll need this too..." https://paste.jvnv.net/raw/14lMz


#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
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
#if 0
Pin utx = GPIOA[2]; // USART2
Pin urx = GPIOA[3];
#else
Pin utx = GPIOA[9]; // USART1 (connected to wch-link on black eval board)
Pin urx = GPIOA[10];
#endif
#elif defined(GD32V)
Pin led = GPIOA[7];
#elif defined(CH58x)
Pin led = GPIO[(0<<8)|0];  // A0.... we might need some macros to fiddle this...
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

void rcc_init();

volatile uint16_t lol_char;

int main() {
    rcc_init();

#if defined(CH58x)
    // GPIOS are always clocked?
#else
    RCC.enable(rcc::GPIOA);
#endif
//    RCC.enable(rcc::USBFS);
//
//    usb.init();

    //sleep(10);

    led.set_mode(Pin::Output);

    utx.set_mode(Pin::AF);
    urx.set_mode(Pin::AF);

    /*
        usart_set_baudrate(hw_details.mb_port->usart, ulBaudRate);
        usart_set_flow_control(hw_details.mb_port->usart, USART_FLOWCONTROL_NONE);
        usart_set_mode(hw_details.mb_port->usart, USART_MODE_TX_RX);

        usart_set_stopbits(hw_details.mb_port->usart, stop_bits);

	usart_set_parity(hw_details.mb_port->usart, USART_PARITY_NONE);
        usart_disable_rx_interrupt(hw_details.mb_port->usart);
        usart_disable_tx_interrupt(hw_details.mb_port->usart);
        nvic_enable_irq(hw_details.mb_port->nvic_usart);
        usart_enable(hw_details.mb_port->usart);
     * */

    RCC.enable(rcc::USART1);
    uint32_t pclk = 8000000; // default is 8MHz HSI
    uint32_t baud = 115200;
    USART1->BRR = (pclk + baud / 2) / baud;
    USART1->CR1 = (1 << 3) | (1 << 2) | (1 << 13); // TE, RE, UE
    USART1->CR1 |= (1<<5); // RXNEIE
    interrupt_ctl.enable(interrupt::irq::USART1);
    
    
    int i = 0;
    int qq = 0;
    while(1) {
//        usb.process();
	qq++;
	if (qq % 80000 == 0) {
	    led.toggle();
	    USART1.write_blocking('a' + i % 26);
	    i++;
	}
	if (lol_char) {
		USART1.write_blocking('[');
		USART1.write_blocking(lol_char);
		USART1.write_blocking(']');
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



template <>
void interrupt::handler<interrupt::irq::USART1>() {
	if (USART1->SR & (1<<5)) {
		lol_char = USART1->DR;
	}
}

