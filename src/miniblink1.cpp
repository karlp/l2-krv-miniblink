// zyp's "I think this is my gd32v miniblink?" https://paste.jvnv.net/raw/Z30Xj
// plus zyp's "you'll need this too..." https://paste.jvnv.net/raw/14lMz


#include <gpio/gpio.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
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

    while(1) {
//        usb.process();

	led.toggle();

        sleep(40000);
    }
}
