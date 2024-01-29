
#include "common.h"

enum usb_clock
{
  MCGPLL0,
  MCGPLL1,
  MCGFLL,
  PLL1,
  CLKIN
};


/* Select Clock source */
#define USB_CLOCK   MCGPLL0
//#define USB_CLOCK   MCGPLL1
//#define USB_CLOCK   MCGFLL
//#define USB_CLOCK   PLL1
//#define USB_CLOCK   CLKIN

/* USB Fractional Divider value for 120MHz input */
/** USB Clock = PLL0 x (FRAC +1) / (DIV+1)       */
/** USB Clock = 120MHz x (1+1) / (4+1) = 48 MHz    */
  #define USB_FRAC    SIM_CLKDIV2_USBFSFRAC_MASK
  #define USB_DIV     SIM_CLKDIV2_USBFSDIV(4)

#define FLAG_SET(BitNumber, Register)        (Register |=(1<<BitNumber))
#define FLAG_CLR(BitNumber, Register)        (Register &=~(1<<BitNumber))
#define FLAG_CHK(BitNumber, Register)        (Register & (1<<BitNumber))

#define ENABLE_USB_5V   GPIOB_PSOR|=(1<<8);
#define DISABLE_USB_5V  GPIOB_PCOR|=(1<<8);

void vfnInitUSBClock (uint8 u8ClkOption);

