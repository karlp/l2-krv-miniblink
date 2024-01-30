
#include "sysinit.h"
#include "bsp_K70.h"

void tu_init(void)
{
	sysinit();
	vfnInitUSBClock(USB_CLOCK);
}
