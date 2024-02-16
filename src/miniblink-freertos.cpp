
#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

#include <cstdio>
#include <cerrno>
#include <cstdlib>
#include <unistd.h>

#include <interrupt/interrupt.h>
#include <nxp_kx/rcm.h>
#include <nxp_kx/sim.h>
#include <nxp_kx/wdog.h>
#include <gpio/gpio.h>

Pin led1 = GPIO_LED1;

extern void laks_entry(void);

void entry(void)
{
	// MUST be early!
	WDOG.unlock();
	WDOG.disable();

	laks_entry();
}

static void task_blink1(void *pvParameters)
{
	(void)pvParameters;
#ifdef RCC_ENABLE1
	SIM.enable(RCC_ENABLE1);
#endif

	// FIXME - this is gross.  want to use a ?friend? class
	// we need to assign the right "PRCx" from the led1 pin?
	PCRB.mux(led1.n, NXP_PCR_KX_t<NXP_PCR_KX_reg_t>::Alt1_GPIO);

	led1.set_out();

	int qq = 0;
	while (1)
	{
		vTaskDelay(portTICK_PERIOD_MS * 500);
		// vTaskDelay(pdMS_TO_TICKS(500));  // these _should_ be "the same" right? :)
		printf("tick: %d\n", qq++);
		led1.toggle();
	}
}

int main()
{
	// Too late to turn off watchdog here!
	uint8_t reason0 = RCM->SRS0;
	uint8_t reason1 = RCM->SRS1;
	if (reason0 == 0x42 && reason1 == 0x69)
	{
		while (1)
			;
	}
	printf("boot reset reasons: %x %x\n", reason0, reason1);
	// Required to use FreeRTOS ISR methods!
	// NVIC.set_priority(interrupt::irq::DMA1_CH1, 6 << configPRIO_BITS);

	printf("Welcome to laks+freertos on kinetis\n");

	xTaskCreate(task_blink1, "blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
	vTaskStartScheduler();
}

void vAssertCalled(const char *const pcFileName, unsigned long ulLine)
{
	volatile unsigned long ulSetToNonZeroInDebuggerToContinue = 0;

	/* Parameters are not used. */
	(void)ulLine;
	(void)pcFileName;

	taskENTER_CRITICAL();
	{
		while (ulSetToNonZeroInDebuggerToContinue == 0)
		{
			/* Use the debugger to set ulSetToNonZeroInDebuggerToContinue to a
			non zero value to step out of this function to the point that raised
			this assert(). */
			__asm volatile("NOP");
			__asm volatile("NOP");
		}
	}
	taskEXIT_CRITICAL();
}

// TODO -figure out how to give this to freertosconfig?
// #define vPortSVCHandler SVC_Handler
// #define xPortPendSVHandler PendSV_Handler
// #define xPortSysTickHandler SysTick_Handler
extern "C"
{
	void vPortSVCHandler(void);
	void xPortPendSVHandler(void);
	void xPortSysTickHandler(void);
}
template <>
void interrupt::handler<interrupt::exception::SVCall>()
{
	vPortSVCHandler();
}
template <>
void interrupt::handler<interrupt::exception::PendSV>()
{
	xPortPendSVHandler();
}
template <>
void interrupt::handler<interrupt::exception::SysTick>()
{
	xPortSysTickHandler();
}