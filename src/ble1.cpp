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
#include <rtc/rtc.h>
#include <syscfg/syscfg.h>
#include <timer/timer.h>
#include <uart/uart.h>


#include "config.h"
#include "broadcaster.h"

// IMO, we pass MEM_BUF via the bleConfig_t.MEMAddr, so this is free form, but whateves.
//extern "C" {
__attribute__((aligned(4))) uint32_t MEM_BUF[BLE_MEMHEAP_SIZE / 4];
const uint8_t MacAddr[6] = {0x84, 0xC2, 0xE4, 0x03, 0x02, 0x02};
tmosTaskID halTaskID; 

//}
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


#if defined(CH58x)
Pin led = GPIO[(0 << 8) | 0]; // A0.... we might need some macros to fiddle this...
Pin utx = GPIO[(0 << 8) | 9]; // A9
Pin urx = GPIO[(0 << 8) | 8]; // A8
auto rcc_uart = rcc::UART1;
auto my_uart = UART1; // connected to P4 on the CH582M-R0-1v0 board
#else
#warning "unspecifed board, defaulting led to PA0"
Pin led = GPIOA[0];
#endif

extern "C" int _write(int file, char* ptr, int len)
{
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

#if defined(CH58x)

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
	ss = SYSCFG->SAFE_ACCESS_SIG; // just looking at timing. can be removed.
	SYSCFG.unlock_safe(); // you _probably_ still need this one!
	FLASH->CFG = flash_cfg; // undocumented...
	RCC->PLL_CONFIG |= (1 << 7); // undocumented...  yey...

	return 480000000u / real_div;
}
#endif

#define LED_BLINK_EVENT       0x0001
#define HAL_KEY_EVENT         0x0002
#define HAL_REG_INIT_EVENT    0x2000
#define HAL_TEST_EVENT        0x4000


tmosEvents wch_handle_events(tmosTaskID task_id, tmosEvents events)
{
	uint8_t *msgPtr;

	// goog: Process the HAL layer message, call tmos_msg_receive to read the message, and delete the message after processing
	if (events & SYS_EVENT_MSG) { // ??????HAL??????????????????tmos_msg_receive??????????????????????????????????????????
		msgPtr = tmos_msg_receive(task_id);
		if (msgPtr) {
			/* De-allocate */
			tmos_msg_deallocate(msgPtr);
		}
		return events ^ SYS_EVENT_MSG;
	}
	if (events & LED_BLINK_EVENT) {
#if(defined HAL_LED) && (HAL_LED == TRUE)
		HalLedUpdate();
#endif // HAL_LED
		return events ^ LED_BLINK_EVENT;
	}
	if (events & HAL_KEY_EVENT) {
#if(defined HAL_KEY) && (HAL_KEY == TRUE)
		HAL_KeyPoll(); /* Check for keys */
		tmos_start_task(halTaskID, HAL_KEY_EVENT, MS1_TO_SYSTEM_TIME(100));
		return events ^ HAL_KEY_EVENT;
#endif
	}
	if (events & HAL_REG_INIT_EVENT) {
#if(defined BLE_CALIBRATION_ENABLE) && (BLE_CALIBRATION_ENABLE == TRUE) // ???????????????????????????????????????10ms
		BLE_RegInit(); // ??????RF (Calibrate RF)
#if(CLK_OSC32K)
		Lib_Calibration_LSI(); // ????????????RC (Calibrate Internal RC)
#endif
		return events ^ HAL_REG_INIT_EVENT;
#endif
	}
	if (events & HAL_TEST_EVENT) {
		printf("* \n");
		tmos_start_task(halTaskID, HAL_TEST_EVENT, MS1_TO_SYSTEM_TIME(1000));
		return events ^ HAL_TEST_EVENT;
	}
	return 0;

}

uint32_t wch_ble_seed_random(void) {
	// Hardly the fanciest, but it's enough...
	return SYSTICK->CNT;
}


void wch_ble_init()
{
	// Turn on Systick, we're just going free running.
	SYSTICK->CMP = 0xFFFFFFFFFFFFFFFF - 1;
	// init, autoreload, hclk source, and enable.
	SYSTICK->CTLR = (1<<5) | (1<<3) | (1<<2) | (1<<0);
	


	bleConfig_t cfg = {0}; // is c++ the with or without 0 form? lol
	cfg.MEMAddr = (uint32_t) MEM_BUF;
	cfg.MEMLen = (uint32_t) BLE_MEMHEAP_SIZE;
	cfg.BufMaxLen = (uint32_t) BLE_BUFF_MAX_LEN;
	cfg.BufNumber = (uint32_t) BLE_BUFF_NUM;
	cfg.TxNumEvent = (uint32_t) BLE_TX_NUM_EVENT;
	cfg.TxPower = (uint32_t) BLE_TX_POWER;
	cfg.ConnectNumber = 0x3 << 2 | 0x3; // max please?
	// FIXME - shit, I should implementthat?
	//cfg.srandCB = SYS_GetSysTickCnt;
	cfg.srandCB = wch_ble_seed_random;
	// FIXME - selRTCclock?
	for (int i = 0; i < 6; i++) {
		cfg.MacAddr[i] = MacAddr[5 - i];
	}
	auto i = BLE_LibInit(&cfg);
	if (i) {
		printf("LIB init error code: %x ...\n", i);
		while (1);
	}


}

void hack_rtc_timer_init(void)
{
	SYSCFG.unlock_safe();
	// R8_CK32K_CONFIG |= RB_CLK_OSC32K_XT | RB_CLK_INT32K_PON | RB_CLK_XT32K_PON;
	// Select external, but turn on both internal and external? I dunno tom...
	RCC->CK32K_CONFIG |= (1<<2) | (1<<1) | (1<<0);
	SYSCFG.lock_safe();
	
	// RTC_InitTime(2020, 1, 1, 0, 0, 0); //RTC??????????????????????????????????
	SYSCFG.unlock_safe();

	// so, days are since 2020-1-1, and then I need a "2 seconds" count
	// and a "32k clock ticks" count.... yolo that shit right now.
	int days = 892; // 2022-june-11
	uint32_t secs2 = 0; // could not care less right now
	uint32_t ck32ticks = 0; // could not care less right now
	uint32_t tt = (secs2<<16 | ck32ticks);

	SYSCFG.unlock_safe();
	RTC->TRIG = days;
	RTC->MODE_CTRL |= (1<<7); // LOAD_HI
	SYSCFG.unlock_safe();
	RTC->TRIG = tt;
	RTC->MODE_CTRL |= (1<<6); // LOAD_LO
	SYSCFG.lock_safe();

}

int main()
{
	uint32_t sys_speed = rcc_set_pll(8); // 60MHz
#if defined(CH58x)
	utx.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);
	urx.set_mode(Pin::Input, Pin::Pull::Up, Pin::Drive::Low5);
#endif
	uart_enable(sys_speed);
	printf("Booted at %lu\n", sys_speed);

	led.set_mode(Pin::Output, Pin::Pull::Floating, Pin::Drive::Low5);

	// this is hopefully the blob version?
	printf("%s\n", VER_LIB);

	int i = 0;
	int qq = 0;


	bStatus_t bs;
	//CH58X_BLEInit();
	wch_ble_init();
	//HAL_Init();
	tmosTaskID halTaskID = TMOS_ProcessEventRegister(wch_handle_events);
	printf("Created event register: %d\n", halTaskID);
	// HAL_TimeInit();
	hack_rtc_timer_init();
	TMOS_TimerInit(0);
#if(defined BLE_CALIBRATION_ENABLE) && (BLE_CALIBRATION_ENABLE == TRUE)
	// goog: "Add calibration tasks, and a single calibration takes less than 10ms"
	bs = tmos_start_task(halTaskID, HAL_REG_INIT_EVENT, MS1_TO_SYSTEM_TIME(BLE_CALIBRATION_PERIOD)); // ?????????????????????????????????????????????10ms
    	printf("task start bl reg returned: %d\n", bs);

#endif


	bs = GAPRole_BroadcasterInit();
	printf("gaprole init returned: %d\n", bs);
	Broadcaster_Init();
	//Main_Circulation();
	uint64_t last = SYSTICK->CNT;

	while (1) {
		TMOS_SystemProcess();
		if (SYSTICK->CNT - last > 30000000) {
			printf("tick: %d\n", i++);
			last = SYSTICK->CNT;
		}
	}

}

#if defined(CH58x)

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

