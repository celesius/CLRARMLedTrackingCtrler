//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "usart.h"
// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.

uint32_t m_nStart;               //DEBUG Stopwatch start cycle counter value
uint32_t m_nStop;                //DEBUG Stopwatch stop cycle counter value

#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))

#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define  DEM_CR      *(volatile u32 *)0xE000EDFC
#define  DEM_CR_TRCENA                   (1 << 24)

#define STOPWATCH_START { m_nStart = *((volatile unsigned int *)0xE0001004);}//DWT_CYCCNT;}
#define STOPWATCH_STOP  { m_nStop = *((volatile unsigned int *)0xE0001004);}

static inline void stopwatch_reset(void)
{
	/* Enable DWT */
	DEMCR |= DEMCR_TRCENA;
	*DWT_CYCCNT = 0;
	/* Enable CPU cycle counter */
	DWT_CTRL |= CYCCNTENA;
	DEM_CR |=  DEM_CR_TRCENA;
}

static inline uint32_t stopwatch_getticks()
{
	return CPU_CYCLES;
}

static inline void stopwatch_delay(uint32_t ticks)
{
	stopwatch_reset();
	while(1)
	{
		if (stopwatch_getticks() >= ticks)
			break;
	}
}

//uint32_t CalcNanosecondsFromStopwatch(uint32_t nStart, uint32_t nStop)
uint32_t CalcUsecondsFromStopwatch(uint32_t nStart, uint32_t nStop)
{
	uint32_t nTemp;
	uint32_t n;

	nTemp = nStop - nStart;

	//nTemp *= 1000;                          // Scale cycles by 1000.
	n = SystemCoreClock / 1000000;          // Convert Hz to MHz. SystemCoreClock = 168000000

	nTemp = nTemp / n;                      // nanosec = (Cycles * 1000) / (Cycles/microsec)

	return nTemp;
}

void getArray(uint8_t *allData, uint16_t vDataLength, uint8_t *vData)
{


}

unsigned char getChar(uint8_t *string){
	if(string[0] == '1'){
		return 1;
	} else if(string[0] == '2'){
		return 2;
	} else if(string[0] == '3'){
		return 3;
	} else if(string[0] == '4'){
		return 4;
	} else if(string[0] == '5'){
		return 5;
	} else if(string[0] == '6'){
		return 6;
	} else if(string[0] == '7'){
		return 7;
	} else if(string[0] == '8'){
		return 8;
	} else if(string[0] == '9'){
		return 9;
	}else{
		return 0;
	}
}

uint16_t led_mask;
#if 1
int main(int argc, char* argv[])
{
	trace_puts("Hello ARM World!\n");
	timer_start();
	blink_led_init();
	blink_tracking_led_init();

	uint8_t buff[128];

	strcpy((char*)&COM_INFO[COM1]->name, "COM1");
	COM_INFO[COM1]->baudrate = 115200;
	COM_INFO[COM1]->onoff = 1;
	COM_INFO[COM1]->rxstatus = 0;

	COM_INFO[COM1]->rxnbr = 0;
	COM_INFO[COM1]->rxsize = 128;//AT_RESP_SIZE;
	COM_INFO[COM1]->rxbuf = buff;

	COM_Init(COM1, 115200);
	int len = 0;
	int i = 0;

	int loopNum = 0;
	char rdata[100];
	int rdataLoop = 0;
	char turnLight = 0;

	while(1){
		//blink_led_on();
		//timer_sleep(BLINK_OFF_TICKS);
		//blink_led_off();
		//timer_sleep(BLINK_OFF_TICKS);
		if(COM_INFO[COM1]->rxstatus & 0x8000){
			//trace_printf("%d\n",COM_INFO[COM1]->rxsize);
			//trace_printf("%s\n",COM_INFO[COM1]->rxbuf);
			//trace_printf("len = %d\n",COM_INFO[COM1]->rxnbr);
			uint8_t vData[100] = {0};
			//memcpy(vData, COM_INFO[COM1]->rxbuf,COM_INFO[COM1]->rxnbr);
			//vData[COM_INFO[COM1]->rxnbr] = '\0';
			COM_INFO[COM1]->rxbuf[COM_INFO[COM1]->rxnbr] = '\0';
			led_mask = (uint16_t)atoi((char *)COM_INFO[COM1]->rxbuf)&0xff;
			//unsigned char const led_mask = getChar(COM_INFO[COM1]->rxbuf); //(unsigned char)atoi((char *)COM_INFO[COM1]->rxbuf)&0xff;
			//trace_printf("vData    = %s\n", vData);
			trace_printf("led_mask = 0x%x\n", led_mask); //不print  led_mask 很可能是0!!!!
			COM_INFO[COM1]->rxstatus = 0;	//清除接收状态字
			COM_INFO[COM1]->rxnbr = 0;// 清除接收计数
			blink_set_tracking_led(led_mask);
			if(turnLight){
				turnLight = 0;
				//blink_led_on();
			} else {
				turnLight = 1;
				//blink_led_off();
			}

		} else {
			//trace_printf("rxstatus  0x%x\n",COM_INFO[COM1]->rxstatus);
		}
		//char send_data[10] = {0};
		//sprintf(send_data,"d = %d\r\n",loopNum++);
		//for(i=0;i<10;i++){
			//USART_SendData(USART1, send_data[i]);//串口发送一个数据
			//while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);//等待串口发送完成
		//}
		//return (unsigned char)USART_ReceiveData(USARTx);
	}
}
#endif
//led test
#if 0
int main(int argc, char* argv[])
{
	trace_puts("Hello ARM World!\n");
	timer_start();
	blink_led_init();
	blink_tracking_led_init();

	while(1){
/*
		blink_led_on();
		blink_set_tracking_led(0x05);
		timer_sleep(BLINK_OFF_TICKS);
		blink_led_off();
		blink_set_tracking_led(0x06);
		timer_sleep(BLINK_OFF_TICKS);
		blink_set_tracking_led(0x08);
		timer_sleep(BLINK_OFF_TICKS);
*/
		blink_set_tracking_led(0x01);
		timer_sleep(BLINK_OFF_TICKS);
		blink_set_tracking_led(0x02);
		timer_sleep(BLINK_OFF_TICKS);
		blink_set_tracking_led(0x04);
		timer_sleep(BLINK_OFF_TICKS);
		blink_set_tracking_led(0x08);
		timer_sleep(BLINK_OFF_TICKS);

	}
}
#endif

/*
 * timer
int
main(int argc, char* argv[])
{
	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	timer_start();

	blink_led_init();

	uint32_t seconds = 0;
	int timeDiff = 0;

	stopwatch_reset();

	// Infinite loop
	while (1)
	{

		blink_led_on();
		STOPWATCH_START;
		timer_sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);
		STOPWATCH_STOP;

		blink_led_off();
		timer_sleep(BLINK_OFF_TICKS);


		//blink_led_on();
		//stopwatch_delay(SystemCoreClock);
		timeDiff = CalcUsecondsFromStopwatch(m_nStart, m_nStop);
		trace_printf("My function took %f seconds\n", (float)timeDiff/(1000.0*1000.0));
		trace_printf(" m_nStart = %d\n", m_nStart);
		trace_printf(" m_nStop  = %d\n", m_nStop);
		trace_printf("My function took %d useconds\n", timeDiff);


		//blink_led_off();
		//stopwatch_delay(SystemCoreClock);


		++seconds;

		// Count seconds on the trace device.
		trace_printf("Second %u\n", seconds);
		//trace_printf("My function took %d nanoseconds\n", timeDiff);
	}
	// Infinite loop, never return.
}
*/

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
