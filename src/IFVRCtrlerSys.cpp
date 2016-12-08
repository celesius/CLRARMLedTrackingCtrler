/*
 * IFVRCtrlerInit.cpp
 *
 *  Created on: 2016年11月3日
 *      Author: clover
 */
#include <stm32f10x.h>
#include "IFVRCtrlerSys.h"
#include <string.h>

void SysTick_Configuration(void);
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void initAllPins();

void IFVRCtrlerInit(){
	RCC_Configuration();
	initAllPins();
}

void RCC_Configuration(void)
{
	SystemInit();
}


void initAllPins(){
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure); /* Reset init structure */

	/* Turn on all GPIO ports ***********************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
			RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
			RCC_APB2Periph_GPIOE, ENABLE);

	/* Configure all unused GPIO port pins in Analog Input mode (floating
				input trigger OFF), this will reduce the power consumption and increase
				the device immunity against EMI/EMC **********************************/

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void EnterSleepMode(void){
	SCB->SCR |= (1ul << 1);
	SCB->SCR &= ~(1ul << 2);
}
void ExitSleepMode(void){
	SCB->SCR &= ~(1ul << 1);
}

void InitCOM1(){

	uint8_t buff[128];
	strcpy((char*)&COM_INFO[COM1]->name, "COM1");
	COM_INFO[COM1]->baudrate = 115200;
	COM_INFO[COM1]->onoff = 1;
	COM_INFO[COM1]->rxstatus = 0;

	COM_INFO[COM1]->rxnbr = 0;
	COM_INFO[COM1]->rxsize = 128;//AT_RESP_SIZE;
	COM_INFO[COM1]->rxbuf = buff;

	COM_Init(COM1, 115200);

}
