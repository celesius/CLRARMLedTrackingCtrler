/*
 * IFVRCtrlerButton.cpp
 *
 *  Created on: 2016年11月1日
 *      Author: clover
 */

#include "IFVRCtrlerButton.h"
#include "../IFVRCtrlerPinConfig.h"

#include "stm32f10x.h"
#include "diag/Trace.h"

#include "../common/bitband.h"

#define CONNECTGND

#ifndef BUTTON_IRQ_MOD
//#define BUTTONA = GPIOin(GPIOC, KEYPINA);
//#define BUTTONB = GPIOin(GPIOC, KEYPINB);
#endif

#define BUTTON_PORT  GPIOC
#define BUTTONA  KEYPINA
#define BUTTONB  KEYPINB



void IRQ_INIT(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = KEYPINA | KEYPINB;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0 );
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn  ; //PPP外部中断线
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1 );
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn  ; //PPP外部中断线
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line0 | EXTI_Line1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger =  EXTI_Trigger_Rising_Falling;  //EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0); //不能复用
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1 );
/*
	EXTI_ClearITPendingBit(EXTI_Line1 );
	EXTI_InitStructure.EXTI_Line = EXTI_Line1 ;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
*/
}


void IRQ_INITAButton(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = KEYPINA;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0 );
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn  ; //PPP外部中断线
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
/*
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1 );
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn  ; //PPP外部中断线
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
*/
	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line0 );
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger =  EXTI_Trigger_Rising_Falling;  //EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0); //不能复用
}


void normalButtonInit(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = KEYPINA | KEYPINB;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void normalButtonBInit(){
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = KEYPINB;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

IFVRCtrlerButton::IFVRCtrlerButton() {
	// TODO Auto-generated constructor stub
#ifdef BUTTON_IRQ_MOD
	IRQ_INITAButton();
	normalButtonBInit();
	bBcnt = 0;
#else
	normalButtonInit();
	bAcnt = 0;
	bBcnt = 0;
#endif
}

IFVRCtrlerButton::~IFVRCtrlerButton() {
	// TODO Auto-generated destructor stub
}

int IFVRCtrlerButton::getButtonA(){
	static uint32_t getButton = 0;
#ifdef CONNECTGND
	getButton = ((uint32_t)BUTTON_PORT->IDR & BUTTONA) ? 0:1;
#else
	getButton = ((uint32_t)BUTTON_PORT->IDR & BUTTONA) ? 1:0;
#endif
	if(getButton){
		if(bAcnt > 100){
			return 1;
		}
		bAcnt++;
	}else{
		bAcnt = 0;
	}
	return 0;
}
int IFVRCtrlerButton::getButtonB(){
	static uint32_t getButton = 0;
#ifdef CONNECTGND
	 getButton = ((uint32_t)BUTTON_PORT->IDR & BUTTONB) ? 0:1;
#else
	 getButton = ((uint32_t)BUTTON_PORT->IDR & BUTTONB) ? 1:0;
#endif
	//trace_printf("key = %d\n",getButton);
	if(getButton){
		if(bBcnt > 100){
			bBcnt = 0;
			return 1;
		}
		bBcnt++;
	}else{
		bBcnt = 0;
	}
	return 0;
}
