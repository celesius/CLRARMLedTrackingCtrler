//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

#include "BlinkLed.h"

// ----------------------------------------------------------------------------

void
blink_led_init()
{
  // Enable GPIO Peripheral clock
  RCC_APB2PeriphClockCmd(BLINK_RCC_MASKx(BLINK_PORT_NUMBER), ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin in output push/pull mode
  GPIO_InitStructure.GPIO_Pin = BLINK_PIN_MASK(BLINK_PIN_NUMBER);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BLINK_GPIOx(BLINK_PORT_NUMBER), &GPIO_InitStructure);

  // Start with led turned off
  blink_led_off();
}


void
blink_tracking_led_init(void)
{
  RCC_APB2PeriphClockCmd(BLINK_RCC_MASKx(BLINK_TRACKING_PORT_NUMBER), ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure pin in output push/pull mode
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;  //BLINK_PIN_MASK(0);
  //GPIO_InitStructure.GPIO_Pin = BLINK_PIN_MASK(1);
  //GPIO_InitStructure.GPIO_Pin = BLINK_PIN_MASK(2);
  //GPIO_InitStructure.GPIO_Pin = BLINK_PIN_MASK(4);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BLINK_GPIOx(BLINK_TRACKING_PORT_NUMBER), &GPIO_InitStructure);
  blink_set_tracking_led(0x00);
}

void blink_set_tracking_led(uint16_t data)
{
#if (BLINK_ACTIVE_LOW)
	//关闭所有led
	//for(i=0;i<4;i++){
	//	GPIO_SetBits(BLINK_GPIOx(BLINK_TRACKING_PORT_NUMBER),
	//			BLINK_PIN_MASK(i));
	//}
	GPIO_SetBits(BLINK_GPIOx(BLINK_TRACKING_PORT_NUMBER), 0x0f);
	if(data != 0)
		GPIO_ResetBits(BLINK_GPIOx(BLINK_TRACKING_PORT_NUMBER), data);

/*
	for(i=0;i<4;i++){
		unsigned mask = (data >> i) & 0x01;
		if(mask != 0){
			//trace_printf("Hello ARM World! data = %d\n",data);
			GPIO_ResetBits(BLINK_GPIOx(BLINK_TRACKING_PORT_NUMBER),
					BLINK_PIN_MASK(i));
		}
	}
	*/
#else
  GPIO_SetBits(BLINK_GPIOx(BLINK_PORT_NUMBER),
      BLINK_PIN_MASK(BLINK_PIN_NUMBER));
#endif
}

// ----------------------------------------------------------------------------
