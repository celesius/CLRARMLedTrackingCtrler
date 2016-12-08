/*
 * CLRPWMCtrl.cpp
 *
 *  Created on: 2016年8月25日
 *      Author: clover
 */

#include "CLRPWMCtrl.h"
#include "stm32f10x.h"
#include "diag/Trace.h"

/*******************************************************************************
 * Function Name  : RCC_Configuration
 * Description    : Configures the different system clocks.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void PWM_RCC_Configuration(int pinNum)
{
	ErrorStatus HSEStartUpStatus;
    /* RCC system reset(for debug purpose) */
    RCC_DeInit();

    /* Enable HSE */
    RCC_HSEConfig(RCC_HSE_ON);

    /* Wait till HSE is ready */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if(HSEStartUpStatus == SUCCESS)
    {
        /* HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);

        /* PCLK2 = HCLK */
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* PCLK1 = HCLK/4 */
        RCC_PCLK1Config(RCC_HCLK_Div4);

        /* Flash 2 wait state */
        FLASH_SetLatency(FLASH_Latency_2);
        /* Enable Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

        /* PLLCLK = 8MHz * 9 = 72 MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_12);

        /* Enable PLL */
        RCC_PLLCmd(ENABLE);

        /* Wait till PLL is ready */
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        /* Select PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        /* Wait till PLL is used as system clock source */
        while(RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }

    /* TIM2 clock enable */
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /* GPIOA clock enable */
    if(pinNum != 1)
    	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

/*******************************************************************************
 * Function Name  : GPIO_Configuration
 * Description    : Configure the TIM2 Pins.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void PWM_GPIO_Configuration(int pinNum)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if(pinNum != 1){
    	/* GPIOA Configuration:TIMNVIC_Configuration2 Channel1, 2, 3 and 4 in Output */
    	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;  //6->G  7->IR
    	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    	GPIO_Init(GPIOA, &GPIO_InitStructure);
    }
    //jiangbo
    if(pinNum != 1){
    	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //0->B 1->R
    }else{
    	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //0->B 1->R
    }
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*******************************************************************************
 * Function Name  : NVIC_Configuration
 * Description    : Configure the nested vectored interrupt controller.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void PWM_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;


#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    /* Enable the TIM2 global Interrupt */
    //NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//TIM2_IRQChannel;
    //NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//TIM2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void pwm_init(int pinNum)
{
    PWM_RCC_Configuration(pinNum);
    PWM_NVIC_Configuration();
    PWM_GPIO_Configuration(pinNum);
}

void TimebaseCgf(uint8_t R, uint8_t G, uint8_t B, uint8_t IR)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	vu16 AAR = 255;
    /* ---------------------------------------------------------------
       TIM2 Configuration: Output Compare Toggle Mode:
       TIM2CLK = 36 MHz, Prescaler = 0x2, TIM2 counter clock = 12 MHz
       CC1 update rate = TIM2 counter clock / CCR1_Val = 366.2 Hz
       CC2 update rate = TIM2 counter clock / CCR2_Val = 732.4 Hz
       CC3 update rate = TIM2 counter clock / CCR3_Val = 1464.8 Hz
       CC4 update rate = TIM2 counter clock / CCR4_Val =  2929.6 Hz
       --------------------------------------------------------------- */

    /* Time base configuration */
    //TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    //TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_Period = AAR;
    //TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
    TIM_TimeBaseStructure.TIM_Prescaler = 0x00;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    //TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* Output Compare Toggle Mode configuration: Channel1 */
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_Pulse = 2047;
    TIM_OCInitStructure.TIM_Pulse = G;  //pa6      g

    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_Pulse = 1023;
    TIM_OCInitStructure.TIM_Pulse = IR;    //pa7
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_Pulse = 511;
    TIM_OCInitStructure.TIM_Pulse = B;     //pb0      b
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_Pulse = 255;
    TIM_OCInitStructure.TIM_Pulse = R;     //pb1      r
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    /* TIM enable counter */
    TIM_Cmd(TIM3, ENABLE);

    /* TIM IT enable */
    TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
}

CLRPWMCtrl::CLRPWMCtrl(int pinNum) {
	// TODO Auto-generated constructor stub
	pwm_init(pinNum);
	TimebaseCgf(255,255,255,255);
}

CLRPWMCtrl::~CLRPWMCtrl() {
	// TODO Auto-generated destructor stub
}

void CLRPWMCtrl::run()
{
	static const int step = 10;
	static int setData = 0;
	if(setData+step > 255){
		setData = 10;
	}
	else
		setData = setData + step;
	TimebaseCgf(setData,setData,setData, setData);
}

void pwmSet(char data){
	TimebaseCgf(data, data, data, data);
}
