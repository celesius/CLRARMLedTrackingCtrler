/*******************************************************************************
 * @name    : 定时器配置
 * @author  : 布谷鸟
 * @web     : WWW.UCORTEX.COM
 * @version : V1.3
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : 介绍如何配置定时器和编写定时中断服务程序
 * ---------------------------------------------------------------------------- 
 * @copyright
 *
 * UCORTEX版权所有，违者必究！例程源码仅供大家参考，旨在交流学习和节省开发时间，
 * 对于因为参考本文件内容，导致您的产品直接或间接受到破坏，或涉及到法律问题，作
 * 者不承担任何责任。对于使用过程中发现的问题，如果您能在WWW.UCORTEX.COM网站反
 * 馈给我们，我们会非常感谢，并将对问题及时研究并作出改善。例程的版本更新，将不
 * 做特别通知，请您自行到WWW.UCORTEX.COM下载最新版本，谢谢。
 * 对于以上声明，UCORTEX保留进一步解释的权利！
 * ----------------------------------------------------------------------------
 * @description
 *
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * 更改时间：2014-04-06    更改人：布谷鸟
 * 版本记录：V1.0
 * 更改内容：新建
 * ----------------------------------------------------------------------------
 * 更改时间：2014-05-02    更改人：布谷鸟
 * 版本记录：V1.1
 * 更改内容：添加TIM4_CH3 PWM输出配置
 * ----------------------------------------------------------------------------
 * 更改时间：2014-05-02    更改人：布谷鸟
 * 版本记录：V1.2
 * 更改内容：添加TIM2_CH1输入捕获配置
 * ----------------------------------------------------------------------------
 * 更改时间：2014-05-10    更改人：布谷鸟
 * 版本记录：V1.3
 * 更改内容：添加TIM3和TIM2的级联配置
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
 
#include "timer.h" 

/**
  * @brief 定时器3(TIM3)中断初始化
  * @param arr：自动重装值。
  * @param psc：时钟预分频数
  * @retval none
  * @note
	*    配置定时器3(TIM3)为向下计数模式
  *    定时器计数频率 = PCLK / ( psc + 1 )
  *    定时器中断周期 = ( arr + 1 )*( pac + 1) / PCLK
  */
void TIM3_INT_Init(uint16_t arr, uint16_t psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能外设时钟
	
	TIM_TimeBaseStructure.TIM_Period = arr;		//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc;	//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向下上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	//根据指定的参数初始化TIMx的时间基数单位
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);//使能ARR预装载，防止向上计数时更新事件异常延迟
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );	//允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;						//TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;				//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);	//初始化NVIC寄存器

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级

	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}

/**
  * @brief 定时器4通道3(TIM4_CH3)PWM输出配置
  * @param arr：自动重装值。
  * @param psc：时钟预分频数
  * @retval none
  * @note
	*    配置TIM4_CH3 PWM输出
  *    定时器计数频率 = PCLK / ( psc + 1 )
  *    定时器更新周期 = ( arr + 1 )*( psc + 1) / PCLK
  */
void TIM4_PWM_Init(uint16_t arr,uint16_t psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//使能定时器4时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIOB和AFIO复用功能时钟 
 
	//配置PB8为复用推挽输出,输出TIM4_CH3的PWM脉冲波形	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //PB8，对应TIM4_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
 
   //初始化TIM4
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	TIM_ARRPreloadConfig(TIM4,ENABLE);//使能ARR预装载，防止向上计数时更新事件异常延迟
	
	//初始化TIM4_CH3 PWM输出
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //将配置参数初始化外设TIM4 OC3

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR3上的预装载寄存器
 
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
}


//CaptureStatus : Bit15 - 完成捕获标志
//                Bit14 - 捕捉到上升沿标志
//                Bit13 - 溢出错误标志
//                Bit12 - 保留位
//                Bit11..0 - 更新事件发生次数
__IO uint16_t CaptureStatus = 0;//捕获状态
__IO uint16_t CaptureValue1 = 0;//上升沿时的捕获值
__IO uint16_t CaptureValue2 = 0;//下降沿时的捕获值
/**
  * @brief 配置TIM2_CH1为输入捕获
  * @param arr：自动重装值。
  * @param psc：时钟预分频数
  * @retval none
  * @note
  *    定时器计数频率 = PCLK / ( psc + 1 )
  *    定时器更新频率 = PCLK / (( psc + 1)*( arr + 1 ))
  */
void TIM2_Capture_Init(uint16_t arr,uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//打开GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//打开TIM2时钟

  //设置PA0为浮空输入，注意开发板上PA0外部有10K的下拉电阻
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //配置TIM2中断通道，设置系统中断组为2，抢占优先级为0，子优先级为2
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//中断号
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);//根据配置参数初始化对应的中断通道
	
	//注意在同一个项目中，中断组必须是一样的！
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置NVIC中断分组2:2位抢占优先级，2位响应优先级


	  /* TIM2 时基模块初始化 ---------------------
     定时器计数频率Fck_cnt = PCLK/(psc+1)
		 定时器更新频率Freq = PCLK/((psc+1)*(arr+1))
  ------------------------------------------------------------ */
	TIM_TimeBaseStructure.TIM_Period = arr;		//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;	//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向下计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//根据指定的参数初始化TIMx的时间基数单位
	
	TIM_ARRPreloadConfig(TIM2,ENABLE);//使能ARR预装载，防止向上计数时更新事件异常延迟

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );	//允许更新中断
	
	  /* TIM2 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM2 CH1 pin (PA0)  
     The Rising edge is used as active edge,
     The TIM2 CCR1 is used to compute the frequency value 
  ------------------------------------------------------------ */

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM2, ENABLE);

  /* Enable the CC1 Interrupt Request */
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
}

/**
  * @brief 配置TIM3,TIM2为两个定时器级联，形成一个32位的us级定时器
  * @param None
  * @retval none
  * @note 定时器级联: 1MHz时钟 -> TIM3 -> TIM2
  *   定时器计数频率 = PCLK / ( psc + 1 )
  *   定时器更新频率 = PCLK / (( psc + 1)*( arr + 1 ))
	*		TIM3和TIM2配置为主从模式，TIM3为主控定时器，TIM2为从定时器
  */
void TIM3_TIM2_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 

	//TIM3时基单元配置
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;     
	TIM_TimeBaseStructure.TIM_Prescaler = 72;	//1MHz，1us计数一次
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  TIM_ARRPreloadConfig(TIM3, ENABLE);	// 开启TIM3的预装在功能

	//将TIM3配置为TIM2的主定时器(Master)
	//将TIM3的更新事件作为输出触发TRGO(Trigger Output)
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	//使能TIM3的主从模式(Master Slave Mode）
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
	
  // TIM2 基本配置: TIM2为TIM32的从定时器(Slave)
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;	//自动重装值         
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;	//预分频       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);	//开启TIM2的预装在功能

	//将TIM2配置为TIM3的从定时器(Slave)
  TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);//选择TIM3的TRGO作为输入触发(ITR)
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);//选择ITR上升沿作为时钟输入
  //使能TIM2的主从模式(Master Slave Mode)
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

  //启动定时器
	TIM_Cmd(TIM3, ENABLE); 
	TIM_Cmd(TIM2, ENABLE);                  
}

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
