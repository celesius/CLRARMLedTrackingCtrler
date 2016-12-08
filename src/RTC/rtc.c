/*******************************************************************************
 * @name    : RTC底层驱动
 * @author  : 布谷鸟
 * @web     : WWW.UCORTEX.COM
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : STM32内部实时时钟RTC底层驱动
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
 *
 ******************************************************************************/
 
#include "rtc.h" 

uint8_t TimeDisplay=0;//时间更新变量

/**
  * @brief RTC初始化
  * @retval none
  * @note
	*    使能RTC中断，配置外部32.768KHz晶振RTC时钟源。
  */
void RTC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//配置RTC中断
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断组2
  
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;//中断号
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)//RTC尚未配置过
	{
		/* 使能 PWR 和 BKP 时钟 */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	  PWR_BackupAccessCmd(ENABLE);//允许访问备份区
	  BKP_DeInit();								//复位备份区

	  RCC_LSEConfig(RCC_LSE_ON);	//使能外部32.768KHz低速晶振
	  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){}//等待晶振准备好
	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//选择32.768KHz晶振作为RTC时钟
	  RCC_RTCCLKCmd(ENABLE);	//使能RTC时钟

	  RTC_WaitForSynchro();		//等待RTC寄存器同步
	  RTC_WaitForLastTask();	//等待RTC寄存器操作完成
	  
	  RTC_ITConfig(RTC_IT_SEC, ENABLE);	//使能RTC秒中断
	  RTC_WaitForLastTask();						//等待RTC寄存器操作完成

	  /* 设置RTC预分频，使得RTC周期为1秒 */
	  RTC_SetPrescaler(32767);	// RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
	  RTC_WaitForLastTask();		//等待RTC寄存器操作完成

		RTC_SetCounter(22*3600+25*60+45);//默认设置为22:25:45
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);//在备份区域写入配置后的标识
	}
	else
	{
	  RTC_WaitForSynchro();		//等待RTC寄存器同步
	  RTC_WaitForLastTask();	//等待RTC寄存器操作完成
	  
	  RTC_ITConfig(RTC_IT_SEC, ENABLE);	//使能RTC秒中断
	  RTC_WaitForLastTask();						//等待RTC寄存器操作完成
	}
}



/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/

