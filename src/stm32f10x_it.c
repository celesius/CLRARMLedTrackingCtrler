/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include <stdio.h>
#include "usart.h"
#include "LED/led.h"
#include "RTC/rtc.h"
#include "TIMER/timer.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*
void SysTick_Handler(void)
{
}
*/

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  ʵʱʱ��RTC���жϷ������
  * @param  None
  * @retval None
  */
/*
void RTC_IRQHandler(void)
{
  if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
  {
    RTC_ClearITPendingBit(RTC_IT_SEC);//���RTC���жϱ�־λ
    LED=!LED;							//LED��ת
    TimeDisplay = 1;			//ʱ����±�־��1
    RTC_WaitForLastTask();//�ȴ�RTC�Ĵ���������� 
  }
}
*/

/**
  * @brief  EXTI0�жϷ������
  * @param  None
  * @retval None
  */
/*
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
		LED=!LED;
    EXTI_ClearITPendingBit(EXTI_Line0);//���EXTI line0���жϱ�־
  }
}
*/

/**
  * @brief  USART1�����ж�
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	uint8_t data;
	//trace_printf("get OUT IF \n");
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	//�����ж�
	{
		//trace_printf("get U1 \n");
		data = USART_ReceiveData(USART1);//��ȡ���յ�������

		COM1_Info.rxstatus &= 0xFF00;
		COM1_Info.rxstatus |= data;
		COM1_Info.rxstatus |= 0x1000;			//���ý��ձ�־λ
/*-------------------------------------------------------------------*/
		if((COM1_Info.rxstatus & 0x8000) == 0)				//����δ���
		{
			if(COM1_Info.rxstatus & 0x4000)							//�Ѿ����յ��س���
			{
				if(data == '\n'){
					trace_printf("get n \n");
					COM1_Info.rxstatus |= 0x8000;						//���յ����м�
				}
				else																			//û�н��յ������Ļس�����"\r\n"
				{
					COM1_Info.rxnbr = 0;
					COM1_Info.rxstatus = 0;
				}
			}
			else //��û�յ�'\r'
			{
				if(data == '\r'){
					trace_printf("get r \n");
					trace_printf("%s\n",COM_INFO[COM1]->rxbuf);
					COM1_Info.rxstatus	|=	0x4000;					//���յ��س���
				}else
				{
					COM1_Info.rxbuf[COM1_Info.rxnbr]	=	data ;
					COM1_Info.rxnbr++;
					if(COM1_Info.rxnbr	>= (COM1_Info.rxsize - 1))//�������
					{
						COM1_Info.rxnbr = 0;
						COM1_Info.rxstatus = 0;
					}
				}		 
			}
		}
/*-------------------------------------------------------------------*/
	}
}

/**
  * @brief  USART2�����ж�
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{	
	uint8_t data;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)	//�����ж�
	{
		data = USART_ReceiveData(USART2);//��ȡ���յ�������
		COM2_Info.rxstatus &= 0xFF00;
		COM2_Info.rxstatus |= data;
		COM2_Info.rxstatus |= 0x1000;			//���ý��ձ�־λ
/*-------------------------------------------------------------------*/
		if((COM2_Info.rxstatus & 0x8000) == 0)				//����δ���
		{
			if(COM2_Info.rxstatus & 0x4000)							//�Ѿ����յ��س���
			{
				if(data == '\n'){
					COM2_Info.rxstatus |= 0x8000;						//���յ����м�
					COM2_Info.rxbuf[COM2_Info.rxnbr] = '\0';
				}
				else																			//û�н��յ������Ļس�����"\r\n"
				{
					COM2_Info.rxnbr = 0;
					COM2_Info.rxstatus = 0;
				}
			}
			else //��û�յ�'\r'
			{
				if(data == '\r')
					COM2_Info.rxstatus	|=	0x4000;					//���յ��س���
				else
				{
					COM2_Info.rxbuf[COM2_Info.rxnbr]	=	data ;
					COM2_Info.rxnbr++;
					if(COM2_Info.rxnbr	>= (COM2_Info.rxsize - 1))//�������
					{
						COM2_Info.rxnbr = 0;
						COM2_Info.rxstatus = 0;
					}
				}		 
			}
		}
/*-------------------------------------------------------------------*/
	}
}

/**
  * @brief  USART3�����ж�
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	uint8_t data;
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)	//�����ж�
	{
		data = USART_ReceiveData(USART3);//��ȡ���յ�������
		COM3_Info.rxstatus &= 0xFF00;
		COM3_Info.rxstatus |= data;
		COM3_Info.rxstatus |= 0x1000;			//���ý��ձ�־λ
/*-------------------------------------------------------------------*/
		if((COM3_Info.rxstatus & 0x8000) == 0)				//����δ���
		{
			if(COM3_Info.rxstatus & 0x4000)							//�Ѿ����յ��س���
			{
				if(data == '\n'){
					COM3_Info.rxstatus |= 0x8000;						//���յ����м�
				}
				else																			//û�н��յ������Ļس�����"\r\n"
				{
					COM3_Info.rxnbr = 0;
					COM3_Info.rxstatus = 0;
				}
			}
			else //��û�յ�'\r'
			{
				if(data == '\r')
					COM3_Info.rxstatus	|=	0x4000;					//���յ��س���
				else
				{
					COM3_Info.rxbuf[COM3_Info.rxnbr]	=	data ;
					COM3_Info.rxnbr++;
					if(COM3_Info.rxnbr	>= (COM3_Info.rxsize - 1))//�������
					{
						COM3_Info.rxnbr = 0;
						COM3_Info.rxstatus = 0;
					}
				}		 
			}
		}
/*-------------------------------------------------------------------*/
	}
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET) //�����˸����ж�
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//��������жϱ�־
		if(CaptureStatus&0X4000)//����������
		{
			if((CaptureStatus&0X0FFF)>=0X0FFF)//����������ʱ��̫����
			{
				CaptureStatus|=0X2000;//���������־
			}
			else
			{
				CaptureStatus++;//�����¼������ۼ�
			}
		}
	}
	
  if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) 
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);//�������Ƚ��жϱ�־
		
		if(CaptureStatus&0X8000)//�ϴβ���ֵδ��ʹ�ã��������ϴβ����ֵ
		{
			CaptureStatus = 0;
			CaptureValue1 = 0;
			CaptureValue2 = 0;
		}
		
    if((CaptureStatus&0X4000) == 0)//��׽��������
    {
      CaptureValue1 = TIM_GetCapture1(TIM2);//��¼����ֵ
      CaptureStatus |= 0X4000;//��־��׽��������
			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);//�����½��ز���
    }
    else if(CaptureStatus&0X4000)//�����½���
    {
      CaptureValue2 = TIM_GetCapture1(TIM2); //��¼����ֵ
      CaptureStatus &= ~0X4000;	//��������ر�־
			CaptureStatus |= 0X8000;	//���ò�����ɱ�־
			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising);//����Ϊ�����ز���
    }
  }
}

/**
  * @brief  ��ʱ��3�жϷ������
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���TIM3�����жϷ������
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  ); //���TIM3�����жϱ�־ 
		LED=!LED;
	}
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
