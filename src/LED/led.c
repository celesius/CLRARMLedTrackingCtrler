/*******************************************************************************
 * @name    : LED���������ļ�
 * @author  : ������
 * @web     : WWW.UCORTEX.COM
 * @version : V1.1
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : ����STM32 PB12��Ϊ����ڣ���������LEDָʾ��
 * ---------------------------------------------------------------------------- 
 * @copyright
 *
 * UCORTEX��Ȩ���У�Υ�߱ؾ�������Դ�������Ҳο���ּ�ڽ���ѧϰ�ͽ�ʡ����ʱ�䣬
 * ������Ϊ�ο����ļ����ݣ��������Ĳ�Ʒֱ�ӻ����ܵ��ƻ������漰���������⣬��
 * �߲��е��κ����Ρ�����ʹ�ù����з��ֵ����⣬���������WWW.UCORTEX.COM��վ��
 * �������ǣ����ǻ�ǳ���л�����������⼰ʱ�о����������ơ����̵İ汾���£�����
 * ���ر�֪ͨ���������е�WWW.UCORTEX.COM�������°汾��лл��
 * ��������������UCORTEX������һ�����͵�Ȩ����
 * ----------------------------------------------------------------------------
 * @description
 * LED -> PB12
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-04-03    �����ˣ�������
 * �汾��¼��V1.0
 * �������ݣ��½�
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-04-03    �����ˣ�������
 * �汾��¼��V1.1
 * �������ݣ���Ϊλ������ģʽ
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
#include "led.h"

/**
  * @brief  LED IO��ʼ��
  * @param  None
  * @retval None
	* @note   LED���ӵ�GPIOB.12���͵�ƽ�����ߵ�ƽ��
  */
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;	
       
  /* ʹ��GPIOBʱ�� */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

  /* ����GPIOB.12Ϊ�������ģʽ */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);//����GPIOB.12Ϊ�ߵ�ƽ���ر�LED
}

/**
  * @brief  ����LEDָʾ��
  * @param  None
  * @retval None
	* @note   LED���ӵ�GPIOB.12���͵�ƽ�����ߵ�ƽ��
  */
void LED_On(void)
{
	LED = 0;
}

/**
  * @brief  �ر�LEDָʾ��
  * @param  None
  * @retval None
	* @note   LED���ӵ�GPIOB.12���͵�ƽ�����ߵ�ƽ��
  */
void LED_Off(void)
{
	LED = 1;
}

/**
  * @brief  LEDָʾ�Ʒ�ת
  * @param  None
  * @retval None
	* @note   LED���ӵ�GPIOB.12���͵�ƽ�����ߵ�ƽ��
  */
void LED_Toggle(void)
{
	LED = !LED;
}

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
