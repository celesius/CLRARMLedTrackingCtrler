/*******************************************************************************
 * @name    : ��ʱ�����ļ�
 * @author  : ������
 * @web     : WWW.UCORTEX.COM
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : �����΢�����ʱ���������ʵ�����̵���
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
 * KEY -> PA0
 *
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-04-03    �����ˣ�������
 * �汾��¼��V1.0
 * �������ݣ��½�
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
#include "delay.h"

/**
  * @brief  ����SysTick����ʱ��ΪHCLK/8
  * @param  None
  * @retval None
	* @note   
	*    ��HCLKΪ72MHzʱ����ӳ�ʱ��Ϊ0xFFFFFF/9000000 (S) = 1864ms
  */
void Delay_Init(void)
{
	/* ����SysTick�ļ���ʱ��Ƶ��ΪHCLK/8�����HCLK��72MHz����SysTick����ʱ����9MHz*/
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
}

/**
  * @brief  ms��ʱ����
	* @param  xms: ��ʱ�ĺ�����
  * @retval None
	* @note   
	*    ͨ������Ĺ�ʽ���ı�SysTick���ж�ʱ��:
  *                          
  *    ����ֵ(Reload Value) = SysTick����ʱ��Ƶ�� (Hz) x  ϣ���жϵ�ʱ���� (s)
  *  
  *    - ����ֵ��ΪSysTick_Config()�Ĳ�������
	*    - ����ֵ�������� 0xFFFFFF������������ʱ1864ms
  */
void delay_ms(uint16_t xms)
{
	uint32_t reload;
	
	reload = SystemCoreClock/8000;//��ʱ1ms������ֵ
	reload *= xms;//��ʱxms������ֵ��
	SysTick->LOAD = (reload & 0xFFFFFF) - 1;//����SysTick����ֵ
	SysTick->VAL = 0;//����ֵ����
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;//��ʼ��������
	while(!((SysTick->CTRL) & (1 << 16)));//�ȴ�ʱ�䵽��
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//�رյδ������
	SysTick->VAL = 0x00;//��ռ�����
}

/**
  * @brief  us��ʱ����
	* @param  xus: ��ʱ��΢����
  * @retval None
	* @note   
	*		ͨ������Ĺ�ʽ���ı�SysTick���ж�ʱ��:
  *                          
  *   ����ֵ(Reload Value) = SysTick����ʱ��Ƶ�� (Hz) x  ϣ���жϵ�ʱ���� (s)
  *  
  *   - ����ֵ��ΪSysTick_Config()�Ĳ�������
	*		- ����ֵ�������� 0xFFFFFF������������ʱ1864ms
  */
void delay_us(uint32_t xus)
{
	uint32_t reload;
	
	reload = SystemCoreClock/8000000;//��ʱ1us������ֵ
	reload *= xus;//��ʱxus������ֵ��
	SysTick->LOAD = (reload & 0xFFFFFF) - 1;//����SysTick����ֵ
	SysTick->VAL = 0;//����ֵ����
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;//��ʼ��������
	while(!((SysTick->CTRL) & (1 << 16)));//�ȴ�ʱ�䵽��
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;//�رյδ������
	SysTick->VAL = 0x00;//��ռ�����
}

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
