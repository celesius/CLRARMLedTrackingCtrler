/*******************************************************************************
 * @name    : RTC�ײ�����
 * @author  : ������
 * @web     : WWW.UCORTEX.COM
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : STM32�ڲ�ʵʱʱ��RTC�ײ�����
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
 *
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-04-06    �����ˣ�������
 * �汾��¼��V1.0
 * �������ݣ��½�
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
 
#include "rtc.h" 

uint8_t TimeDisplay=0;//ʱ����±���

/**
  * @brief RTC��ʼ��
  * @retval none
  * @note
	*    ʹ��RTC�жϣ������ⲿ32.768KHz����RTCʱ��Դ��
  */
void RTC_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	//����RTC�ж�
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж���2
  
  NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;//�жϺ�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)//RTC��δ���ù�
	{
		/* ʹ�� PWR �� BKP ʱ�� */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	  PWR_BackupAccessCmd(ENABLE);//������ʱ�����
	  BKP_DeInit();								//��λ������

	  RCC_LSEConfig(RCC_LSE_ON);	//ʹ���ⲿ32.768KHz���پ���
	  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET){}//�ȴ�����׼����
	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//ѡ��32.768KHz������ΪRTCʱ��
	  RCC_RTCCLKCmd(ENABLE);	//ʹ��RTCʱ��

	  RTC_WaitForSynchro();		//�ȴ�RTC�Ĵ���ͬ��
	  RTC_WaitForLastTask();	//�ȴ�RTC�Ĵ����������
	  
	  RTC_ITConfig(RTC_IT_SEC, ENABLE);	//ʹ��RTC���ж�
	  RTC_WaitForLastTask();						//�ȴ�RTC�Ĵ����������

	  /* ����RTCԤ��Ƶ��ʹ��RTC����Ϊ1�� */
	  RTC_SetPrescaler(32767);	// RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
	  RTC_WaitForLastTask();		//�ȴ�RTC�Ĵ����������

		RTC_SetCounter(22*3600+25*60+45);//Ĭ������Ϊ22:25:45
		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);//�ڱ�������д�����ú�ı�ʶ
	}
	else
	{
	  RTC_WaitForSynchro();		//�ȴ�RTC�Ĵ���ͬ��
	  RTC_WaitForLastTask();	//�ȴ�RTC�Ĵ����������
	  
	  RTC_ITConfig(RTC_IT_SEC, ENABLE);	//ʹ��RTC���ж�
	  RTC_WaitForLastTask();						//�ȴ�RTC�Ĵ����������
	}
}



/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/

