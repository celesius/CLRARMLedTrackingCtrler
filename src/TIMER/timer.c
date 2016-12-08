/*******************************************************************************
 * @name    : ��ʱ������
 * @author  : ������
 * @web     : WWW.UCORTEX.COM
 * @version : V1.3
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : ����������ö�ʱ���ͱ�д��ʱ�жϷ������
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
 * ����ʱ�䣺2014-05-02    �����ˣ�������
 * �汾��¼��V1.1
 * �������ݣ����TIM4_CH3 PWM�������
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-05-02    �����ˣ�������
 * �汾��¼��V1.2
 * �������ݣ����TIM2_CH1���벶������
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-05-10    �����ˣ�������
 * �汾��¼��V1.3
 * �������ݣ����TIM3��TIM2�ļ�������
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
 
#include "timer.h" 

/**
  * @brief ��ʱ��3(TIM3)�жϳ�ʼ��
  * @param arr���Զ���װֵ��
  * @param psc��ʱ��Ԥ��Ƶ��
  * @retval none
  * @note
	*    ���ö�ʱ��3(TIM3)Ϊ���¼���ģʽ
  *    ��ʱ������Ƶ�� = PCLK / ( psc + 1 )
  *    ��ʱ���ж����� = ( arr + 1 )*( pac + 1) / PCLK
  */
void TIM3_INT_Init(uint16_t arr, uint16_t psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʹ������ʱ��
	
	TIM_TimeBaseStructure.TIM_Period = arr;		//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc;	//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM�����ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);//ʹ��ARRԤװ�أ���ֹ���ϼ���ʱ�����¼��쳣�ӳ�
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE );	//��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;						//TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;				//�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);	//��ʼ��NVIC�Ĵ���

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�

	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
}

/**
  * @brief ��ʱ��4ͨ��3(TIM4_CH3)PWM�������
  * @param arr���Զ���װֵ��
  * @param psc��ʱ��Ԥ��Ƶ��
  * @retval none
  * @note
	*    ����TIM4_CH3 PWM���
  *    ��ʱ������Ƶ�� = PCLK / ( psc + 1 )
  *    ��ʱ���������� = ( arr + 1 )*( psc + 1) / PCLK
  */
void TIM4_PWM_Init(uint16_t arr,uint16_t psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ�ܶ�ʱ��4ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIOB��AFIO���ù���ʱ�� 
 
	//����PB8Ϊ�����������,���TIM4_CH3��PWM���岨��	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //PB8����ӦTIM4_CH3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
 
   //��ʼ��TIM4
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ʹ��ARRԤװ�أ���ֹ���ϼ���ʱ�����¼��쳣�ӳ�
	
	//��ʼ��TIM4_CH3 PWM���
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  //�����ò�����ʼ������TIM4 OC3

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR3�ϵ�Ԥװ�ؼĴ���
 
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
}


//CaptureStatus : Bit15 - ��ɲ����־
//                Bit14 - ��׽�������ر�־
//                Bit13 - ��������־
//                Bit12 - ����λ
//                Bit11..0 - �����¼���������
__IO uint16_t CaptureStatus = 0;//����״̬
__IO uint16_t CaptureValue1 = 0;//������ʱ�Ĳ���ֵ
__IO uint16_t CaptureValue2 = 0;//�½���ʱ�Ĳ���ֵ
/**
  * @brief ����TIM2_CH1Ϊ���벶��
  * @param arr���Զ���װֵ��
  * @param psc��ʱ��Ԥ��Ƶ��
  * @retval none
  * @note
  *    ��ʱ������Ƶ�� = PCLK / ( psc + 1 )
  *    ��ʱ������Ƶ�� = PCLK / (( psc + 1)*( arr + 1 ))
  */
void TIM2_Capture_Init(uint16_t arr,uint16_t psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//��TIM2ʱ��

  //����PA0Ϊ�������룬ע�⿪������PA0�ⲿ��10K����������
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //����TIM2�ж�ͨ��������ϵͳ�ж���Ϊ2����ռ���ȼ�Ϊ0�������ȼ�Ϊ2
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//�жϺ�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//�����ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);//�������ò�����ʼ����Ӧ���ж�ͨ��
	
	//ע����ͬһ����Ŀ�У��ж��������һ���ģ�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�


	  /* TIM2 ʱ��ģ���ʼ�� ---------------------
     ��ʱ������Ƶ��Fck_cnt = PCLK/(psc+1)
		 ��ʱ������Ƶ��Freq = PCLK/((psc+1)*(arr+1))
  ------------------------------------------------------------ */
	TIM_TimeBaseStructure.TIM_Period = arr;		//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;	//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���¼���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	TIM_ARRPreloadConfig(TIM2,ENABLE);//ʹ��ARRԤװ�أ���ֹ���ϼ���ʱ�����¼��쳣�ӳ�

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );	//��������ж�
	
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
  * @brief ����TIM3,TIM2Ϊ������ʱ���������γ�һ��32λ��us����ʱ��
  * @param None
  * @retval none
  * @note ��ʱ������: 1MHzʱ�� -> TIM3 -> TIM2
  *   ��ʱ������Ƶ�� = PCLK / ( psc + 1 )
  *   ��ʱ������Ƶ�� = PCLK / (( psc + 1)*( arr + 1 ))
	*		TIM3��TIM2����Ϊ����ģʽ��TIM3Ϊ���ض�ʱ����TIM2Ϊ�Ӷ�ʱ��
  */
void TIM3_TIM2_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 

	//TIM3ʱ����Ԫ����
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;     
	TIM_TimeBaseStructure.TIM_Prescaler = 72;	//1MHz��1us����һ��
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  TIM_ARRPreloadConfig(TIM3, ENABLE);	// ����TIM3��Ԥװ�ڹ���

	//��TIM3����ΪTIM2������ʱ��(Master)
	//��TIM3�ĸ����¼���Ϊ�������TRGO(Trigger Output)
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	//ʹ��TIM3������ģʽ(Master Slave Mode��
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
	
  // TIM2 ��������: TIM2ΪTIM32�ĴӶ�ʱ��(Slave)
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;	//�Զ���װֵ         
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;	//Ԥ��Ƶ       
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_ARRPreloadConfig(TIM2, ENABLE);	//����TIM2��Ԥװ�ڹ���

	//��TIM2����ΪTIM3�ĴӶ�ʱ��(Slave)
  TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);//ѡ��TIM3��TRGO��Ϊ���봥��(ITR)
  TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);//ѡ��ITR��������Ϊʱ������
  //ʹ��TIM2������ģʽ(Master Slave Mode)
  TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);

  //������ʱ��
	TIM_Cmd(TIM3, ENABLE); 
	TIM_Cmd(TIM2, ENABLE);                  
}

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
