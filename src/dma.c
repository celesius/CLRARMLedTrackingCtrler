/*******************************************************************************
 * @name    : DMA����
 * @author  : ������
 * @web     : WWW.UCORTEX.COM
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : ����STM32 DMA������
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
 #include "dma.h"
 
 
/**
  * @brief ��ѯģʽDMA��ʼ��
  * @param DMAy : DMA1 �� DMA2
	* @param DMAy_Channelx : ����DMAͨ������ο���STM32���Ĳο��ֲᡷ��148ҳ����P149ҳ
	* @param PeripheralBaseAddr : �������ݼĴ�����ַ
	* @param MemoryBaseAddr : �洢���׵�ַ��һ�������Ƕ����buffer����ĵ�ַ��
	* @param BufferSize : Buffer�Ĵ�С�������Ƕ����buffer�����С��
	* @param DataType : �������ͣ�0 - 8bit�� 1 - 16bit�� 2 - 32bit
	* @param DMA_DIR : DMA_DIR_PeripheralDST(buffer->����) �� DMA_DIR_PeripheralSRC(����->buffer)
	* @param DMA_Mode: ��ͨ����ģʽ(DMA_Mode_Normal��ѭ������ģʽ(DMA_Mode_Circular)
  * @retval None
  * @note
  */
void DMA_Polling_Init(DMA_TypeDef* DMAy, DMA_Channel_TypeDef* DMAy_Channelx,
										uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, 
										uint32_t BufferSize, uint32_t DataType,
										uint32_t DMA_DIR, uint32_t DMA_Mode)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	if(DMAy==DMA1)RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//��DMAʱ��
	else RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);//��DMAʱ��

  /* USARTy TX DMA1 Channel (triggered by USARTy Tx event) Config */
  DMA_DeInit(DMAy_Channelx);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)PeripheralBaseAddr;//�����ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)MemoryBaseAddr;//�洢����ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR;//DMA����DMA_DIR_PeripheralDST��DMA_DIR_PeripheralSRC
  DMA_InitStructure.DMA_BufferSize = BufferSize;//Buffer��С
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢����ַ����
  DMA_InitStructure.DMA_PeripheralDataSize = (uint32_t)(DataType<<8);//������������
  DMA_InitStructure.DMA_MemoryDataSize = (uint32_t)(DataType<<10);//Buffer��������
  DMA_InitStructure.DMA_Mode = DMA_Mode;//��ͨ����ģʽ(DMA_Mode_Normal��ѭ������ģʽ(DMA_Mode_Circular)
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//��DMAͨ��ӵ�и����ȼ�
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//�Ǵ洢�����洢���Ĵ���
  DMA_Init(DMAy_Channelx, &DMA_InitStructure);//��������Ϣд��Ĵ�������ʼ��DMA
}

/**
  * @brief ����һ��DMA����
	* @param DMAy_Channelx : ����DMAͨ������ο���STM32���Ĳο��ֲᡷ��148ҳ����P149ҳ
	* @param BufferSize : ����DMA������ٸ�����
  * @retval None
  * @note
  */
void DMA_Transfer_Start(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber)
{ 
	DMA_Cmd(DMAy_Channelx, DISABLE );	//�ر�DMAͨ��  
 	DMA_SetCurrDataCounter(DMAy_Channelx, DataNumber);//���ñ���DMA�����������
 	DMA_Cmd(DMAy_Channelx, ENABLE);		//��DMAͨ��
}
 
 /********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
 

