/*******************************************************************************
 * @name    : ����ͨ�ŵײ�����
 * @author  : ������
 * @web     : WWW.UCORTEX.COM
 * @version : V1.2
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : STM32���ڵ����ú�ʹ��
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
 * ע��ʹ�ñ����������ļ�ʱ����Ҫ��stm32f10x_it.c��д��Ӧ�Ĵ��ڽ����жϺ���
 * void USART1_IRQHandler(void)��
 *
 * COM1:
 * USART1_TX -> PA9
 * USART1_RX -> PA10
 * 
 * COM2:
 * USART2_TX -> PA2
 * USART2_RX -> PA3
 * 
 * COM3:
 * USART3_TX -> PB10
 * USART3_RX -> PB11
 *
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
 * ����ʱ�䣺2014-05-01    �����ˣ�������
 * �汾��¼��V1.1
 * �������ݣ������DMA���䷽ʽ��֧��
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-05-10    �����ˣ�������
 * �汾��¼��V1.2
 * �������ݣ����ô�����Ϣ�ṹ����������ݣ�ע�ⲻ���ϼ����ϰ汾������������Ҫ
 *		���Ĵ���״̬�֣����ڽ��ռ���ֵ�����жϷ������Ҳ������Ӧ�ĵ�����
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
#include "usart.h"
#include "string.h"
#if COM_DMA_TRANSFER
//#include "dma.h"
#endif

#if COM_RX_LEN
/* COM_RX_STA: ���ڽ���״̬��
 * bit15��	������ɱ�־
 * bit14��	���յ�0x0d
 * bit13~0��	���յ�����Ч�ֽ���Ŀ
 */
uint16_t COM_RX_STA = 0;

/* ���ڽ��ջ��� */
uint8_t COM_RX_BUF[COM_RX_LEN];

#endif

//���崮����Ϣ�ṹ��
/*------------COM1--------------*/
COM_InfoTypeDef COM1_Info = 
{
		"COM1", //name
		1, //onoff
		COM_RX_LEN, //rxsize
		1, //txsize
		0, //rxstatus
		0, //txstatus
		0, //rxnbr
		115200, //baudrate
		COM_RX_BUF, //rxbuf
		(uint8_t*)0  //txbuf
};

/*------------COM2--------------*/
COM_InfoTypeDef COM2_Info = 
{
		"COM2", //name
		0, //onoff
		COM_RX_LEN, //rxsize
		1, //txsize
		0, //rxstatus
		0, //txstatus
		0, //rxnbr
		115200, //baudrate
		COM_RX_BUF, //rxbuf
		(uint8_t*)0  //txbuf
};

/*------------COM3--------------*/
COM_InfoTypeDef COM3_Info = 
{
		"COM3", //name
		0, //onoff
		COM_RX_LEN, //rxsize
		1, //txsize
		0, //rxstatus
		0, //txstatus
		0, //rxnbr
		115200, //baudrate
		COM_RX_BUF, //rxbuf
		(uint8_t*)0  //txbuf
};

COM_InfoTypeDef* COM_INFO[COMn] = {&COM1_Info, &COM2_Info, &COM3_Info};

/******************************* @printf  *************************************/

#ifdef PRINTF_SUPPORT //���´���,֧��printf����,������Ҫѡ��use MicroLIB
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	return x;
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	USART_SendData(PRINTF_SUPPORT, (uint8_t) ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(PRINTF_SUPPORT, USART_FLAG_TC) == RESET)
	{}
	return ch;
}
#endif
/********************************* @end ***************************************/

/**
 * @brief ���崮�����PORT��CLK��
 */
USART_TypeDef* COM_USART[COMn] = {UCORTEX_COM1, UCORTEX_COM2, UCORTEX_COM3}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {UCORTEX_COM1_TX_GPIO_PORT, UCORTEX_COM2_TX_GPIO_PORT, UCORTEX_COM3_TX_GPIO_PORT};

GPIO_TypeDef* COM_RX_PORT[COMn] = {UCORTEX_COM1_RX_GPIO_PORT, UCORTEX_COM2_RX_GPIO_PORT, UCORTEX_COM3_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {UCORTEX_COM1_CLK, UCORTEX_COM2_CLK, UCORTEX_COM3_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {UCORTEX_COM1_TX_GPIO_CLK, UCORTEX_COM2_TX_GPIO_CLK, UCORTEX_COM3_TX_GPIO_CLK};

const uint32_t COM_RX_PORT_CLK[COMn] = {UCORTEX_COM1_RX_GPIO_CLK, UCORTEX_COM2_RX_GPIO_CLK, UCORTEX_COM3_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {UCORTEX_COM1_TX_PIN, UCORTEX_COM2_TX_PIN, UCORTEX_COM3_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {UCORTEX_COM1_RX_PIN, UCORTEX_COM2_RX_PIN, UCORTEX_COM3_RX_PIN};

const uint8_t COM_IRQn[COMn] = {UCORTEX_COM1_IRQn, UCORTEX_COM2_IRQn, UCORTEX_COM3_IRQn};

#if COM_DMA_TRANSFER
DMA_TypeDef* COM_DMA[COMn] = {UCORTEX_COM1_DMA, UCORTEX_COM2_DMA, UCORTEX_COM3_DMA}; 

DMA_Channel_TypeDef* COM_DMA_CH[COMn][2] = {{UCORTEX_COM1_RX_DMA_CH, UCORTEX_COM1_TX_DMA_CH},
		{UCORTEX_COM2_RX_DMA_CH, UCORTEX_COM2_TX_DMA_CH},
		{UCORTEX_COM3_RX_DMA_CH, UCORTEX_COM3_TX_DMA_CH}};

const uint32_t COM_DR_BASE[COMn] = { UCORTEX_COM1_DR_Base, UCORTEX_COM2_DR_Base, UCORTEX_COM3_DR_Base};

#endif

/**
 * @brief  ��ʼ������GPIO
 * @param  COM: ָ��Ҫ��ʼ����COM PORT
 *     @arg COM1 ����1
 *     @arg COM2 ����2
 *     @arg COM3 ����3
 * @retval None
 * @note None
 */
void COM_GPIOInit(COM_TypeDef COM)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ����GPIOʱ�Ӻ͸��ù���ʱ��RCC_APB2Periph_AFIO */
	RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

	/* ���� USART Tx ����������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

	/* ���� USART Rx �������� */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
	GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);
}

/**
 * @brief  ���ô���
 * @param  COM: ָ��Ҫ��ʼ����COM PORT
 *     @arg COM1 ����1
 *     @arg COM2 ����2
 *     @arg COM3 ����3
 * @param  Baudrate ���ڲ�����
 * @retval None
 * @note None
 */
void COM_Init(COM_TypeDef COM, uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	COM_GPIOInit(COM);//��ʼ������GPIO

	/* ��������ʱ�ӣ�ע�⴮��1������APB1���������£��������ڹ���APB2����������*/
	if (COM == COM1)
		RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE); //��������ʱ��
	else
		RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);	//��������ʱ��

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�������ȼ���2
	/* USARTx NVIC������Ϣ */
	NVIC_InitStructure.NVIC_IRQChannel = COM_IRQn[COM];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;				//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//���������������Ϣ��ʼ��NVIC�Ĵ���


	/*    ����������Ϣ
	      -------------
        - �����ʸ��ݲ���ָ��
        - ���ݳ���8bit
        - 1��ֹͣλ
        - ����żУ��λ
        - ��Ӳ������������(RTS �� CTS �ź�)
        - �����ڷ��ͺͽ���
	 */
	USART_InitStructure.USART_BaudRate = BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(COM_USART[COM], &USART_InitStructure);			//���ݴ���������Ϣ��ʼ������

	USART_ITConfig(COM_USART[COM], USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(COM_USART[COM], ENABLE);										//��������
}

#if COM_DMA_TRANSFER
/**
 * @brief  ��ѯ��ʽ(Polling)����DMA��ʼ��
 * @param  COM: Ҫ���õĴ���
 *     @arg COM1 ����1(USART1)
 *     @arg COM2 ����2(USART2)
 *     @arg COM3 ����3(USART3)
 * @param  MemoryBaseAddr : DMA�����ַ��һ�������Ƕ���Ļ��������ַ��
 * @param  BufferSize : �����С
 * @param  DMA_DIR : DMA���䷽��
 *     @arg DMA_DIR_PeripheralDST : ���ڷ���DMA���� Buffer->USART ��DMA����
 *     @arg DMA_DIR_PeripheralSRC : ���ڽ���DMA���� USART->Buffer ��DMA����
 * @retval None
 * @note �ڱ������У�������Ӧ�Ĵ���DMA�ӿڣ�����δ����DMA���䣬����Ҫ����DMA����ʱ��
 *				�����DMA_Polling_Enable(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber)
 */
void COM_DMA_Polling_Init(COM_TypeDef COM, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DMA_DIR)
{
	//���ڶ�Ӧ��DMA
	DMA_TypeDef* DMAy = COM_DMA[COM];
	//��Ӧ��DMAͨ����ע�⴮�ڽ��պͷ��Ͷ�Ӧ��ͬ��DMAͨ����
	DMA_Channel_TypeDef* DMAy_Channelx = COM_DMA_CH[COM][DMA_DIR>>4];
	//�������ݼĴ�����ַ
	uint32_t PeripheralBaseAddr = COM_DR_BASE[COM];
	//����λ��: 0 -> 8bit, 1 -> 16bit, 2-> 32bit
	uint32_t DataType = 0; //8-bitλ��

	//����һ�����ö�Ӧ��DMAͨ��
	DMA_Polling_Init(DMAy, DMAy_Channelx, PeripheralBaseAddr, MemoryBaseAddr, BufferSize, DataType, DMA_DIR, DMA_Mode_Normal);

	//��������򿪴���DMA���ͻ���սӿ�
	if(DMA_DIR==DMA_DIR_PeripheralDST)//�򿪴���DMA����
		USART_DMACmd(COM_USART[COM], USART_DMAReq_Tx, ENABLE);

	if(DMA_DIR==DMA_DIR_PeripheralSRC)//�򿪴���DMA����
		USART_DMACmd(COM_USART[COM], USART_DMAReq_Rx, ENABLE);
}


/* -------------------------------���õĴ��ڲ����ӿں���------------------------------ */

/**
 * @brief  ���ڷ���һ���ֽ�
 *	@param	COM: COM1, COM2...
 * @param  c: �����͵��ֽ�
 * @retval None
 */
void COM_PutChar(COM_TypeDef COM, uint8_t c)
{
	USART_SendData(COM_USART[COM], c);
	while (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TXE) == RESET)
	{
	}
}

/**
 * @brief  ��ӡ�ַ���
 *	@param	COM: COM1, COM2...
 * @param  s: �ַ���ָ��
 * @retval None
 */
void COM_PutStr(COM_TypeDef COM, uint8_t *s)
{
	while (*s != '\0')
	{
		COM_PutChar(COM, *s);
		s++;
	}
}

/**
 * @brief  ��ѯ�����Ƿ���յ�һ���ַ�
 * @param  COM: COM1,COM2...
 * @param  pdata: �������ݴ�ŵ�ַ
 * @retval 0: ���ճɹ�
 *         1: δ���յ�
 */
uint8_t COM_ReceiveByte(COM_TypeDef COM, uint8_t *pdata)
{
	if(COM_INFO[COM]->rxstatus & 0x1000)//����Ӧ��COM�Ƿ���յ�����
	{
		COM_INFO[COM]->rxstatus &= ~(0x1000);//������ձ�־λ
		*pdata = COM_INFO[COM]->rxstatus & 0x00ff;
		return 0;
	}
	else return 1;
}

/**
 * @brief  �ȴ����մӴ��ڽ���һ���ֽ�
 * @param  COM: ���ں�, COM1,COM2...
 * @param	pdata: ���յ������ݴ�ŵ�ַ
 *	@param	timeout: ��ʱ���
 *    @arg 0  - ������ʱ��⣬һֱ�ȵ�����Ϊֹ
 *    @arg >0 - �ȴ���ʱ����
 * @retval 0 - ���ճɹ���1 - ����ʧ��
 */
uint8_t COM_Get(COM_TypeDef COM, uint8_t *pdata, uint32_t timeout)
{
	if(timeout > 0)//���ճ�ʱ���
	{
		while (timeout--)
		{
			if (COM_ReceiveByte(COM, pdata) == 0) return 0;//���յ�����
		}
	}
	else//������ʱ��⣬һֱ�ȴ�
	{
		while(1)
		{
			if (COM_ReceiveByte(COM,pdata) == 0) return 0;
		}
	}
	return 1;

}

/**
 * @brief  �ȴ��Ӵ��ڽ���һ���ַ���
 * @param  COM: ���ں�, COM1,COM2...
 *	@param	timeout: ��ʱ���
 * @retval 0 - ���ճɹ��� 1 - ����ʧ��
 */
uint8_t COM_GetStr(COM_TypeDef COM, uint32_t timeout)
{
	uint8_t c = 0;
	uint8_t error = 1;

	do
	{
		error = COM_Get(COM, &c, timeout);//�ȴ�����һ���ַ�
		if(error) return 1;//������ճ�ʱ�����ش���

		if (c == '\r') //���յ��س���
		{
			continue;
		}

		if (c == '\b') //���յ��˸��
		{
			COM_INFO[COM]->rxnbr--;

			if ((COM_INFO[COM]->rxnbr))
			{
				COM_PutStr(COM, "\b \b");
				COM_INFO[COM]->rxnbr--;
			}
			continue;
		}

		if (c >= 0x20 && c <= 0x7E)//�ַ������
		{
			COM_PutChar(COM, c);//�ַ�����
		}

	} while ((COM_INFO[COM]->rxstatus & 0x8000) == 0);

	COM_PutStr(COM, "\r\n");
	COM_INFO[COM]->rxbuf[COM_INFO[COM]->rxnbr] = '\0';//�ַ�ĩβ����ַ���������
	return 0;
}

/**
 * @brief  USART1串口中断
 * @param  None
 * @retval None
 * @note
 *    有效的一帧数据以回车换行结尾'\r'+'\n'
 */
void USART1_IRQHandler(void)
{
	uint8_t data;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	//½ÓÊÕÖÐ¶Ï
	{
		data = USART_ReceiveData(USART1);//¶ÁÈ¡½ÓÊÕµ½µÄÊý¾Ý
		COM1_Info.rxstatus &= 0xFF00;
		COM1_Info.rxstatus |= data;
		COM1_Info.rxstatus |= 0x1000;			//ÉèÖÃ½ÓÊÕ±êÖ¾Î»
		//trace_printf("data 	= 0x%x\n",data);
		//trace_printf("data c	= %c\n",data);
		//trace_printf("rxstatus = 0x%x\n",COM1_Info.rxstatus);
		/*-------------------------------------------------------------------*/
		if((COM1_Info.rxstatus & 0x8000) == 0)				//½ÓÊÕÎ´Íê³É
		{
			if(COM1_Info.rxstatus & 0x4000)							//ÒÑ¾­½ÓÊÕµ½»Ø³µ¼ü
			{
				if(data == '\n')
					COM1_Info.rxstatus |= 0x8000;						//½ÓÊÕµ½»»ÐÐ¼ü
				else																			//Ã»ÓÐ½ÓÊÕµ½Á¬ÐøµÄ»Ø³µ»»ÐÐ"\r\n"
				{
					COM1_Info.rxnbr = 0;
					COM1_Info.rxstatus = 0;
				}
			}
			else //»¹Ã»ÊÕµ½'\r'
			{
				if(data == '\r')
					COM1_Info.rxstatus	|=	0x4000;					//½ÓÊÕµ½»Ø³µ¼ü
				else
				{
					COM1_Info.rxbuf[COM1_Info.rxnbr]	=	data ;
					COM1_Info.rxnbr++;
					if(COM1_Info.rxnbr	>= (COM1_Info.rxsize - 1))//»º³åÒç³ö
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

#endif

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
