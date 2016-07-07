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

#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"

#define COMn							3							//����STM32֧�ֵĴ�����	

//����print���ĸ���������������ʹ��printf����ע�͵���һ��
#define PRINTF_SUPPORT		UCORTEX_COM1	
#ifdef PRINTF_SUPPORT
#include <stdio.h>
#endif

//�Ƿ�ʹ�ô���DMA����
#define 	COM_DMA_TRANSFER		1 			//�������Ҫ֧��DMA���붨��Ϊ0

#define COM_RX_LEN						128			//���ڽ��ջ��峤��

#if COM_RX_LEN
extern uint16_t COM_RX_STA;						//���ڽ���״̬��
extern uint8_t COM_RX_BUF[COM_RX_LEN];//���ڽ��ջ���
//extern uint16_t COM_DR_DATA[COMn];		//���ڽ�������
#endif



//������Ϣ�ṹ�嶨��
typedef struct
{
	char 			name[5];	//�豸���ַ��� COM1,COM2...
	uint8_t   onoff;    //����״̬���򿪻�ر�
	uint8_t   rxsize;		//���ݻ����С
	uint8_t   txsize;   //���ͻ����С
	uint16_t  rxstatus;	//����״̬��
	uint16_t  txstatus;	//����״̬��
	uint16_t  rxnbr;		//���յ���Ч���ݸ���
	uint32_t  baudrate;	//������
	uint8_t*  rxbuf;		//���ݽ��ջ����ַ
	uint8_t*  txbuf;		//���ͻ����ַ
}COM_InfoTypeDef;
/**
	*	rxstatus:
	*	bit15,		������ɱ�־
	*	bit14,		���յ��س���'\r'
	* bit13,		���յ����м�'\n'
	* bit12,		���ڽ��յ�һ���ַ�
	*	bit11~8		����
	*	bit7~0��	���յ�������
  */

extern COM_InfoTypeDef COM1_Info, COM2_Info, COM3_Info;
extern COM_InfoTypeDef* COM_INFO[COMn];

/**
 * @brief ����COM PORT1�����ӵ�USART1
 */ 
#define UCORTEX_COM1								USART1
#define UCORTEX_COM1_CLK						RCC_APB2Periph_USART1
#define UCORTEX_COM1_TX_PIN					GPIO_Pin_9
#define UCORTEX_COM1_TX_GPIO_PORT		GPIOA
#define UCORTEX_COM1_TX_GPIO_CLK		RCC_APB2Periph_GPIOA
#define UCORTEX_COM1_RX_PIN					GPIO_Pin_10
#define UCORTEX_COM1_RX_GPIO_PORT		GPIOA
#define UCORTEX_COM1_RX_GPIO_CLK		RCC_APB2Periph_GPIOA
#define UCORTEX_COM1_IRQn						USART1_IRQn
#define UCORTEX_COM1_DR_Base				((uint32_t)0x40013804)
#define UCORTEX_COM1_DMA						((DMA_TypeDef*)DMA1)
#define UCORTEX_COM1_TX_DMA_CH			((DMA_Channel_TypeDef*)DMA1_Channel4)
#define UCORTEX_COM1_RX_DMA_CH			((DMA_Channel_TypeDef*)DMA1_Channel5)


/**
 * @brief ����COM PORT2�����ӵ�USART2
 */ 
#define UCORTEX_COM2								USART2
#define UCORTEX_COM2_CLK						RCC_APB1Periph_USART2
#define UCORTEX_COM2_TX_PIN					GPIO_Pin_2
#define UCORTEX_COM2_TX_GPIO_PORT		GPIOA
#define UCORTEX_COM2_TX_GPIO_CLK		RCC_APB2Periph_GPIOA
#define UCORTEX_COM2_RX_PIN					GPIO_Pin_3
#define UCORTEX_COM2_RX_GPIO_PORT		GPIOA
#define UCORTEX_COM2_RX_GPIO_CLK		RCC_APB2Periph_GPIOA
#define UCORTEX_COM2_IRQn						USART2_IRQn
#define UCORTEX_COM2_DR_Base				((uint32_t)0x40004404)
#define UCORTEX_COM2_DMA						((DMA_TypeDef*)DMA1)
#define UCORTEX_COM2_TX_DMA_CH			((DMA_Channel_TypeDef*)DMA1_Channel7)
#define UCORTEX_COM2_RX_DMA_CH			((DMA_Channel_TypeDef*)DMA1_Channel6)

/**
 * @brief ����COM PORT3�����ӵ�USART3
 */ 
#define UCORTEX_COM3								USART3
#define UCORTEX_COM3_CLK						RCC_APB1Periph_USART3
#define UCORTEX_COM3_TX_PIN					GPIO_Pin_10
#define UCORTEX_COM3_TX_GPIO_PORT		GPIOB
#define UCORTEX_COM3_TX_GPIO_CLK		RCC_APB2Periph_GPIOB
#define UCORTEX_COM3_RX_PIN					GPIO_Pin_11
#define UCORTEX_COM3_RX_GPIO_PORT		GPIOB
#define UCORTEX_COM3_RX_GPIO_CLK		RCC_APB2Periph_GPIOB
#define UCORTEX_COM3_IRQn						USART3_IRQn
#define UCORTEX_COM3_DR_Base				((uint32_t)0x40004804)
#define UCORTEX_COM3_DMA						((DMA_TypeDef*)DMA1)
#define UCORTEX_COM3_TX_DMA_CH			((DMA_Channel_TypeDef*)DMA1_Channel2)
#define UCORTEX_COM3_RX_DMA_CH			((DMA_Channel_TypeDef*)DMA1_Channel3)

//���崮�ڶ˿�
typedef enum 
{
  COM1 = 0,
  COM2 = 1,
	COM3 = 2
} COM_TypeDef;


void COM_Init(COM_TypeDef COM, uint32_t BaudRate); //���ڳ�ʼ��
void COM_DMA_Polling_Init(COM_TypeDef COM, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DMA_DIR);

/* -------------------------------���õĴ��ڲ����ӿں���------------------------------ */
void COM_PutChar(COM_TypeDef COM, uint8_t c);//���ڷ����ַ�
void COM_PutStr(COM_TypeDef COM, uint8_t *s);//��������ַ���
uint8_t COM_GetInput(COM_TypeDef COM, uint8_t *pdata, uint32_t timeout);//�ȴ�����һ���ֽڵ�����
uint8_t COM_GetStr(COM_TypeDef COM, uint32_t timeout);//�ȴ�����һ���ַ���


#endif

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
