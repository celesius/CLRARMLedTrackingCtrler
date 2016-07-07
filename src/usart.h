/*******************************************************************************
 * @name    : 串口通信底层驱动
 * @author  : 布谷鸟
 * @web     : WWW.UCORTEX.COM
 * @version : V1.2
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : STM32串口的配置和使用
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
 * 注意使用本串口驱动文件时，需要在stm32f10x_it.c编写相应的串口接收中断函数
 * void USART1_IRQHandler(void)。
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
 * 更改时间：2014-04-03    更改人：布谷鸟
 * 版本记录：V1.0
 * 更改内容：新建
 * ----------------------------------------------------------------------------
 * 更改时间：2014-05-01    更改人：布谷鸟
 * 版本记录：V1.1
 * 更改内容：添加了DMA传输方式的支持
 * ----------------------------------------------------------------------------
 * 更改时间：2014-05-10    更改人：布谷鸟
 * 版本记录：V1.2
 * 更改内容：采用串口信息结构体管理串口数据，注意不向上兼容老版本串口驱动（需要
 *		更改串口状态字，串口接收计数值）。中断服务程序也做了响应的调整！
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/

#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"

#define COMn							3							//定义STM32支持的串口数	

//定义print从哪个串口输出，如果不使用printf，则注释掉这一行
#define PRINTF_SUPPORT		UCORTEX_COM1	
#ifdef PRINTF_SUPPORT
#include <stdio.h>
#endif

//是否使用串口DMA传输
#define 	COM_DMA_TRANSFER		1 			//如果不需要支持DMA，请定义为0

#define COM_RX_LEN						128			//串口接收缓冲长度

#if COM_RX_LEN
extern uint16_t COM_RX_STA;						//串口接收状态字
extern uint8_t COM_RX_BUF[COM_RX_LEN];//串口接收缓冲
//extern uint16_t COM_DR_DATA[COMn];		//串口接收数据
#endif



//串口信息结构体定义
typedef struct
{
	char 			name[5];	//设备号字符串 COM1,COM2...
	uint8_t   onoff;    //串口状态，打开或关闭
	uint8_t   rxsize;		//数据缓冲大小
	uint8_t   txsize;   //发送缓冲大小
	uint16_t  rxstatus;	//接收状态字
	uint16_t  txstatus;	//发送状态字
	uint16_t  rxnbr;		//接收到有效数据个数
	uint32_t  baudrate;	//波特率
	uint8_t*  rxbuf;		//数据接收缓冲地址
	uint8_t*  txbuf;		//发送缓冲地址
}COM_InfoTypeDef;
/**
	*	rxstatus:
	*	bit15,		接收完成标志
	*	bit14,		接收到回车键'\r'
	* bit13,		接收到换行键'\n'
	* bit12,		串口接收到一个字符
	*	bit11~8		保留
	*	bit7~0，	接收到的数据
  */

extern COM_InfoTypeDef COM1_Info, COM2_Info, COM3_Info;
extern COM_InfoTypeDef* COM_INFO[COMn];

/**
 * @brief 定义COM PORT1，连接到USART1
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
 * @brief 定义COM PORT2，连接到USART2
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
 * @brief 定义COM PORT3，连接到USART3
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

//定义串口端口
typedef enum 
{
  COM1 = 0,
  COM2 = 1,
	COM3 = 2
} COM_TypeDef;


void COM_Init(COM_TypeDef COM, uint32_t BaudRate); //串口初始化
void COM_DMA_Polling_Init(COM_TypeDef COM, uint32_t MemoryBaseAddr, uint32_t BufferSize, uint32_t DMA_DIR);

/* -------------------------------常用的串口操作接口函数------------------------------ */
void COM_PutChar(COM_TypeDef COM, uint8_t c);//串口发送字符
void COM_PutStr(COM_TypeDef COM, uint8_t *s);//串口输出字符串
uint8_t COM_GetInput(COM_TypeDef COM, uint8_t *pdata, uint32_t timeout);//等待接收一个字节的数据
uint8_t COM_GetStr(COM_TypeDef COM, uint32_t timeout);//等待接收一个字符串


#endif

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
