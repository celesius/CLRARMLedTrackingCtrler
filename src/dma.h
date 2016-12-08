/*******************************************************************************
 * @name    : DMA配置头文件
 * @author  : 布谷鸟
 * @web     : WWW.UCORTEX.COM
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : 介绍STM32 DMA的配置
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
 *
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * 更改时间：2014-04-03    更改人：布谷鸟
 * 版本记录：V1.0
 * 更改内容：新建
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
 
#ifndef __DMA_H
#define __DMA_H
#include "stm32f10x.h"

void DMA_Polling_Init(DMA_TypeDef* DMAy, DMA_Channel_TypeDef* DMAy_Channelx, 
										uint32_t PeripheralBaseAddr, uint32_t MemoryBaseAddr, 
										uint32_t BufferSize, uint32_t DataType,
										uint32_t DMA_DIR, uint32_t DMA_Mode);
										
void DMA_Transfer_Start(DMA_Channel_TypeDef* DMAy_Channelx, uint16_t DataNumber);

#endif

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/

