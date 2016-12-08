/*
 * IFVRCtrlerPinConfig.h
 *
 *  Created on: 2016年11月1日
 *      Author: clover
 */

#ifndef IFVRCTRLERPINCONFIG_H_
#define IFVRCTRLERPINCONFIG_H_

#include "stm32f10x.h"

#define BUTTON_IRQ_MOD

const uint32_t KEYPINA = GPIO_Pin_0;
const uint32_t KEYPINB = GPIO_Pin_1;
const GPIO_TypeDef *KEYPINBANK = GPIOC;




#endif /* IFVRCTRLERPINCONFIG_H_ */
