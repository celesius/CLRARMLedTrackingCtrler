/*
 * IFVRCtrlerInit.h
 *
 *  Created on: 2016年11月3日
 *      Author: clover
 */

#ifndef IFVRCTRLERSYS_H_
#define IFVRCTRLERSYS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "usart.h"
#ifdef __cplusplus
}
#endif
void IFVRCtrlerInit();

void InitCOM1();

void EnterSleepMode(void);
void ExitSleepMode(void);



#endif /* IFVRCTRLERSYS_H_ */
