/*
 * CLRIRLed.cpp
 *
 *  Created on: 2016年8月25日
 *      Author: clover
 */

#include "CLRIRLed.h"
#include "stm32f10x.h"
#include "diag/Trace.h"

/*初始化一个按键和一个 IO
 *按键连接外部中断,控制LED相应
 */


CLRIRLed::CLRIRLed() {
	// TODO Auto-generated constructor stub
	//IRQ_INIT();
	pwm = new CLRPWMCtrl(4);
}

CLRIRLed::~CLRIRLed() {
	// TODO Auto-generated destructor stub
}

void CLRIRLed::setLedLight(char step)
{
	pwm->pwmSet(step);
}
