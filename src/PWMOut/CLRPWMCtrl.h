/*
 * CLRPWMCtrl.h
 *
 *  Created on: 2016年8月25日
 *      Author: clover
 */

#ifndef PWMOUT_CLRPWMCTRL_H_
#define PWMOUT_CLRPWMCTRL_H_

//最新的硬件 LED连接在 PB1上
class CLRPWMCtrl {
public:
	CLRPWMCtrl(int pinNum);
	virtual ~CLRPWMCtrl();
	void run();
	void pwmSet(char data);
};

#endif /* PWMOUT_CLRPWMCTRL_H_ */
