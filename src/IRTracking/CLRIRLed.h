/*
 * CLRIRLed.h
 *
 *  Created on: 2016年8月25日
 *      Author: clover
 */

#ifndef IRTRACKING_CLRIRLED_H_
#define IRTRACKING_CLRIRLED_H_
#include "../PWMOut/CLRPWMCtrl.h"

class CLRIRLed {
public:
	CLRIRLed();
	virtual ~CLRIRLed();
private:
	void setLedLight(char step);
	CLRPWMCtrl *pwm;
};

#endif /* IRTRACKING_CLRIRLED_H_ */
