/*
 * CLRI2C.h
 *
 *  Created on: 2016年8月11日
 *      Author: clover
 */

#ifndef CLRI2C_H_
#define CLRI2C_H_

#include "stm32f10x.h"

class CLRI2C {
public:
	CLRI2C();
	virtual ~CLRI2C();
private :
	/*
	 * Foo
	 * */
	void RCC_Configuration(void);
	void NVIC_Configuration(void);
	void GPIO_Configuration(void);
	/*
	 * data
	 * */
	ErrorStatus HSEStartUpStatus;

};

#endif /* CLRI2C_H_ */
