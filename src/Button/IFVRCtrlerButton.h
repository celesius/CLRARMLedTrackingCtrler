/*
 * IFVRCtrlerButton.h
 *
 *  Created on: 2016年11月1日
 *      Author: clover
 */

#ifndef BUTTON_IFVRCTRLERBUTTON_H_
#define BUTTON_IFVRCTRLERBUTTON_H_
#include <stdint.h>

class IFVRCtrlerButton {
	uint32_t bAcnt;
	uint32_t bBcnt;
public:
	IFVRCtrlerButton();
	virtual ~IFVRCtrlerButton();
	int getButtonA();
	int getButtonB();
};

#endif /* BUTTON_IFVRCTRLERBUTTON_H_ */
