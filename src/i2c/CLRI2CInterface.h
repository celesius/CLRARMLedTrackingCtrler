/*
 * CLRI2CInterface.h
 *
 *  Created on: 2016年8月12日
 *      Author: clover
 */
#ifndef CLRI2CINTERFACE_H_
#define CLRI2CINTERFACE_H_
#include "stm32f10x.h"

class CLRI2CInterface {
public:
	CLRI2CInterface();
	virtual ~CLRI2CInterface();
    void write(uint8_t devAddress, uint8_t address,uint8_t *data_write, int data_count, char stop_flag); // no stop
    bool read(uint8_t devAddress, uint8_t address, uint8_t *data_read, int data_count, char stop_flag);
private:

};

#endif /* CLRI2CINTERFACE_H_ */
