/*
 * i2c_ee.h
 *
 *  Created on: 2016年8月15日
 *      Author: clover
 */

#ifndef I2C_EE_H_
#define I2C_EE_H_

#include "stm32f10x.h"

void I2C_EE_Init();
void I2C_EE_BufferRead(u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);

#endif /* I2C_EE_H_ */
