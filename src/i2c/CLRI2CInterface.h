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
	//CLRI2CInterface(struct CLR_I2C_port a_i2c_port);
	CLRI2CInterface();
	CLRI2CInterface(uint16_t speed);
	virtual ~CLRI2CInterface();
    void write(uint8_t devAddress, uint8_t address,uint8_t *data_write, int data_count, char stop_flag); // no stop
    void writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
    void writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    void writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
    void writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
    void writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);

    bool read(uint8_t devAddress, uint8_t address, uint8_t *data_read, int data_count, char stop_flag);
    bool readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
    bool readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
    //bool readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout);
    bool readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
    bool readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
private:
    //struct CLR_I2C_port m_i2c_port;

};

#endif /* CLRI2CINTERFACE_H_ */
