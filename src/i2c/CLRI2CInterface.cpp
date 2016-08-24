/*
 * CLRI2CInterface.cpp
 *
 *  Created on: 2016年8月12日
 *      Author: clover
 */

#include "CLRI2CInterface.h"
#include "diag/Trace.h"
extern "C" {
#include "i2c.h"
I2C_StatusTypeDef status;

#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0

CLRI2CInterface::CLRI2CInterface() {
	// TODO Auto-generated constructor stub
	I2C_Soft_Init(100);
}

CLRI2CInterface::CLRI2CInterface(uint16_t speed) {
	// TODO Auto-generated constructor stub
	I2C_Soft_Init(speed);
}

CLRI2CInterface::~CLRI2CInterface() {
	// TODO Auto-generated destructor stub
}

//void CLRI2CInterface::write(int address,char *data_write, int data_count, char stop_flag)
void CLRI2CInterface::write(uint8_t devAddress, uint8_t address,uint8_t *data_write, int data_count, char stop_flag) // no stop
{
	status = I2C_WriteBurst(devAddress, address, (uint8_t *)data_write, (unsigned long)data_count);
	if(status != I2C_SUCCESS){
		if(status == I2C_TIMEOUT)
			trace_puts("WRITE I2C_TIMEOUT\n");
		else
			trace_puts("WRITE I2C_ERR\n");
	}
}

void CLRI2CInterface::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
	write(devAddr, regAddr, &data, 1, 0);
}

void CLRI2CInterface::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data){
	I2C_WriteBit(devAddr, regAddr , bitNum, data);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
void CLRI2CInterface::writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        writeByte(devAddr, regAddr, b);
    } else {
    	trace_puts("writeBits err!\n");
        //return false;
    }
}

void CLRI2CInterface::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
	uint8_t dataWrite[2] = {0};
	uint8_t tempData = 0;
	//先发送高8位
	tempData = (uint8_t)data>>8;
	writeByte(devAddr, regAddr, tempData);
	tempData = (uint8_t)(data & 0x00ff);
	writeByte(devAddr, regAddr, tempData);
}

void CLRI2CInterface::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
	write(devAddr, regAddr, data, length, 0); // no stop
}


//bool CLRI2CInterface::read(int address, char *data_read, int data_count, char stop_flag)
bool CLRI2CInterface::read(uint8_t devAddress, uint8_t address, uint8_t *data_read, int data_count, char stop_flag)
{
	//status = I2C_ReadOneByte( MPU9250_ADDRESS, (uint8_t)address, (uint8_t *)data_read);
	status = I2C_ReadBurst(devAddress, address, (uint8_t *)data_read, (unsigned long)data_count);
	if(status == I2C_SUCCESS){
		return true;
	}else{
		if(status == I2C_TIMEOUT)
			trace_puts("READ I2C_TIMEOUT\n");
		else
			trace_puts("READ I2C_ERR\n");
		return false;
	}
}

bool CLRI2CInterface::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	return read(devAddr, regAddr, data, length, 0);
}

bool CLRI2CInterface::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data)
{
	return read(devAddr, regAddr, data,1,0);
}

/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
bool CLRI2CInterface::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    //uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    bool isReadOk = read(devAddr, regAddr, &b,1,0);
    *data = b & (1 << bitNum);
    return isReadOk;
}

bool CLRI2CInterface::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{

	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t b;
	bool count;
	if ((count = readByte(devAddr, regAddr, &b)) != 0) {
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}else{
		trace_puts("readBits err\n");
	}
	return count;
}

} //end  extern "C"
