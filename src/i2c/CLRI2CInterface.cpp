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
	I2C_Soft_Init();
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

}
