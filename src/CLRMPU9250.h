/*
 * CLRMPU9250.h
 *
 *  Created on: 2016年8月12日
 *      Author: clover
 */

#ifndef CLRMPU9250_H_
#define CLRMPU9250_H_

#include <stdio.h>
#include "./i2c/CLRI2CInterface.h"

class CLRMPU9250 {
public:
	CLRMPU9250();
	virtual ~CLRMPU9250();
	char readByte(uint8_t address, uint8_t subAddress);
	void resetMPU9250();
	void initMPU9250();
	void MadgwickQuaternionUpdate(float adelay,float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	void initAK8963(float * destination);
	void getMres();
	void getGres();
	void getAres();

	float magCalibration[3];
private:
	CLRI2CInterface *i2c;
	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
	void readAccelData(int16_t * destination);
	void readGyroData(int16_t * destination);
	void readMagData(int16_t * destination);
	int16_t readTempData();
	void calibrateMPU9250(float * dest1, float * dest2);
	void MPU9250SelfTest(float * destination);
	void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
};

#endif /* CLRMPU9250_H_ */
