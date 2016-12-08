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

#define INT_STATUS 0x3A
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
	void calibrateMPU9250(float * dest1, float * dest2);
	void wait(float time);
	void readAccelData(int16_t * destination);
	void readGyroData(int16_t * destination);
	void readMagData(int16_t * destination);
	int16_t readTempData();
	void loop(float *q,float deltat);

	float magCalibration[3];
	float accelBias[3];
	float gyroBias[3]; // Bias corrections for gyro and accelerometer
	float magbias[3];  // Factory mag calibration and mag bias
	uint8_t Ascale;
	uint8_t Gscale;
	uint8_t Mscale;
	uint8_t Mmode;
	float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
	int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
	float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
	float q[4];           // vector to hold quaternion
private:
	void setup();
	void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
	void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
	void MPU9250SelfTest(float * destination);
	void MahonyQuaternionUpdate(float deltat, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

	CLRI2CInterface *i2c;
	//struct CLR_I2C_port i2c_port;
};

#endif /* CLRMPU9250_H_ */
