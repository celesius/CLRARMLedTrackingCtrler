/*
 * CLRMPU6050DMP.h
 *
 *  Created on: 2016年8月22日
 *      Author: clover
 */

#ifndef MPU6050DMP_CLRMPU6050DMP_H_
#define MPU6050DMP_CLRMPU6050DMP_H_

#include "diag/Trace.h"

//#include "MPU6050_6Axis_MotionApps20"
#include "MPU6050.h"

class CLRMPU6050DMP {
public:
	CLRMPU6050DMP();
	virtual ~CLRMPU6050DMP();
	void setup();
	bool loop(int16_t *q);
private:
	MPU6050 *mpu;
	//int32_t q[4];
	//unsigned char serialString[100];

};

#endif /* MPU6050DMP_CLRMPU6050DMP_H_ */
