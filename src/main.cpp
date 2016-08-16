//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

extern "C" {
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f10x.h"
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "usart.h"

#include "CLRCppTest.h"
#include "./i2c/CLRI2CInterface.h"
#include "CLRMPU9250.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.

uint32_t m_nStart;               //DEBUG Stopwatch start cycle counter value
uint32_t m_nStop;                //DEBUG Stopwatch stop cycle counter value

#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))

#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define  DEM_CR      *(volatile u32 *)0xE000EDFC
#define  DEM_CR_TRCENA                   (1 << 24)

#define STOPWATCH_START { m_nStart = *((volatile unsigned int *)0xE0001004);}//DWT_CYCCNT;}
#define STOPWATCH_STOP  { m_nStop = *((volatile unsigned int *)0xE0001004);}

static inline void stopwatch_reset(void)
{
	/* Enable DWT */
	DEMCR |= DEMCR_TRCENA;
	*DWT_CYCCNT = 0;
	/* Enable CPU cycle counter */
	DWT_CTRL |= CYCCNTENA;
	DEM_CR |=  DEM_CR_TRCENA;
}

static inline uint32_t stopwatch_getticks()
{
	return CPU_CYCLES;
}

static inline void stopwatch_delay(uint32_t ticks)
{
	stopwatch_reset();
	while(1)
	{
		if (stopwatch_getticks() >= ticks)
			break;
	}
}

//uint32_t CalcNanosecondsFromStopwatch(uint32_t nStart, uint32_t nStop)
uint32_t CalcUsecondsFromStopwatch(uint32_t nStart, uint32_t nStop)
{
	uint32_t nTemp;
	uint32_t n;

	nTemp = nStop - nStart;

	//nTemp *= 1000;                          // Scale cycles by 1000.
	n = SystemCoreClock / 1000000;          // Convert Hz to MHz. SystemCoreClock = 168000000

	nTemp = nTemp / n;                      // nanosec = (Cycles * 1000) / (Cycles/microsec)

	return nTemp;
}

void getArray(uint8_t *allData, uint16_t vDataLength, uint8_t *vData)
{

}

unsigned char getChar(uint8_t *string){
	if(string[0] == '1'){
		return 1;
	} else if(string[0] == '2'){
		return 2;
	} else if(string[0] == '3'){
		return 3;
	} else if(string[0] == '4'){
		return 4;
	} else if(string[0] == '5'){
		return 5;
	} else if(string[0] == '6'){
		return 6;
	} else if(string[0] == '7'){
		return 7;
	} else if(string[0] == '8'){
		return 8;
	} else if(string[0] == '9'){
		return 9;
	}else{
		return 0;
	}
}

void init_I2C()
{
	I2C_Cmd(I2C1, ENABLE);

}

//uart + imu
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
int main(int argc, char* argv[])
{
	timer_start();
	CLRI2CInterface *i2c = new CLRI2CInterface();
	uint8_t buff[128];
	strcpy((char*)&COM_INFO[COM1]->name, "COM1");
	COM_INFO[COM1]->baudrate = 115200;
	COM_INFO[COM1]->onoff = 1;
	COM_INFO[COM1]->rxstatus = 0;

	COM_INFO[COM1]->rxnbr = 0;
	COM_INFO[COM1]->rxsize = 128;//AT_RESP_SIZE;
	COM_INFO[COM1]->rxbuf = buff;

	COM_Init(COM1, 115200);
	uint8_t	putS[20] = "jiangbo\r\n";
	//std::string *putS;// =  new std::string;  //"jiangbo";
	int loop = 0;
	char *loopChar;
	char read_data = 0;
	/*
    while(1){
		//sprintf(loopChar,"%d\r\n",loop++);
		//putS->append(loopChar);
		//putS->append("\r\n");
		COM_PutStr(COM1, putS);
		//COM_PutStr(COM1, (uint8_t *)loopChar);
		stopwatch_delay(SystemCoreClock);
		//I2C_EE_BufferRead(&read_data,0x75,1);
		i2c->read(0x75, &read_data, 1, 0);
		trace_printf("read = 0x%x\n", read_data);
		read_data = 0;

		trace_puts("Hello ARM World!\n");
	}
	 */

	//t.start();
	//lcd.init();
	//lcd.setBrightness(0.05);
	CLRMPU9250 *mpu9250 = new CLRMPU9250();

	// Read the WHO_AM_I register, this is a good test of communication
	uint8_t whoami = mpu9250->readByte(MPU9250_ADDRESS, 0x75);  // Read WHO_AM_I register for MPU-9250
	trace_printf("I AM 0x%x\n", whoami);
	trace_printf("I SHOULD BE 0x71\n");

	if (whoami == 0x71) // WHO_AM_I should always be 0x68
	{
		trace_printf("MPU9250 is online...\n\r");
		wait(1);
		//lcd.clear();
		//lcd.printString("MPU9250 OK", 0, 0);

		mpu9250->resetMPU9250(); // Reset registers to default in preparation for device calibration
		mpu9250->calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
		trace_printf("x gyro bias = %f\n\r", gyroBias[0]);
		trace_printf("y gyro bias = %f\n\r", gyroBias[1]);
		trace_printf("z gyro bias = %f\n\r", gyroBias[2]);
		trace_printf("x accel bias = %f\n\r", accelBias[0]);
		trace_printf("y accel bias = %f\n\r", accelBias[1]);
		trace_printf("z accel bias = %f\n\r", accelBias[2]);
		wait(2);
		mpu9250->initMPU9250();
		trace_printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
		mpu9250->initAK8963(mpu9250->magCalibration);
		trace_printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
		trace_printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<Ascale));
		trace_printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<Gscale));
		if(Mscale == 0) trace_printf("Magnetometer resolution = 14  bits\n\r");
		if(Mscale == 1) trace_printf("Magnetometer resolution = 16  bits\n\r");
		if(Mmode == 2) trace_printf("Magnetometer ODR = 8 Hz\n\r");
		if(Mmode == 6) trace_printf("Magnetometer ODR = 100 Hz\n\r");
		wait(2);
	}
	else
	{
		trace_printf("Could not connect to MPU9250: \n\r");
		trace_printf("%#x \n",  whoami);

		//lcd.clear();
		//lcd.printString("MPU9250", 0, 0);
		//lcd.printString("no connection", 0, 1);
		//lcd.printString("0x", 0, 2);  lcd.setXYAddress(20, 2); lcd.printChar(whoami);

		while(1) ; // Loop forever if communication doesn't happen
	}

	mpu9250->getAres(); // Get accelerometer sensitivity
	mpu9250->getGres(); // Get gyro sensitivity
	mpu9250->getMres(); // Get magnetometer sensitivity
	trace_printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/aRes);
	trace_printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/gRes);
	trace_printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mRes);
	magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
	magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

	while(1) {

		// If intPin goes high, all data registers have new data
		if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

			mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
			// Now we'll calculate the accleration value into actual g's
			ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
			ay = (float)accelCount[1]*aRes - accelBias[1];
			az = (float)accelCount[2]*aRes - accelBias[2];

			mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
			// Calculate the gyro value into actual degrees per second
			gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
			gy = (float)gyroCount[1]*gRes - gyroBias[1];
			gz = (float)gyroCount[2]*gRes - gyroBias[2];

			mpu9250.readMagData(magCount);  // Read the x/y/z adc values
			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental corrections
			mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
			my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
			mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
		}

		Now = t.read_us();
		deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
		lastUpdate = Now;

		sum += deltat;
		sumCount++;

		//    if(lastUpdate - firstUpdate > 10000000.0f) {
		//     beta = 0.04;  // decrease filter gain after stabilized
		//     zeta = 0.015; // increasey bias drift gain after stabilized
		//   }

		// Pass gyro rate as rad/s
		mpu9250.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
		// mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

		// Serial print and/or display at 0.5 s rate independent of data rates
		delt_t = t.read_ms() - count;
		if (delt_t > 500) { // update LCD once per half-second independent of read rate

			pc.printf("ax = %f", 1000*ax);
			pc.printf(" ay = %f", 1000*ay);
			pc.printf(" az = %f  mg\n\r", 1000*az);

			pc.printf("gx = %f", gx);
			pc.printf(" gy = %f", gy);
			pc.printf(" gz = %f  deg/s\n\r", gz);

			pc.printf("gx = %f", mx);
			pc.printf(" gy = %f", my);
			pc.printf(" gz = %f  mG\n\r", mz);

			tempCount = mpu9250.readTempData();  // Read the adc values
			temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
			pc.printf(" temperature = %f  C\n\r", temperature);

			pc.printf("q0 = %f\n\r", q[0]);
			pc.printf("q1 = %f\n\r", q[1]);
			pc.printf("q2 = %f\n\r", q[2]);
			pc.printf("q3 = %f\n\r", q[3]);

			lcd.clear();
			lcd.printString("MPU9250", 0, 0);
			lcd.printString("x   y   z", 0, 1);
			lcd.setXYAddress(0, 2); lcd.printChar((char)(1000*ax));
			lcd.setXYAddress(20, 2); lcd.printChar((char)(1000*ay));
			lcd.setXYAddress(40, 2); lcd.printChar((char)(1000*az)); lcd.printString("mg", 66, 2);


			// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
			// In this coordinate system, the positive z-axis is down toward Earth.
			// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
			// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
			// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
			// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
			// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
			// applied in the correct order which for this configuration is yaw, pitch, and then roll.
			// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
			yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
			pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
			roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
			pitch *= 180.0f / PI;
			yaw   *= 180.0f / PI;
			yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
			roll  *= 180.0f / PI;

			trace_printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
			trace_printf("average rate = %f\n\r", (float) sumCount/sum);

			count = t.read_ms();
			sum = 0;
			sumCount = 0;
		}
	}
}

//uart + led
uint16_t led_mask;
#if 0
int main(int argc, char* argv[])
{
	trace_puts("Hello ARM World!\n");

	//CLRCppTest *clrt = new CLRCppTest();

	/*
	timer_start();
	blink_led_init();
	blink_tracking_led_init();

	uint8_t buff[128];

	strcpy((char*)&COM_INFO[COM1]->name, "COM1");
	COM_INFO[COM1]->baudrate = 115200;
	COM_INFO[COM1]->onoff = 1;
	COM_INFO[COM1]->rxstatus = 0;

	COM_INFO[COM1]->rxnbr = 0;
	COM_INFO[COM1]->rxsize = 128;//AT_RESP_SIZE;
	COM_INFO[COM1]->rxbuf = buff;

	COM_Init(COM1, 115200);

	int len = 0;
	int i = 0;

	int loopNum = 0;
	char rdata[100];
	int rdataLoop = 0;
	char turnLight = 0;

	while(1){
		if(COM_INFO[COM1]->rxstatus & 0x8000){
			uint8_t vData[100] = {0};
			COM_INFO[COM1]->rxbuf[COM_INFO[COM1]->rxnbr] = '\0';
			led_mask = (uint16_t)atoi((char *)COM_INFO[COM1]->rxbuf)&0xff;
			//unsigned char const led_mask = getChar(COM_INFO[COM1]->rxbuf); //(unsigned char)atoi((char *)COM_INFO[COM1]->rxbuf)&0xff;
			//trace_printf("vData    = %s\n", vData);
			trace_printf("led_mask = 0x%x\n", led_mask);	//不print  led_mask 很可能是0!!!!
			COM_INFO[COM1]->rxstatus = 0;					//清除接收状态字
			COM_INFO[COM1]->rxnbr = 0;						// 清除接收计数
			blink_set_tracking_led(led_mask);
		} else {
			//trace_printf("rxstatus  0x%x\n",COM_INFO[COM1]->rxstatus);
		}
	}
	*/
}
#endif
//led test
#if 0
int main(int argc, char* argv[])
{
	trace_puts("Hello ARM World!\n");
	timer_start();
	blink_led_init();
	blink_tracking_led_init();

	while(1){
		/*
		blink_led_on();
		blink_set_tracking_led(0x05);
		timer_sleep(BLINK_OFF_TICKS);
		blink_led_off();
		blink_set_tracking_led(0x06);
		timer_sleep(BLINK_OFF_TICKS);
		blink_set_tracking_led(0x08);
		timer_sleep(BLINK_OFF_TICKS);
		 */
		blink_set_tracking_led(0x01);
		timer_sleep(BLINK_OFF_TICKS);
		blink_set_tracking_led(0x02);
		timer_sleep(BLINK_OFF_TICKS);
		blink_set_tracking_led(0x04);
		timer_sleep(BLINK_OFF_TICKS);
		blink_set_tracking_led(0x08);
		timer_sleep(BLINK_OFF_TICKS);

	}
}
#endif

/*
 * timer
int
main(int argc, char* argv[])
{
	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	timer_start();

	blink_led_init();

	uint32_t seconds = 0;
	int timeDiff = 0;

	stopwatch_reset();

	// Infinite loop
	while (1)
	{

		blink_led_on();
		STOPWATCH_START;
		timer_sleep(seconds == 0 ? TIMER_FREQUENCY_HZ : BLINK_ON_TICKS);
		STOPWATCH_STOP;

		blink_led_off();
		timer_sleep(BLINK_OFF_TICKS);


		//blink_led_on();
		//stopwatch_delay(SystemCoreClock);
		timeDiff = CalcUsecondsFromStopwatch(m_nStart, m_nStop);
		trace_printf("My function took %f seconds\n", (float)timeDiff/(1000.0*1000.0));
		trace_printf(" m_nStart = %d\n", m_nStart);
		trace_printf(" m_nStop  = %d\n", m_nStop);
		trace_printf("My function took %d useconds\n", timeDiff);


		//blink_led_off();
		//stopwatch_delay(SystemCoreClock);


		++seconds;

		// Count seconds on the trace device.
		trace_printf("Second %u\n", seconds);
		//trace_printf("My function took %d nanoseconds\n", timeDiff);
	}
	// Infinite loop, never return.
}
 */

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
}
