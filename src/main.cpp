//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Button/IFVRCtrlerButton.h"
#include "IFVRCtrlerSys.h"
extern "C" {
#include "stm32f10x.h"
#include "diag/Trace.h"
//#include <cmsis_armcc.h>

#include "Timer.h"
#include "BlinkLed.h"
#include "usart.h"
#include "ADC/CLRADC.h"

//#include "CLRCppTest.h"
//#include "./i2c/CLRI2CInterface.h"
//#include "CLRMPU9250.h"
//#include "MPU6050DMP/CLRMPU6050DMP.h"

//#include "IRTracking/CLRIRLed.h"
//#include "PWMOut/CLRPWMCtrl.h"
#include "IRTracking/CLRIRLed.h"

#include "bluetooth.h"

#include "common/delay/delay.h"

#include "TSM12/CLRTouchpad.h"

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

//#define BLUETOOTHTEST
//#define COMTEST
#define TOUCHPADTEST

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

float getSecond(){
	STOPWATCH_START;
	uint32_t nTemp;
	uint32_t n;
	n = SystemCoreClock / 1000000;          // Convert Hz to MHz. SystemCoreClock = 168000000
	nTemp = m_nStart / n;
	return  (float)nTemp/(1000.0*1000.0);
}

float getMs(){
	STOPWATCH_START;
	uint32_t nTemp;
	uint32_t n;
	n = SystemCoreClock / 1000000;          // Convert Hz to MHz. SystemCoreClock = 168000000
	nTemp = m_nStart / n;
	return  (float)nTemp/(1000.0);
}

float getUs(uint32_t in ){
	//STOPWATCH_START;
	uint32_t nTemp;
	uint32_t n;
	n = SystemCoreClock / 1000000;          // Convert Hz to MHz. SystemCoreClock = 168000000
	nTemp = in / n;
	return  (float)nTemp;
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

//中断处理程序
void EXTI15_10_IRQHandler(void)
//void EXTI0_IRQHandler(void)
{
    //if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    //{
    //    EXTI_ClearITPendingBit(EXTI_Line11); //清除标志
    //}
    //trace_printf("key down !!!!\n");
    //EXTI_ClearITPendingBit(EXTI_Line0); //清除标志
	uint8_t	putS[60] = "jiangbo\r\n";
    if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
        trace_printf("key down !!!!\n");
		//COM_PutStr(COM1, putS);
        EXTI_ClearITPendingBit(EXTI_Line12); //清除标志
    }
}

void SetupSleep(void);
void ClearSleep(void);

#ifndef BLUETOOTHTEST
//CLRPWMCtrl *pwd = new CLRPWMCtrl();
#endif
void EXTI0_IRQHandler(void)
{

	int a = 100;
	if (EXTI_GetITStatus(EXTI_Line0) != RESET){
		trace_printf("button A on\n");
		ExitSleepMode();
		//SetupSleep();
	  //__WFI();
		//PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);
		while(0){
			if(a-- == 0){
				break;
			}
			trace_printf(" a = %d\n", a);
		}

		EXTI_ClearITPendingBit(EXTI_Line0); //清除标志
	}

#if 0
	static int isUp = 0;
	char blueSendData[100] = {0};// = "AT+VERSION?\r\n";
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		//trace_printf("key down IRQ0 !!!!\n");
		if(isUp){
			//ctrlIOUp();
			//AT_Mode();
			trace_printf("key down UP !!!!\n");
		strcpy(blueSendData, "AT+ADCN?\r\n");
		COM_PutStr(AT_COM, (uint8_t *)blueSendData);
		}else{
			//ctrlIODown();
		//strcpy(blueSendData, "AT+BIND?\r\n");
		//strcpy(blueSendData, "AT+ADCN?\r\n");
		//COM_PutStr(AT_COM, (uint8_t *)blueSendData);

		strcpy(blueSendData, "AT+STATE?\r\n");
		COM_PutStr(AT_COM, (uint8_t *)blueSendData);
			trace_printf("key down DOWN !!!!\n");
		}


		unsigned char send[20] = "nihao \r\n";

		//COM_PutStr(COM1, send);
		isUp = abs(isUp - 1);  //!isUp;
		EXTI_ClearITPendingBit(EXTI_Line0); //清除标志
	}
#endif
}

void EXTI1_IRQHandler(void){
	if (EXTI_GetITStatus(EXTI_Line1) != RESET){
		trace_printf("button B on\n");
		EXTI_ClearITPendingBit(EXTI_Line1); //清除标志
	}
}

void blink_led_prems(int ms){
	static bool led_on = true;
	delay_ms(ms);
	if(led_on){
		blink_led_on();
		//ctrlIOUp();
	}
	else{

		blink_led_off();
		//ctrlIODown();
	}
	led_on = !led_on;

}

#ifdef TOUCHPADTEST
int main(int argc, char* argv[])
{
	IFVRCtrlerInit();
	Delay_Init();
	blink_led_init();
	trace_printf("start init pad\n");
	CLRTouchpad *m_touchpad = new CLRTouchpad();
	trace_printf("end init pad\n");

	//CLRADC *m_adc = new CLRADC();
	while(1)
	{
		blink_led_prems(50);
		//trace_printf("adc = %d\n", m_adc->get_adc_value());
		m_touchpad->run_loop();
	}
}
#endif

#ifdef BLUETOOTHTEST
int main(int argc, char* argv[])
{
	//PWR_DeInit();
	//SystemInit();
	IFVRCtrlerInit();
	Delay_Init();
	blink_led_init();
	//timer_start();
	//CLRIRLed *ir = new CLRIRLed();
	//ctrlIOUp();
	IFVRCtrlerButton *button = new IFVRCtrlerButton();
	//uint32_t loop = 0;
	InitCOM1();
	char usart1Puts[100] = {0};

	BT_Init();
	ATSetup();
	//ATCheck();

	//CLRIRLed *ifvrLed = new CLRIRLed();

	bool led_on = true;

	while(1){
		//trace_printf("main loop\n");
		//trace_printf("aaa == %d\n",button->getButtonA());
		//trace_printf("bbb == %d\n",button->getButtonB());
		/*
		//进入休眠模式
		if(button->getButtonB()){
			trace_printf("get A\n");
			EnterSleepMode();
			__WFI();
		}*/

		//trace_printf(" Bluetooth state = %d\n", BTGetStatus());

		//timer_sleep(TIMER_FREQUENCY_HZ/2);
		delay_ms(50);
		if(led_on){
			blink_led_on();
			//ctrlIOUp();
		}
		else{

			blink_led_off();
			//ctrlIODown();
		}
		led_on = !led_on;

		if((COM_INFO[AT_COM]->rxstatus & 0x8000))
		{
			trace_printf("%s\n",COM_INFO[COM2]->rxbuf);
			sprintf(usart1Puts,"%s\r\n",COM_INFO[COM2]->rxbuf);
			COM_PutStr(COM1,(uint8_t *)usart1Puts);
			COM_INFO[COM2]->rxstatus = 0;	//清除接收状态字
			COM_INFO[COM2]->rxnbr = 0;		//清除数据接收计数值
		} else {
			//trace_printf("no get \n");
		}

/*
		char sendChar[12];
		trace_printf("will send\n");
		sprintf(sendChar, "send=%d\r\n", loop++);
		COM_PutStr(COM1, (uint8_t*)sendChar);
		trace_printf("send %s \n", sendChar);
*/
		//delay_ms(  1000);
/*
		if((BT_LED == 1) && (COM_INFO[AT_COM]->rxstatus & 0x8000))
		{
			// Show COM gets
			if(COM_INFO[AT_COM]->rxnbr > 16)//OLED一行最多显示16个字符
				COM_INFO[AT_COM]->rxbuf[15] = '\0';//添加字符串结束符
			else
				COM_INFO[AT_COM]->rxbuf[COM_INFO[AT_COM]->rxnbr] = '\0';//添加字符串结束符

			trace_printf("%s\n",COM_INFO[AT_COM]->rxbuf);

			// Do something after info
			//Check_and_SetLed();

			// clear COM status
			COM_INFO[AT_COM]->rxstatus = 0;	//清除接收状态字
			COM_INFO[AT_COM]->rxnbr = 0;		//清除数据接收计数值

		}
		*/

	}

	return 0;
}
#endif


//IR tracking
#ifdef COMTEST
int main(int argc, char* argv[])
{
	timer_start();
	blink_led_init();
	initCOM1();
	bool led_on = true;
	CLRIRLed *ir = new CLRIRLed();
	uint8_t	putS[60] = "jiangbo\r\n";
	uint8_t getS[60] = {0};
	unsigned int loopCnt = 0;
	pwd->run();
	while(1){
		timer_sleep(TIMER_FREQUENCY_HZ/2);
		if(led_on)
			blink_led_on();
		else
			blink_led_off();

		sprintf((char *)putS,"jiangbo %d \r\n",loopCnt++);

		COM_PutStr(COM1, putS);

		//uint8_t a = COM_Get(COM1, (uint8_t *)getS, 100);
		//trace_printf(" get data = %s \n", getS);

		led_on = !led_on;

	}

}
#endif

#if 0
void USART1_IRQHandler(void)
{
	trace_printf("in irq\n");
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)	//
	{
		trace_printf("!Reset\n");
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
#endif
#if 0
//DMP6050 或者9250 目前不带地磁矫正
int main(int argc, char* argv[])
{
	timer_start();
	initCOM1();
	CLRMPU6050DMP *mpu = new CLRMPU6050DMP();
	//CLRMPU9250 *mpu = new CLRMPU9250();
	uint8_t Noo[20] = "no---\r\n";
	uint8_t	putS[60] = {0};

	float q[4] = {0};
	while(1){
		if(mpu->loop(q)){
			sprintf((char *)putS,"quat#%f#%f#%f#%f\r\n",q[0],q[1],q[2],q[3]);
			COM_PutStr(COM1, putS);
		}else{
			COM_PutStr(COM1, Noo);
		}
	}
}
#endif

//中断测试
/*
int main(int argc, char* argv[])
{
	timer_start();
	blink_led_init();
	RCC_INIT();
	GPIO_INIT();
	NVIC_INIT();
	EXTI_INIT();
	initCOM1();
	bool led_on = true;
	while(1){
		timer_sleep(TIMER_FREQUENCY_HZ/2);
		if(led_on)
			blink_led_on();
		else
			blink_led_off();
		led_on = !led_on;
	}
}
*/

//uart + imu
#if 0
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
int main(int argc, char* argv[])
{
	timer_start();
	blink_led_init();
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
	uint8_t	putS[60] = "jiangbo\r";
	//std::string *putS;// =  new std::string;  //"jiangbo";
	int loop = 0;
	char *loopChar;
	uint8_t read_data = 0;

	float lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval                               // used to calculate integration interval

	float sum = 0.0;
	uint32_t sumCount = 0;
	int delt_t = 0; // used to control display output rate
	int count = 0;  // used to control display output rate
	int16_t tempCount;   // Stores the real internal chip temperature in degrees Celsius
	float temperature;
	const float PI = 3.14159265358979323846f;
	float pitch, yaw, roll = 0.0;
	float deltat = 0.0f;                             // integration interval for both filter schemes

	float tempTime = 0;
	float lastTime = 0;
	bool led_on = true;
	while(0){
		//STOPWATCH_START;
		timer_sleep(TIMER_FREQUENCY_HZ/2);
		//STOPWATCH_STOP;

		//STOPWATCH_START;
		//dd = m_nStart - tempTime;
		//float timeDiff = CalcUsecondsFromStopwatch(m_nStart, tempTime);
		//tempTime = m_nStart;
		//tempTime = getMs(m_nStart);
		tempTime = getMs();
		float timeDiff = tempTime - lastTime;
		lastTime = tempTime;
		//trace_printf("My function took %f seconds\n", (float)timeDiff/(1000.0*1000.0));
		trace_printf("My function took %f seconds\n", timeDiff);
	}
    while(0){
		//sprintf(loopChar,"%d\r\n",loop++);
		//putS->append(loopChar);
		//putS->append("\r\n");
		COM_PutStr(COM1, putS);
		//COM_PutStr(COM1, (uint8_t *)loopChar);
		stopwatch_delay(SystemCoreClock);
		//I2C_EE_BufferRead(&read_data,0x75,1);
		i2c->read( MPU9250_ADDRESS,0x75, &read_data, 1, 0);
		trace_printf("read = 0x%x\n", read_data);
		read_data = 0;

		if(led_on)
			blink_led_on();
		else
			blink_led_off();
		led_on = !led_on;

		trace_puts("Hello ARM World!\n");
	}

	//t.start();
	//lcd.init();
	//lcd.setBrightness(0.05);

#if 1

	CLRMPU9250 *mpu9250 = new CLRMPU9250();

	// Read the WHO_AM_I register, this is a good test of communication
	uint8_t whoami = mpu9250->readByte(MPU9250_ADDRESS, 0x75);  // Read WHO_AM_I register for MPU-9250
	trace_printf("I AM 0x%x\n", whoami);
	trace_printf("I SHOULD BE 0x71\n");

	if (whoami == 0x71) // WHO_AM_I should always be 0x68
	{
		trace_printf("MPU9250 is online...\n\r");
		mpu9250->wait(1);
		//lcd.clear();
		//lcd.printString("MPU9250 OK", 0, 0);
		mpu9250->resetMPU9250(); // Reset registers to default in preparation for device calibration
		trace_printf("eeerr\n");
		mpu9250->calibrateMPU9250(mpu9250->gyroBias, mpu9250->accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
		trace_printf("x gyro bias = %f\n", mpu9250->gyroBias[0]);
		trace_printf("y gyro bias = %f\n", mpu9250->gyroBias[1]);
		trace_printf("z gyro bias = %f\n", mpu9250->gyroBias[2]);
		trace_printf("x accel bias = %f\n", mpu9250->accelBias[0]);
		trace_printf("y accel bias = %f\n", mpu9250->accelBias[1]);
		trace_printf("z accel bias = %f\n", mpu9250->accelBias[2]);
		mpu9250->wait(2);
		mpu9250->initMPU9250();
		trace_printf("MPU9250 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
		mpu9250->initAK8963(mpu9250->magCalibration);
		trace_printf("AK8963 initialized for active data mode....\n\r"); // Initialize device for active mode read of magnetometer
		trace_printf("Accelerometer full-scale range = %f  g\n\r", 2.0f*(float)(1<<mpu9250->Ascale));
		trace_printf("Gyroscope full-scale range = %f  deg/s\n\r", 250.0f*(float)(1<<mpu9250->Gscale));
		if(mpu9250->Mscale == 0) trace_printf("Magnetometer resolution = 14  bits\n\r");
		if(mpu9250->Mscale == 1) trace_printf("Magnetometer resolution = 16  bits\n\r");
		if(mpu9250->Mmode == 2) trace_printf("Magnetometer ODR = 8 Hz\n\r");
		if(mpu9250->Mmode == 6) trace_printf("Magnetometer ODR = 100 Hz\n\r");
		mpu9250->wait(2);
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
	trace_printf("Accelerometer sensitivity is %f LSB/g \n\r", 1.0f/mpu9250->aRes);
	trace_printf("Gyroscope sensitivity is %f LSB/deg/s \n\r", 1.0f/mpu9250->gRes);
	trace_printf("Magnetometer sensitivity is %f LSB/G \n\r", 1.0f/mpu9250->mRes);
	mpu9250->magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	mpu9250->magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
	mpu9250->magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

	while(1) {

		// If intPin goes high, all data registers have new data
		if(mpu9250->readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

			mpu9250->readAccelData(mpu9250->accelCount);  // Read the x/y/z adc values
			// Now we'll calculate the accleration value into actual g's
			mpu9250->ax = (float)mpu9250->accelCount[0]*mpu9250->aRes - mpu9250->accelBias[0];  // get actual g value, this depends on scale being set
			mpu9250->ay = (float)mpu9250->accelCount[1]*mpu9250->aRes - mpu9250->accelBias[1];
			mpu9250->az = (float)mpu9250->accelCount[2]*mpu9250->aRes - mpu9250->accelBias[2];

			mpu9250->readGyroData(mpu9250->gyroCount);  // Read the x/y/z adc values
			// Calculate the gyro value into actual degrees per second
			mpu9250->gx = (float)mpu9250->gyroCount[0]*mpu9250->gRes - mpu9250->gyroBias[0];  // get actual gyro value, this depends on scale being set
			mpu9250->gy = (float)mpu9250->gyroCount[1]*mpu9250->gRes - mpu9250->gyroBias[1];
			mpu9250->gz = (float)mpu9250->gyroCount[2]*mpu9250->gRes - mpu9250->gyroBias[2];

			mpu9250->readMagData(mpu9250->magCount);  // Read the x/y/z adc values
			// Calculate the magnetometer values in milliGauss
			// Include factory calibration per data sheet and user environmental corrections
			mpu9250->mx = (float)mpu9250->magCount[0]*mpu9250->mRes*mpu9250->magCalibration[0] - mpu9250->magbias[0];  // get actual magnetometer value, this depends on scale being set
			mpu9250->my = (float)mpu9250->magCount[1]*mpu9250->mRes*mpu9250->magCalibration[1] - mpu9250->magbias[1];
			mpu9250->mz = (float)mpu9250->magCount[2]*mpu9250->mRes*mpu9250->magCalibration[2] - mpu9250->magbias[2];
		}

		STOPWATCH_START;
		//Now = t.read_us();
		Now	= getUs(m_nStart);
		//Now	= getUs();
		deltat = (float)((Now - lastUpdate)/1000000.0f) ; // set integration time by time elapsed since last filter update
		lastUpdate = Now;
		//trace_printf("deltat = %f\n", deltat);

		sum += deltat;
		sumCount++;

		//    if(lastUpdate - firstUpdate > 10000000.0f) {
		//     beta = 0.04;  // decrease filter gain after stabilized
		//     zeta = 0.015; // increasey bias drift gain after stabilized
		//   }

		// Pass gyro rate as rad/s
		//trace_printf("My function took %f seconds\n", deltat);
		mpu9250->MadgwickQuaternionUpdate(deltat, mpu9250->ax, mpu9250->ay, mpu9250->az, mpu9250->gx*PI/180.0f, mpu9250->gy*PI/180.0f, mpu9250->gz*PI/180.0f,  mpu9250->my,  mpu9250->mx, mpu9250->mz);
		// mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

		yaw   = atan2(2.0f * (mpu9250->q[1] * mpu9250->q[2] + mpu9250->q[0] * mpu9250->q[3]), mpu9250->q[0] * mpu9250->q[0] + mpu9250->q[1] * mpu9250->q[1] - mpu9250->q[2] * mpu9250->q[2] - mpu9250->q[3] * mpu9250->q[3]);
		pitch = -asin(2.0f * (mpu9250->q[1] * mpu9250->q[3] - mpu9250->q[0] * mpu9250->q[2]));
		roll  = atan2(2.0f * (mpu9250->q[0] * mpu9250->q[1] + mpu9250->q[2] * mpu9250->q[3]), mpu9250->q[0] * mpu9250->q[0] - mpu9250->q[1] * mpu9250->q[1] - mpu9250->q[2] * mpu9250->q[2] + mpu9250->q[3] * mpu9250->q[3]);
		pitch *= 180.0f / PI;
		yaw   *= 180.0f / PI;
		yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
		roll  *= 180.0f / PI;

		//if(led_on)
		//	blink_led_on();
		//else
		//	blink_led_off();
		//led_on = !led_on;
		trace_printf("Yaw, Pitch, Roll: %f %f %f\n", yaw, pitch, roll);
		//sprintf((char *)putS,"%fp%fp%fpn\r",yaw,pitch,roll);
		//sprintf((char *)putS,"%fp%fp%fp%fpn\r",mpu9250->q[0],mpu9250->q[1],mpu9250->q[2],mpu9250->q[3]);
		sprintf((char *)putS,"quat#%f#%f#%f#%f\r\n",mpu9250->q[0],mpu9250->q[1],mpu9250->q[2],mpu9250->q[3]);
		COM_PutStr(COM1, putS);

		// Serial print and/or display at 0.5 s rate independent of data rates
#if 0
		STOPWATCH_START;
		int nowMs = (int)getMs(m_nStart);
		//delt_t = t.read_ms() - count;
		delt_t = nowMs - count;
		if (delt_t > 30) { // update LCD once per half-second independent of read rate
		//sprintf((char *)putS,"%fp%fp%fp%fpn\r",mpu9250->q[0],mpu9250->q[1],mpu9250->q[2],mpu9250->q[3]);
		//COM_PutStr(COM1, putS);



			STOPWATCH_START;
			count = (int)getMs(m_nStart);
			sum = 0;
			sumCount = 0;


			trace_printf("ax = %f", 1000*mpu9250->ax);
			trace_printf(" ay = %f", 1000*mpu9250->ay);
			trace_printf(" az = %f  mg\n\r", 1000*mpu9250->az);

			trace_printf("gx = %f", mpu9250->gx);
			trace_printf(" gy = %f", mpu9250->gy);
			trace_printf(" gz = %f  deg/s\n\r", mpu9250->gz);

			trace_printf("gx = %f", mpu9250->mx);
			trace_printf(" gy = %f", mpu9250->my);
			trace_printf(" gz = %f  mG\n\r", mpu9250->mz);

			tempCount = mpu9250->readTempData();  // Read the adc values
			temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
			trace_printf(" temperature = %f  C\n\r", temperature);

			trace_printf("q0 = %f\n\r", mpu9250->q[0]);
			trace_printf("q1 = %f\n\r", mpu9250->q[1]);
			trace_printf("q2 = %f\n\r", mpu9250->q[2]);
			trace_printf("q3 = %f\n\r", mpu9250->q[3]);

			//lcd.clear();
			//lcd.printString("MPU9250", 0, 0);
			//lcd.printString("x   y   z", 0, 1);
			//lcd.setXYAddress(0, 2); lcd.printChar((char)(1000*ax));
			//lcd.setXYAddress(20, 2); lcd.printChar((char)(1000*ay));
			//lcd.setXYAddress(40, 2); lcd.printChar((char)(1000*az)); lcd.printString("mg", 66, 2);

			// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
			// In this coordinate system, the positive z-axis is down toward Earth.
			// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
			// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
			// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
			// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
			// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
			// applied in the correct order which for this configuration is yaw, pitch, and then roll.
			// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
			yaw   = atan2(2.0f * (mpu9250->q[1] * mpu9250->q[2] + mpu9250->q[0] * mpu9250->q[3]), mpu9250->q[0] * mpu9250->q[0] + mpu9250->q[1] * mpu9250->q[1] - mpu9250->q[2] * mpu9250->q[2] - mpu9250->q[3] * mpu9250->q[3]);
			pitch = -asin(2.0f * (mpu9250->q[1] * mpu9250->q[3] - mpu9250->q[0] * mpu9250->q[2]));
			roll  = atan2(2.0f * (mpu9250->q[0] * mpu9250->q[1] + mpu9250->q[2] * mpu9250->q[3]), mpu9250->q[0] * mpu9250->q[0] - mpu9250->q[1] * mpu9250->q[1] - mpu9250->q[2] * mpu9250->q[2] + mpu9250->q[3] * mpu9250->q[3]);
			pitch *= 180.0f / PI;
			yaw   *= 180.0f / PI;
			yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
			roll  *= 180.0f / PI;

			trace_printf("Yaw, Pitch, Roll: %f %f %f\n\r", yaw, pitch, roll);
			trace_printf("average rate = %f\n\r", (float) sumCount/sum);

			STOPWATCH_START;
			count = (int)getMs(m_nStart);
			sum = 0;
			sumCount = 0;
		}
#endif
	}
#endif
}
#endif
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

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
}
