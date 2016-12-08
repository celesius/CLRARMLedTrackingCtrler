/*
 * CLRMPU6050DMP.cpp
 *
 *  Created on: 2016年8月22日
 *      Author: clover
 */

#include "CLRMPU6050DMP.h"
//#include "MPU6050.h"

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//中断初始化相关
//INT RCC
void RCC_INIT(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
}
//INT GPIO
void GPIO_INIT(){
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO_Init(GPIOA, &GPIO_InitStructure);

	//将GPIO管脚与外部中断线连接
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource11);
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
}
//INT EXTI
void EXTI_INIT(){
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_ClearITPendingBit(EXTI_Line12);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	//EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
//INT NVIC
void NVIC_INIT(){
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //PPP外部中断线
	//NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//这个channel要与io号相匹配,比如pin0 要用exti0; //PPP外部中断线
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void init_IRQ(){
	RCC_INIT();
	GPIO_INIT();
	NVIC_INIT();
	EXTI_INIT();
}

CLRMPU6050DMP::CLRMPU6050DMP() {
	//serialString = {0};
	mpu = new MPU6050();
	for(int i=0;i<4;i++){
		getQ[i] = 0;
	}
	setup();
	// TODO Auto-generated constructor stub

}

CLRMPU6050DMP::~CLRMPU6050DMP() {
	// TODO Auto-generated destructor stub
}

void CLRMPU6050DMP::setup() {
	// join I2C bus (I2Cdev library doesn't do this automatically)
	/*
		#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	        Wire.begin();
	        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	        Fastwire::setup(400, true);
	    #endif
	 */
	// initialize serial communication
	// (115200 chosen because it is required for Teapot Demo output, but it's
	// really up to you depending on your project)

	//Serial.begin(115200); //jiangbo
	//while (!Serial); //等待串口 wait for Leonardo enumeration, others continue immediately

	// NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
	// Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
	// the baud timing being too misaligned with processor ticks. You must use
	// 38400 or slower in these cases, or use some kind of external separate
	// crystal solution for the UART timer.

	// initialize device
	//Serial.println(F("Initializing I2C devices..."));
	//serialString = "Initializing I2C devices...\0";
	//COM_PutStr(COM1, serialString);
	trace_printf("Initializing I2C devices...\n");
	mpu->initialize();

	//中断初始化
	//pinMode(INTERRUPT_PIN, INPUT);
	// verify connection
	//Serial.println(F("Testing device connections..."));
	//serialString = "Testing device connections...";
	//COM_PutStr(COM1, serialString);
	trace_printf("Testing device connections...\n");

	//Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	if(mpu->testConnection()){
		trace_printf("MPU6050 ready!!!!!!!\n");
	}else{
		trace_printf("MPU6050 connection failed\n");
	}

	//    // wait for ready
	//    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
	//    while (Serial.available() && Serial.read()); // empty buffer
	//    while (!Serial.available());                 // wait for data
	//    while (Serial.available() && Serial.read()); // empty buffer again

	// load and configure the DMP
	//Serial.println(F("Initializing DMP..."));
	trace_printf("Initializing DMP...\n");

	devStatus = mpu->dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu->setXGyroOffset(220);
	mpu->setYGyroOffset(76);
	mpu->setZGyroOffset(-85);
	mpu->setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		trace_printf("Enabling DMP...\n");
		mpu->setDMPEnabled(true);

		// enable Arduino interrupt detection
		trace_printf("Enabling interrupt detection (Arduino external interrupt 0)...\n");
		//attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); //中断配置
		init_IRQ();
		mpuIntStatus = mpu->getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		trace_printf("DMP ready! Waiting for first interrupt...\n");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu->dmpGetFIFOPacketSize();
		trace_printf("packetSize  = %d\n",packetSize);

	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		trace_printf("DMP Initialization failed (code %d)\n",devStatus);
	}

	// configure LED for output
	//pinMode(LED_PIN, OUTPUT);
}

bool mpuInterrupt = false;
/*
void EXTI0_IRQHandler(void)
{
    //if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    //{
    //    EXTI_ClearITPendingBit(EXTI_Line11); //清除标志
    //}
    //trace_printf("key down !!!!\n");
    //EXTI_ClearITPendingBit(EXTI_Line0); //清除标志
	uint8_t	putS[60] = "jiangbo\r\n";
    if (EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        trace_printf("key down !!!!\n");
        //mpuInterrupt = true;
		//COM_PutStr(COM1, putS);
        EXTI_ClearITPendingBit(EXTI_Line0); //清除标志
    }
}
 */

#define OUTPUT_READABLE_QUATERNION

uint8_t	putS[30] = "FIFO overflow\r\n";
bool CLRMPU6050DMP::loop(float *q) {
	// if programming failed, don't try to do anything
	if (!dmpReady) {
		trace_printf("return \n");
		return false;
	}

	// wait for MPU interrupt or extra packet(s) available
	//while (!mpuInterrupt && fifoCount < packetSize) {
	// other program behavior stuff here
	// .
	// .
	// .
	// if you are really paranoid you can frequently test in between other
	// stuff to see if mpuInterrupt is true, and if so, "break;" from the
	// while() loop to immediately process the MPU data
	// .
	// .
	// .
	//}
	//trace_printf("loop\n");
	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu->getIntStatus();
	// get current FIFO count
	fifoCount = mpu->getFIFOCount();
	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu->resetFIFO();
		//trace_printf("FIFO overflow!  %d\n", fifoCount);
		return false;
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) {
			fifoCount = mpu->getFIFOCount();
			trace_printf("wait data\n");
		}

		// read a packet from FIFO
		mpu->getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
		// display quaternion values in easy matrix form: w x y z
		mpu->dmpGetQuaternion(getQ, fifoBuffer);
		for(int i=0;i<4;i++){
			q[i] = (float)getQ[i] / 16384.0f;
		}
		//            Serial.print("quat\t");
		//            Serial.print(q.w);
		//            Serial.print("\t");
		//            Serial.print(q.x);
		//            Serial.print("\t");
		//            Serial.print(q.y);
		//            Serial.print("\t");
		//            Serial.println(q.z);
		//trace_printf("q = %d %d %d %d\n",q[0], q[1], q[2], q[3]);
		/*
            Serial.print("quat# ");
            Serial.print(q.w, 2);
            Serial.print("# ");
            Serial.print(q.x, 2);
            Serial.print("# ");
            Serial.print(q.y, 2);
            Serial.print("# ");
            Serial.print(q.z, 2);
            Serial.println(F(""));*/
#endif

#ifdef OUTPUT_READABLE_EULER
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
		Serial.print("euler\t");
		Serial.print(euler[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(euler[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(euler[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Serial.print("ypr\t");
		Serial.print(ypr[0] * 180/M_PI);
		Serial.print("\t");
		Serial.print(ypr[1] * 180/M_PI);
		Serial.print("\t");
		Serial.println(ypr[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
		// display real acceleration, adjusted to remove gravity
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		Serial.print("areal\t");
		Serial.print(aaReal.x);
		Serial.print("\t");
		Serial.print(aaReal.y);
		Serial.print("\t");
		Serial.println(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
		// display initial world-frame acceleration, adjusted to remove gravity
		// and rotated based on known orientation from quaternion
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		Serial.print("aworld\t");
		Serial.print(aaWorld.x);
		Serial.print("\t");
		Serial.print(aaWorld.y);
		Serial.print("\t");
		Serial.println(aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
		// display quaternion values in InvenSense Teapot demo format:
		teapotPacket[2] = fifoBuffer[0];
		teapotPacket[3] = fifoBuffer[1];
		teapotPacket[4] = fifoBuffer[4];
		teapotPacket[5] = fifoBuffer[5];
		teapotPacket[6] = fifoBuffer[8];
		teapotPacket[7] = fifoBuffer[9];
		teapotPacket[8] = fifoBuffer[12];
		teapotPacket[9] = fifoBuffer[13];
		Serial.write(teapotPacket, 14);
		teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif

//blink LED to indicate activity
		//blinkState = !blinkState;
		//digitalWrite(LED_PIN, blinkState);
		return true;
	}
}
