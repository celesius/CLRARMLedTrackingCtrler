/*
 * CLRTouchpad.cpp
 *
 *  Created on: 2016年11月28日
 *      Author: clover
 */

#include "CLRTouchpad.h"
#include "../common/delay/delay.h"
#include "diag/Trace.h"
#include <math.h>
#include <malloc.h>

#define Sensitivity1   (uint8_t)0x02
#define Sensitivity2   (uint8_t)0x03
#define Sensitivity3   (uint8_t)0x04
#define Sensitivity4   (uint8_t)0x05
#define Sensitivity5   (uint8_t)0x06
#define Sensitivity6   (uint8_t)0x07
#define CTRL1          (uint8_t)0x08
#define CTRL2          (uint8_t)0x09
#define Ref_rst1       (uint8_t)0x0A
#define Ref_rst2       (uint8_t)0x0B
#define Ch_hold1       (uint8_t)0x0C
#define Ch_hold2       (uint8_t)0x0D
#define Cal_hold1      (uint8_t)0x0E
#define Cal_hold2      (uint8_t)0x0F
#define Output1        (uint8_t)0x10
#define Output2        (uint8_t)0x11
#define Output3        (uint8_t)0x12

#define TSM_ADDR		(uint8_t)0xD0

CLRTouchpad::CLRTouchpad() {
	// TODO Auto-generated constructor stub
	//i2c_port = {GPIOB,9,8};
	m_i2c = new CLRI2CInterface(100);
	touchpad_pio_init();
	TSM_RESET = 0;
	touchpad_setup();

	m_read_data[0] = 0;
	m_read_data[1] = 0;
	m_read_data[2] = 0;
	for(int i = 0;i<(int)sizeof(m_mat_data);i++){
		m_mat_data[i] = 0;
	}
}

CLRTouchpad::~CLRTouchpad() {
	// TODO Auto-generated destructor stub
	delete m_i2c;
}

void CLRTouchpad::run_loop(){
	//delay_ms(500);
	m_i2c->readByte(TSM_ADDR, Output1, m_read_data);
	m_i2c->readByte(TSM_ADDR, Output2, m_read_data+1);
	m_i2c->readByte(TSM_ADDR, Output3, m_read_data+2);

	for(int i = 0;i<(int)sizeof(m_mat_data);i++){
		m_mat_data[i] = 0;
	}

	if((m_read_data[0] | m_read_data[1] | m_read_data[2]) != 0 ){
	trace_printf("|| ");
	for(int i=0;i<3;i++){
		trace_printf("0x%x ",m_read_data[i]);
		unsigned char base = 3;
		/*
		for(int j=0;j<4;j++){
			if((m_read_data[i] & (base<<(j*2))) != 0){

			}
		}
		*/
		if((m_read_data[i] & 0x03) != 0)
			m_mat_data[i*4 + 0] = 1;
		if((m_read_data[i] & 0x0C) != 0)
			m_mat_data[i*4 + 1] = 1;
		if((m_read_data[i] & 0x30) != 0)
			m_mat_data[i*4 + 2] = 1;
		if((m_read_data[i] & 0xC0) != 0)
			m_mat_data[i*4 + 3] = 1;
	}
	trace_printf("||");
	trace_printf("\n");

	for(int i=0;i<3;i++){
		trace_printf("|| ");
		for(int j=0;j<4;j++){
			trace_printf("%d ",m_mat_data[i*4 + j]);
		}
		trace_printf("||");
		trace_printf("\n");
	}
	mat_data_to_coordinates(m_mat_data);

	trace_printf("--------------------\n");


	}
}

bool CLRTouchpad::touchpad_pio_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0 );
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn  ; //PPP外部中断线
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line6 );
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger =  EXTI_Trigger_Rising_Falling;  //EXTI_Trigger_Rising;//EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6); //不能复用

	//reset and enable pin
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	//GPIO_InitTypeDef GPIO_InitStructure;
	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	///init pa4
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	//GPIO_InitTypeDef GPIO_InitStructure;
	// Configure pin in output push/pull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	TSM_RESET = 1;
	TSM_ENABLE= 0;
	TSM_POWER = 1;

	return true;
}

void CLRTouchpad::touchpad_setup()
{
	m_i2c->writeByte( TSM_ADDR, CTRL2, 0x0f);
	m_i2c->writeByte( TSM_ADDR, Sensitivity1, 0x55);
	m_i2c->writeByte( TSM_ADDR, Sensitivity2, 0x55);
	m_i2c->writeByte( TSM_ADDR, Sensitivity3, 0x55);
	m_i2c->writeByte( TSM_ADDR, Sensitivity4, 0x55);
	m_i2c->writeByte( TSM_ADDR, Sensitivity5, 0x55);
	m_i2c->writeByte( TSM_ADDR, Sensitivity6, 0x55);
	m_i2c->writeByte( TSM_ADDR, CTRL1, 0x22);
	m_i2c->writeByte( TSM_ADDR, Ref_rst1, 0x00);
	m_i2c->writeByte( TSM_ADDR, Ref_rst2, 0x00);
	m_i2c->writeByte( TSM_ADDR, Ch_hold1, 0x00);
	m_i2c->writeByte( TSM_ADDR, Ch_hold2, 0x00);
	m_i2c->writeByte( TSM_ADDR, Cal_hold1, 0x00);
	m_i2c->writeByte( TSM_ADDR, Cal_hold2, 0x00);
	m_i2c->writeByte( TSM_ADDR, CTRL2, 0x07);
/*
	Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(CTRL2)); // sends register address
	   Wire.write(byte(0x0F)); // sends register data
	   Wire.endTransmission(); // stop transmitting


	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Sensitivity1)); // sends register address
	   Wire.write(byte(0x55)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Sensitivity2)); // sends register address
	   Wire.write(byte(0x55)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Sensitivity3)); // sends register address
	   Wire.write(byte(0x55)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Sensitivity4)); // sends register address
	   Wire.write(byte(0x55)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Sensitivity5)); // sends register address
	   Wire.write(byte(0x55)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Sensitivity6)); // sends register address
	   Wire.write(byte(0x55)); // sends register data
	   Wire.endTransmission(); // stop transmitting


	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(CTRL1)); // sends register address
	   Wire.write(byte(0x22)); // sends register data
	   Wire.endTransmission(); // stop transmitting


	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Ref_rst1)); // sends register address
	   Wire.write(byte(0x00)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Ref_rst2)); // sends register address
	   Wire.write(byte(0x00)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Ch_Hold1)); // sends register address
	   Wire.write(byte(0x00)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Ch_Hold2)); // sends register address
	   Wire.write(byte(0x00)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Cal_Hold1)); // sends register address
	   Wire.write(byte(0x00)); // sends register data
	   Wire.endTransmission(); // stop transmitting

	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(Cal_Hold2)); // sends register address
	   Wire.write(byte(0x00)); // sends register data
	   Wire.endTransmission(); // stop transmitting


	   Wire.beginTransmission(TSM12_SLAVE_GND); // sned ic slave address
	   Wire.write(byte(CTRL2)); // sends register address
	   Wire.write(byte(0x07)); // sends register data
	   Wire.endTransmission(); // stop transmitting
*/
}

uint8_t CLRTouchpad::get_coordinate_num(uint8_t *mat_data)
{
	int cnt = 0;
	for(int y = 0;y<6;y++){
		if(mat_data[y+6] != 0)
			for(int x = 0;x<6;x++){
				if(mat_data[x] !=0 )
					cnt++;
			}
	}
	return cnt;
}

void CLRTouchpad::mat_data_to_coordinates(uint8_t *mat_data)//, pad_point *coordinates)
{
	uint8_t cnt_coordinates = get_coordinate_num(mat_data);
	pad_point *coordinates = (pad_point *)malloc(sizeof(pad_point)*cnt_coordinates);
	uint8_t realy_coord_cnt = 0;
	for(int y = 0;y<6;y++){
		if(mat_data[y+6] != 0){
			for(int x = 0;x<6;x++){
				if(mat_data[x] != 0){
					int x_c = 0; //5 6 是交叉的
					if(x == 5)
						x_c = 6;
					else if(x == 6)
						x_c = 5;
					else
						x_c = x;
					coordinates[realy_coord_cnt].x = x_c;
					coordinates[realy_coord_cnt].y = y;
					realy_coord_cnt++;
				}
			}
		}
	}

	if(cnt_coordinates == (realy_coord_cnt)){
		for(int i = 0;i<(realy_coord_cnt );i++){
			trace_printf("x = %d, y = %d \n",coordinates[i].x, coordinates[i].y);
		}
	}else{
		trace_printf("FULL coord error !\n");
		trace_printf(" cnt_coordinates = %d , realy_coord_cnt = %d \n", cnt_coordinates, (realy_coord_cnt));
	}


	free(coordinates);

}
