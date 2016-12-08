/*
 * CLRTouchpad.h
 *
 *  Created on: 2016年11月28日
 *      Author: clover
 */

#ifndef TSM12_CLRTOUCHPAD_H_
#define TSM12_CLRTOUCHPAD_H_

#include "../i2c/CLRI2CInterface.h"
#include "../common/bitband.h"

//pb5(pin57) tsm12 reset | High reset
//pb7() tsm12 i2c enable | low enable
//pb9 tsm12 i2c sda
//pb8 tsm12 i2c sclk
//pb6 tsm12 int
//tsm12 id_sel connect to GND i2c address is 0xd0

#define TSM_RESET	GPIOout(GPIOB, 5)
#define TSM_ENABLE	GPIOout(GPIOB, 7)
#define TSM_POWER	GPIOout(GPIOA, 4)

struct pad_point{
	uint8_t x;
	uint8_t y;
};


class CLRTouchpad {
	bool touchpad_pio_init();
	void touchpad_setup();
	void init_io();
	//void mat_data_to_coordinates(uint8_t *mat_data, pad_point *coordinates);
	void mat_data_to_coordinates(uint8_t *mat_data);
	uint8_t get_coordinate_num(uint8_t *mat_data);
	uint8_t m_read_data[3];
	uint8_t m_mat_data[12];
	CLRI2CInterface *m_i2c;

	//struct CLR_I2C_port i2c_port;
public:
	CLRTouchpad();
	virtual ~CLRTouchpad();
	void run_loop();

};

#endif /* TSM12_CLRTOUCHPAD_H_ */
