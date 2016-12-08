/*
 * CLRADC.h
 *
 *  Created on: 2016年11月29日
 *      Author: clover
 */

#ifndef ADC_CLRADC_H_
#define ADC_CLRADC_H_
#include "stm32f10x.h"

class CLRADC {
	void ADC_GPIO_Configuration();
	void ADC_RCC_Configuration();
	void dma_init();
	void adc_init();
	ADC_InitTypeDef m_ADC_InitStructure;
	DMA_InitTypeDef m_DMA_InitStructure;
	vu16 m_ADC_ConvertedValue;
	ErrorStatus m_HSEStartUpStatus;
public:
	CLRADC();
	virtual ~CLRADC();
	int get_adc_value();
};

#endif /* ADC_CLRADC_H_ */
