/*
 * CLRADC.cpp
 *
 *  Created on: 2016年11月29日
 *      Author: clover
 */

#include "CLRADC.h"

/************ 用于定义ITM Viewer相关的ITM激励寄存器端口 ************************/
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

/*用于定义是否使用ITM Viewer*/
//#define DBG_ITM

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((u32)0x4001244C)

CLRADC::CLRADC() {
	// TODO Auto-generated constructor stub
	ADC_RCC_Configuration();
	ADC_GPIO_Configuration();
	dma_init();
	adc_init();
}

CLRADC::~CLRADC() {
	// TODO Auto-generated destructor stub
}

void  CLRADC::ADC_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
#if 0
  /* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif
  /* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void CLRADC::ADC_RCC_Configuration(void)
{
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Enable ADC1 and GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
}

void CLRADC::dma_init()
{

	/* DMA channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	m_DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	m_DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&m_ADC_ConvertedValue;
	m_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	m_DMA_InitStructure.DMA_BufferSize = 1;
	m_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	m_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	m_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	m_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	m_DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	m_DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	m_DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &m_DMA_InitStructure);
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

void CLRADC::adc_init()
{
	m_ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	m_ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	m_ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	m_ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	m_ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	m_ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &m_ADC_InitStructure);


	  /* ADC1 regular channel14 configuration */
	  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_55Cycles5);

	  /* Enable ADC1 DMA */
	  ADC_DMACmd(ADC1, ENABLE);

	  /* Enable ADC1 */
	  ADC_Cmd(ADC1, ENABLE);

	  /* Enable ADC1 reset calibaration register */
	  ADC_ResetCalibration(ADC1);
	  /* Check the end of ADC1 reset calibration register */
	  while(ADC_GetResetCalibrationStatus(ADC1));

	  /* Start ADC1 calibaration */
	  ADC_StartCalibration(ADC1);
	  /* Check the end of ADC1 calibration */
	  while(ADC_GetCalibrationStatus(ADC1));

	  /* Start ADC1 Software Conversion */
	  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

int CLRADC::get_adc_value(){
	int value = 0;
	value=ADC_GetConversionValue(ADC1);
	return value;
}

