/*******************************************************************************
 * @name    : I2C�ӿ�����ͷ�ļ�
 * @author  : ������
 * @web     : WWW.UCORTEX.COM
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : ʵ����GPIOģ��I2C�ӿڣ���I2C����������
 * ---------------------------------------------------------------------------- 
 * @copyright
 *
 * UCORTEX��Ȩ���У�Υ�߱ؾ�������Դ�������Ҳο���ּ�ڽ���ѧϰ�ͽ�ʡ����ʱ�䣬
 * ������Ϊ�ο����ļ����ݣ��������Ĳ�Ʒֱ�ӻ����ܵ��ƻ������漰���������⣬��
 * �߲��е��κ����Ρ�����ʹ�ù����з��ֵ����⣬���������WWW.UCORTEX.COM��վ��
 * �������ǣ����ǻ�ǳ���л�����������⼰ʱ�о����������ơ����̵İ汾���£�����
 * ���ر�֪ͨ���������е�WWW.UCORTEX.COM�������°汾��лл��
 * ��������������UCORTEX������һ�����͵�Ȩ����
 * ----------------------------------------------------------------------------
 * @description
 * ����������ʹ��ͨ��GPIOʵ��I2Cͨ�ţ����Ҵ�I2C�ٶ�����(1Kbps~400Kbps)���ڲ�ͬ
 * I2C���豸���ٶ�Ҫ��һ����ʱ�򣬿����ڷ���ǰ��������֮ƥ����ٶȣ����ʽ���
 * ���ٻָ�ԭ���Ĳ����ʡ�
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-04-06    �����ˣ�������
 * �汾��¼��V1.0
 * �������ݣ��½�
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/
#ifndef __I2C_H
#define __I2C_H
#include "stm32f10x.h"
#include "../common/bitband.h"

#define I2C_SPEED_1K		5000	//���ݴ������ٶ����ã����ﴦ�����ٶ���72MHz

//I2C�˿ڶ���
#define I2C_SCL    GPIOout(GPIOB, 10)	//SCL
#define I2C_SDA    GPIOout(GPIOB, 11)	//SDA	 
#define READ_SDA   GPIOin( GPIOB, 11)	//����SDA

//����PB11�������
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

typedef enum
{
	I2C_SUCCESS = 0,
	I2C_TIMEOUT,
	I2C_ERROR,
}I2C_StatusTypeDef;

extern uint32_t i2c_speed;	//I2C�����ٶ� = I2C_SPEED_1K / i2c_speed

/* ---------------------------����I2CЭ���д��ʱ����------------------------------*/
void I2C_Soft_Init(void);	//��ʼ��I2C��IO��
void I2C_Start(void);				//����I2C��ʼ�ź�
void I2C_Stop(void);				//����I2Cֹͣ�ź�
uint8_t I2C_Wait_ACK(void);	//I2C�ȴ�ACK�ź�
void I2C_ACK(void);					//I2C����ACK�ź�
void I2C_NACK(void);				//I2C������ACK�ź�
void I2C_Send_Byte(uint8_t data);		//I2C����һ���ֽ�
uint8_t I2C_Read_Byte(uint8_t ack);	//I2C��ȡһ���ֽ�
uint16_t I2C_SetSpeed(uint16_t speed);//����I2C�ٶ�(1Kbps~400Kbps,speed��λ��Kbps)

/* ---------------------------���²����Ƿ�װ�õ�I2C��д����--------------------------- */

//���嵽ĳһ������������ϸ�Ķ�������������I2C���ֵ�˵������ΪĳЩ������I2C�Ķ�д������
//��һЩ���죬����Ĵ��������ھ��������I2C�����У�������֤OK�ģ�
I2C_StatusTypeDef I2C_WriteOneByte(uint8_t DevAddr, uint8_t DataAddr, uint8_t Data);//��I2C���豸д��һ���ֽ�
I2C_StatusTypeDef I2C_WriteBurst(uint8_t DevAddr, uint8_t DataAddr, uint8_t* pData, uint32_t Num);//��I2C���豸����д��Num���ֽ�
I2C_StatusTypeDef I2C_ReadOneByte(uint8_t DevAddr, uint8_t DataAddr, uint8_t* Data);//��I2C���豸��ȡһ���ֽ�
I2C_StatusTypeDef I2C_ReadBurst(uint8_t DevAddr, uint8_t DataAddr, uint8_t* pData, uint32_t Num);//��I2C�豸������ȡNum���ֽ�
I2C_StatusTypeDef I2C_WriteBit(uint8_t DevAddr, uint8_t DataAddr, uint8_t Bitx, uint8_t BitSet);

#endif

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/

