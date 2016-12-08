/*******************************************************************************
 * @name    : ��������
 * @author  : ������
 * @web     : WWW.UCORTEX.COM
 * @version : V1.0
 * @date    : 2014-04-03
 * @MDK     : KEIL MDK4.72a & KEL MDK 5.10
 * @brief   : ����ATָ�����
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
 * ������ģ��ATָ���ʵ��
 *
 *-----------------------------------------------------------------------------
 * @history
 * ----------------------------------------------------------------------------
 * ����ʱ�䣺2014-04-03    �����ˣ�������
 * �汾��¼��V1.0
 * �������ݣ��½�
 * ----------------------------------------------------------------------------
 *
 ******************************************************************************/

#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H
#include "stm32f10x.h"
#include "./common/bitband.h"	//ʹ��λ������ǰ���Ȱ�����ͷ�ļ�
#include "usart.h"

#define AT_COM			COM2
#define AT_TIMEOUT		(1000000)//����ȴ�AT��Ӧ��ʱ����
#define AT_RESP_SIZE	(128)//����AT��Ӧ������ݳߴ�

#define BT_STATE	GPIO_Pin_5  //GPIOin(GPIOA, 5)
#define BT_KEY		GPIOout(GPIOA, 4)
#define BT_PORT		GPIOA
//#define BT_KEY										GPIOout(GPIOA, 4)

extern uint8_t AT_Resp[AT_RESP_SIZE];

void BT_Init(void);
//uint8_t BT_ATMode(void);
void AT_SendCmd(const char *cmd);			//����ATָ��
uint8_t AT_GetResp(void);	//��ȡAT��Ӧ
uint8_t AT_SetRole(uint8_t role);		//��������ģʽ
uint8_t AT_GetRole(void);						//��ȡģ������ģʽ
uint8_t AT_ReSet(void);
void AT_Mode();
//JIANGBO add start
//void ATORGL(); //reset to F
void ATReadName();
void ATReadVERSION();
void ATRename(const char *newName);
int BTGetStatus();

void ATSetup();
void ATCheck();
//JIANGBO add end
#endif

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
