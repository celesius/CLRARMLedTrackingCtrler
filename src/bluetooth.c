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
#include "bluetooth.h"
#include "string.h"
#include "./common/delay/delay.h"
#include "diag/Trace.h"

uint8_t AT_Resp[AT_RESP_SIZE];
uint8_t enterATMode();
void exitATMode();

/**
  * @brief  ����ģ���ʼ��
  * @param  None
  * @retval 0 - ��⵽����ģ�飻1 - δ��⵽����ģ��
	* @note   ��ʼ������ģ����ƶ˿ں�ͨ�Ŵ���
	*			PA5 -> BT_LED���ߵ�ƽ - ��Գɹ����͵�ƽ - δ���
	*			PA7 -> BT_KEY���ߵ�ƽ - AT��Ӧ����״̬���͵�ƽ - ���湤��״̬
  */
	


void BT_Init(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
	
	//����PA5Ϊ��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//��������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//����������Ϣ��ʼ���Ĵ���
	
		
	//PA7����Ϊ�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//����Ϊ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//����������Ϣ��ʼ���Ĵ���
	
	//PA7����Ϊ�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//����Ϊ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//����������Ϣ��ʼ���Ĵ���
	
	//��ʼ�����������ӵĴ���
	strcpy((char*)&COM_INFO[AT_COM]->name, "COM2");
	COM_INFO[AT_COM]->baudrate = 38400;
	COM_INFO[AT_COM]->onoff = 1;
	COM_INFO[AT_COM]->rxstatus = 0;
	COM_INFO[AT_COM]->rxnbr = 0;
	COM_INFO[AT_COM]->rxsize = AT_RESP_SIZE;
	COM_INFO[AT_COM]->rxbuf = (uint8_t*)AT_Resp;
	
	COM_Init(AT_COM, 38400);//38400
	BT_KEY = 0;
	
}
	
uint8_t BT_ATMode(void)
{
	uint8_t retry = 10;
	BT_KEY = 1;//����ATָ����Ӧģʽ
	delay_ms(10);
	//��������ģ�������Ƿ�����
	do
	{
		AT_SendCmd("AT\r\n");	//���Ͳ���ָ��
		trace_printf("send AT\n");
		//��ȡ����ȷ�ķ���ֵ

		//trace_printf("AT_GetResp\n");
		//int atGetResp = AT_GetResp();
		//trace_printf("getChar \n");
		//const char *getChar = (const char *)AT_Resp;

		if((AT_GetResp() == 0) && (strcmp("OK\r\n", (const char *)AT_Resp) == 0))
		//if((atGetResp == 0) && (strcmp("OK\r\n", getChar) == 0))
				break;
		else
			retry--;
	}while(retry);
	
	//BT_KEY = 0;//�˳�ATָ����Ӧģʽ
	
	if(retry == 0) return 1;//��ʼ��ʧ��
	
	return 0;//��ʼ���ɹ�
}

/**
  * @brief  ����ATָ��
  * @param  cmd : ָ���ַ���
  * @retval None
	* @note   ָ����"\r\n"�س����н�β
  */
void AT_SendCmd(const char *cmd)
{
	COM_PutStr(AT_COM, (uint8_t*)cmd);
}

/**
  * @brief  ��ȡAT��Ӧ
  * @param  resp : ��Ӧ�ַ�����ŵ�ַ
  * @retval 0 - �ɹ����AT��Ӧ��1 - ��ȡʧ��
	* @note
  */
uint8_t AT_GetResp(void)
{
  uint32_t timeout = AT_TIMEOUT;
	
	if(timeout > 0)//����ʱ���
	{
		while (timeout--)
		{
			if(COM_INFO[AT_COM]->rxstatus & 0x8000)//���յ���Ӧ����
				break;
		}
	}
	else if(timeout == 0)
	{
		while(1)
		{
			if(COM_INFO[AT_COM]->rxstatus & 0x8000)//���յ���Ӧ����
				break;
		}
	}
	
	if((COM_INFO[AT_COM]->rxstatus & 0x8000) == 0) return 1;//����ʧ�ܣ���ʱ��
	
	COM_INFO[AT_COM]->rxbuf[COM_INFO[AT_COM]->rxnbr++] = '\r';//��ӻس���
	COM_INFO[AT_COM]->rxbuf[COM_INFO[AT_COM]->rxnbr++] = '\n';//��ӻ��м�
	COM_INFO[AT_COM]->rxbuf[COM_INFO[AT_COM]->rxnbr++] = '\0';//����ַ���������
	
	COM_INFO[AT_COM]->rxstatus = 0;	//������ڽ���״̬��
	COM_INFO[AT_COM]->rxnbr = 0;		//������ڽ��ռ���ֵ
	
	return 0;//���ճɹ�
}



/**
  * @brief  ��������ģ��Ϊ��(Master)ģʽ
  * @param  role : 1 - ��ģʽ��0 - ��ģʽ��
  * @retval 0 - ���óɹ���1 - ����ʧ��
	* @note   
  */
uint8_t AT_SetRole(uint8_t role)
{
	int8_t error;
	
	BT_KEY = 1;//����ATָ����Ӧ״̬
	do{
	if(role == 1)
		AT_SendCmd("AT+ROLE=1\r\n");//����Ϊ��ģʽ
	else if(role == 0)
		AT_SendCmd("AT+ROLE=0\r\n");//����Ϊ��ģʽ
	
	error = AT_GetResp();					//��ȡAT��Ӧ
	
	if(error == 0)
	{
		error = strcmp("OK\r\n", (const char*)AT_Resp);//ȷ���Ƿ����óɹ�
		if(!error) { return 0;}//����ʧ��
//		if(error) {BT_KEY = 0; return 1;}//����ʧ��

//		AT_SendCmd("AT+RESET\r\n");//��λģ��
//		error = strcmp("OK\r\n", (const char*)AT_Resp);//ȷ���Ƿ����óɹ�
//		if(error) {BT_KEY = 0; return 1;}//����ʧ��
	}
//	else {BT_KEY = 0; return 1;		OLED_ShowString(0,32, "BT_KEY=0");
//		OLED_Refresh_Gram();		
//		OLED_ShowString(0,32, "             ");
//		OLED_Refresh_Gram();}//����ʧ��
//	
}while(1);
//	BT_KEY = 0;	//���ATָ����Ӧģʽ
//	return 0;		//���óɹ�
}

/**
  * @brief  ��ȡ��ǰ����ģ��Ĺ���ģʽ
  * @param  None
  * @retval 0 - ��ģʽ��1 - ��ģʽ; 0xFF - ��ȡʧ��
	* @note   
  */
uint8_t AT_GetRole(void)
{
	uint8_t error = 0xFF;
	
	// BT_KEY = 1;//����ATָ���Ӧģʽ
	do{
	AT_SendCmd("AT+ROLE?\r\n");	//��ѯ��ǰROLEģʽ
	error = AT_GetResp();				//��ȡAT��Ӧ
//	}while(!error);
//	do{
//		OLED_ShowString(0,32, (const char*)AT_Resp);
//		OLED_Refresh_Gram();		
//		OLED_ShowString(0,32, "                  ");
//		OLED_Refresh_Gram();
//	}while(1);
//	
//	if(error){BT_KEY = 0; return 0xFF;}//δ���յ���ȷAT��Ӧ
//	else
	{
//		error = strcmp("+ROLE=0\r\n", (const char*)AT_Resp);
		char* perror = strstr((const char*)AT_Resp,"+ROLE:0");
		if(perror !=NULL ){ return 0;}//��ģʽ
	
		perror = strstr((const char*)AT_Resp,"+ROLE:1");
		if(perror !=NULL ){ return 1;}//��ģʽ
		
//		perror = strstr((const char*)AT_Resp,"OK");
//		if(perror !=NULL ){		
//			OLED_ShowString(0,32, "OK is showing !");
//			OLED_Refresh_Gram();
//			return 3;
//		}//��ģʽ
		
//		error = strcmp("+ROLE=1\r\n", (const char*)AT_Resp);
//		if(error == 0){BT_KEY = 0; return 1;}//��ģʽ
	}
}while(1);
//		if(error){BT_KEY = 0; return 0xFF;}//δ���յ���ȷAT��Ӧ
//	else
//	{
//		error = strcmp("AT+ROLE=0\r\n", (const char*)AT_Resp);
//		if(error == 0){BT_KEY = 0; return 0;}//��ģʽ
//	
//		error = strcmp("AT+ROLE=1\r\n", (const char*)AT_Resp);
//		if(error == 0){BT_KEY = 0; return 1;}//��ģʽ
//	}

	
	//BT_KEY = 0;
//	return 0xFF;	//��ȡʧ��
}

uint8_t AT_ReSet(void)
{
	int8_t error;
	
	do{
		AT_SendCmd("AT+RESET\r\n");//��λģ��
		error = AT_GetResp();				//��ȡAT��Ӧ
		error = strcmp("OK\r\n", (const char*)AT_Resp);//ȷ���Ƿ����óɹ�
		if(!error) { return 0;}//���óɹ�
	}while(1);
	return 1;
	
}

void AT_Mode()
{
    uint8_t at_md = 0, t = 0, ret = 0, retry = 15;
    while(retry--)
    {
        if(BT_ATMode() == 0)
        {
            at_md = 1;
            break;
        }
        //OLED_ShowString(0,0,"BT ERROR!");
        //OLED_Refresh_Gram();
        trace_printf("BT ERROR!\n");

        if(BT_ATMode() == 0)
        {
            at_md = 1;
            break;
        }
        //OLED_ShowString(0,0,"         ");
        //OLED_Refresh_Gram();
        //delay_ms(5);
        if(t++>2)
        {
            t = 0;
            //LED = !LED;
        }
    }

    //OLED_ShowString(0,32,"CONNECT:        ");
    //OLED_Refresh_Gram();
    trace_printf("CONNECT %d\n", retry);
    if(at_md)
    {
        ret = AT_ReSet();
        if(ret)
        {
            //TimebaseCgf( 0,  127, 0);
            delay_ms(1000);
            //OLED_ShowString(0,48,"Reset fail!");
            trace_printf("Reset fail!");
        }
        else
        {
            //TimebaseCgf( 127, 0, 0);
            delay_ms(1000);
            //OLED_ShowString(0,48,"Reset OK.!");
            trace_printf("Reset OK.!");
        }
        //OLED_Refresh_Gram();
        at_md = 0;
    }
}

//JIANGBO add start
uint8_t enterATMode(){
	if(BT_ATMode() == 0)
		return 1;
	else
		trace_printf("enter AT err \n");
	return 0;
}

void exitATMode(){
	BT_KEY = 0;
}

void ATReadAndPrintReg( const char *cmd ){
	char data[10] = {0};
	sprintf(data,"AT+%s?\r\n",cmd);
	AT_SendCmd(data);
	int t = 0;
	int err = 1;
	for(t=0;t<20;t++){
		delay_ms(10);
		if((COM_INFO[AT_COM]->rxstatus & 0x8000))
		{
			trace_printf("%s\n",COM_INFO[AT_COM]->rxbuf);
			COM_INFO[AT_COM]->rxstatus = 0;	//清除接收状态字
			COM_INFO[AT_COM]->rxnbr = 0;		//清除数据接收计数值
			//COM_INFO[AT_COM]->rxbuf="";
			err = 0;
			break;
		}
	}
	if(err){
		trace_printf("read %s err\n",cmd);
	}
}

void ATWriteRegWithCmd(const char *reg, const char *cmd){
	char data[20] = {0};
	sprintf(data, "AT+%s=%s\r\n",reg,cmd);
	int t = 0;
	int err = 1;
	delay_ms(10);
	AT_SendCmd(data);
	for(t=0;t<20;t++){
	delay_ms(10);
		if((COM_INFO[AT_COM]->rxstatus & 0x8000)){
			trace_printf("%s\n",COM_INFO[AT_COM]->rxbuf);
			COM_INFO[AT_COM]->rxstatus = 0;	//清除接收状态字
			COM_INFO[AT_COM]->rxnbr = 0;		//清除数据接收计数值
			//COM_INFO[AT_COM]->rxbuf="";

			char *error = strstr((const char*)AT_Resp, "OK");//ȷ���Ƿ����óɹ�
			//if(error)
			if(error == NULL){
				trace_printf("ATWriteRegWithCmd %s read OK ERR read = %s\n",reg, AT_Resp);
			}else
				err = 0;
			break;
		}
	}
	if(err)
		trace_printf("ATWriteRegWithCmd %s err \n",reg);
}


void ATWriteRegWithData(const char *reg, const unsigned int aData){
	char data[20] = {0};
	sprintf(data, "AT+%s=%d\r\n",reg,aData);
	int t = 0;
	int err = 1;
	delay_ms(10);
	AT_SendCmd(data);
	for(t=0;t<20;t++){
	delay_ms(10);
		if((COM_INFO[AT_COM]->rxstatus & 0x8000)){
			trace_printf("%s\n",COM_INFO[AT_COM]->rxbuf);
			COM_INFO[AT_COM]->rxstatus = 0;	//清除接收状态字
			COM_INFO[AT_COM]->rxnbr = 0;		//清除数据接收计数值
			//COM_INFO[AT_COM]->rxbuf="";

			char *error = strstr((const char*)AT_Resp, "OK");//ȷ���Ƿ����óɹ�
			//if(error)
			if(error == NULL){
				trace_printf("ATWriteRegWithCmd %s read OK ERR read = %s\n",reg, AT_Resp);
			}else
				err = 0;
			break;
		}
	}
	if(err)
		trace_printf("ATWriteRegWithCmd %s err \n",reg);
}


void ATWriteReg(const char *reg){
	char data[20] = {0};
	sprintf(data, "AT+%s\r\n",reg);
	int t = 0;
	int err = 1;
	delay_ms(10);
	AT_SendCmd(data);
	for(t=0;t<20;t++){
	delay_ms(10);
		if((COM_INFO[AT_COM]->rxstatus & 0x8000)){
			trace_printf("%s\n",COM_INFO[AT_COM]->rxbuf);
			COM_INFO[AT_COM]->rxstatus = 0;	//清除接收状态字
			COM_INFO[AT_COM]->rxnbr = 0;		//清除数据接收计数值
			//COM_INFO[AT_COM]->rxbuf="";

			char *error = strstr((const char*)AT_Resp, "OK");//ȷ���Ƿ����óɹ�
			//if(error)
			if(error == NULL){
				trace_printf("ATWriteRegWithCmd %s read OK ERR read = %s\n",reg, AT_Resp);
			}else
				err = 0;
			break;
		}
	}
	if(err)
		trace_printf("ATWriteRegWithCmd %s err \n",reg);
}

void ATReadName(){
	if(enterATMode()){
		ATReadAndPrintReg("NAME");
	}
	exitATMode();
	//ATSendOneCmd("")
}

void ATReadVERSION(){
	if(enterATMode())
		ATReadAndPrintReg("VERSION");
	exitATMode();
}

void ATORGL() //reset to F
{
	ATWriteReg("ORGL");
}

void ATRename(const char *newName){
	ATWriteRegWithCmd("NAME", newName);
}

void ATSetup(){
	if(enterATMode()){
		//1
		//ATORGL();
		//2
		//ATWriteRegWithData("ROLE", 1);
		//ATWriteRegWithCmd("BIND","ec55,f9,f1640d");
		//3
		//ATWriteReg("RESET");
		//delay_ms(1000);
		ATWriteReg("INIT");
		//delay_ms(1000);
		//ATWriteRegWithCmd("PSWD","1234");
		//ATWriteRegWithCmd("PAIR","ec55,f9,f1640d,40");
		//delay_ms(40*1000);
		//ATWriteRegWithCmd("LINK","ec55,f9,f1640d");

		//ATWriteRegWithCmd("NAME", "IFVRControler");
		//ATWriteRegWithData("CLASS", 2508);
		/*
		//ATWriteReg("INIT");
		ATWriteRegWithCmd("NAME", "IFVRControler");
		ATWriteRegWithData("ROLE", 0);
		ATWriteReg("RESET");
		delay_ms(1000);
		ATWriteReg("INIT");

		//ATWriteRegWithCmd("PSWD","1234");
		//ATWriteRegWithData("CLASS", 0x1050C);
		ATWriteRegWithData("CMODE", 0);
		ATWriteRegWithCmd("BIND","ec55,f9,f1640d");
		//ATWriteRegWithCmd("PAIR","ec55,f9,f1640d,20");
		ATWriteRegWithCmd("FSAD","ec55,f9,f1640d");
		delay_ms(1000);
		ATWriteRegWithCmd("LINK","ec55,f9,f1640d");
		//delay_ms(100);

		 */
	}
	exitATMode();
	delay_ms(1000);
}

void ATCheck(){
	if(enterATMode()){
		//ATReadAndPrintReg("VERSION");
		ATReadAndPrintReg("NAME");
		ATReadAndPrintReg("ROLE");
		ATReadAndPrintReg("CLASS");
		//ATReadAndPrintReg("CMODE");
		//ATReadAndPrintReg("STATE");

		ATReadAndPrintReg("STATE");
		ATReadAndPrintReg("ADCN");
		ATReadAndPrintReg("BIND");
		ATReadAndPrintReg("MRAD");

	}
	exitATMode();
}

/*
 * 状态引脚,在蓝牙连接时为逻辑高,返回1
 * 状态引脚,在蓝牙断开时为逻辑低,返回0
 * */
int BTGetStatus(){
	 return ((int)BT_PORT->IDR & BT_STATE) ? 1:0;
}

//JIANGBO add end

/********************* (C) COPYRIGHT 2014 WWW.UCORTEX.COM **********END OF FILE**********/
