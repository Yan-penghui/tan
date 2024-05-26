
/*-------------------------------------------------*/
/*                                                 */
/*          	 WIFI��ESP8266��Դ�ļ�             */
/*                                                 */
/*-------------------------------------------------*/

// Ӳ�����ӣ�
// PA2 RX
// PA3 TX
// PA4 ��λ

#include "stm32f10x.h"  //������Ҫ��ͷ�ļ�
#include "wifi.h"	    //������Ҫ��ͷ�ļ�
#include "delay.h"	    //������Ҫ��ͷ�ļ�
#include "usart1.h"	    //������Ҫ��ͷ�ļ�
#include "stdio.h"      //������Ҫ��ͷ�ļ�
#include "string.h"	    //������Ҫ��ͷ�ļ�
//#include <stdio.h>	

/*-------------------------------------------------*/
/*��������WiFi��������ָ��                         */
/*��  ����cmd��ָ��                                */
/*��  ����timeout����ʱʱ�䣨100ms�ı�����         */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char WiFi_SendCmd(char *cmd, int timeout)
{
	WiFi_RxCounter = 0;                           	//WiFi������������������                        
	memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);     	//���WiFi���ջ����� 
	WiFi_printf("%s\r\n", cmd);                  	//����ָ��
	
	
	while(timeout--)								//�ȴ���ʱʱ�䵽0
	{     
		delay_ms(100);								//��ʱ100ms 
		if(strstr(WiFi_RX_BUF, "OK"))              	//������յ�OK��ʾָ��ɹ�
		   break;  									//��������whileѭ��
		printf("%d ", timeout);                 	//����������ڵĳ�ʱʱ��
	}			
	printf("\r\n");                          			   
	if(timeout <= 0)return 1;                       //���timeout<=0��˵����ʱʱ�䵽�ˣ�Ҳû���յ�OK������1
	else return 0;		         					//��֮����ʾ��ȷ��˵���յ�OK��ͨ��break��������while
}
/*-------------------------------------------------*/
/*���������Ͽ�MQTT������                                 */
/*��  ����timeout����ʱʱ�䣨100ms�ı�����         */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char MQTT_Reset(char *cmd,int timeout)
{
	WiFi_RxCounter = 0;                           	//WiFi������������������                        
	memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);     	//���WiFi���ջ����� 
	WiFi_printf("%s\r\n", cmd);                  	//����ָ��
	
	
	while(timeout--)								//�ȴ���ʱʱ�䵽0
	{     
		delay_ms(100);								//��ʱ100ms 
		if(strstr(WiFi_RX_BUF, "OK")||strstr(WiFi_RX_BUF, "ERROR"))              	//������յ�OK��ʾָ��ɹ�
		   break;  									//��������whileѭ��
		printf("%d ", timeout);                 	//����������ڵĳ�ʱʱ��
	}			
	printf("\r\n");                          			   
	if(timeout <= 0)return 1;                       //���timeout<=0��˵����ʱʱ�䵽�ˣ�Ҳû���յ�OK������1
	else return 0;		         					//��֮����ʾ��ȷ��˵���յ�OK��ͨ��break��������while
}
/*-------------------------------------------------*/
/*��������WiFi����·����ָ��                       */
/*��  ����timeout����ʱʱ�䣨1s�ı�����            */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char WiFi_JoinAP(int timeout)
{		
	WiFi_RxCounter = 0;                                    //WiFi������������������                        
	memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);              //���WiFi���ջ����� 
	WiFi_printf("AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASS); //����ָ��	
	while(timeout--)									   //�ȴ���ʱʱ�䵽0
	{                                   
		delay_ms(1000);                             	   //��ʱ1s
		if(strstr(WiFi_RX_BUF, "WIFI GOT IP\r\n\r\nOK"))   //������յ�WIFI GOT IP��ʾ�ɹ�
			break;       						           //��������whileѭ��
		printf("%d ", timeout);                            //����������ڵĳ�ʱʱ��
	}
	printf("\r\n");                             	       //���������Ϣ
	if(timeout <= 0)return 1;                              //���timeout<=0��˵����ʱʱ�䵽�ˣ�Ҳû���յ�WIFI GOT IP������1
	return 0;                                              //��ȷ������0
}


/*-------------------------------------------------*/
/*������������MQTT�û�����				            */
/*��  ����timeout�� ��ʱʱ�䣨100ms�ı�����        */
/*����ֵ��0����ȷ  ����������                      */
/*-------------------------------------------------*/
char MQTT_Cfg_Server(int timeout)
{	
	WiFi_RxCounter = 0;                               //WiFi������������������                        
	memset(WiFi_RX_BUF,0,WiFi_RXBUFF_SIZE);           //���WiFi���ջ����� 
	WiFi_printf("AT+MQTTUSERCFG=0,2,\"TAN\",\"yph\",\"1032979127\",0,0,\"\"\r\n"); //��������ָ��
	
	while(timeout--)								  //�ȴ���ʱ���
	{                           
		delay_ms(100);                             	  //��ʱ100ms	
		if(strstr(WiFi_RX_BUF, "OK"))            //������ܵ�OK��ʾ���óɹ�
			break;                                    //����whileѭ��
		if(strstr(WiFi_RX_BUF, "ERROR"))             //������ܵ�ERROR��ʾ����ʧ��
		{WiFi_printf("AT+MQTTUSERCFG=ERROR"); //��������ָ��
			return 1; }                                //����ʧ�ܷ���1
		printf("%d ", timeout);                       //����������ڵĳ�ʱʱ��  
	}
	printf("\r\n");                                   
	if(timeout <= 0)return 2;                         //��ʱ���󣬷���2
	
	return 0;	                                      //�ɹ�����0	
}



/*-------------------------------------------------*/
/*������������MQTT��������������͸��ģʽ            */
/*��  ����timeout�� ��ʱʱ�䣨100ms�ı�����        */
/*����ֵ��0����ȷ  ����������                      */
/*-------------------------------------------------*/
char MQTT_Connect_Server(int timeout)
{	
	WiFi_RxCounter = 0;                               //WiFi������������������                        
	memset(WiFi_RX_BUF,0,WiFi_RXBUFF_SIZE);           //���WiFi���ջ����� 
 
	WiFi_printf("AT+MQTTCONN=0,\"%s\",%d,1\r\n", ServerIP, ServerPort);//�������ӷ�����ָ��
	
	while(timeout--)								  //�ȴ���ʱ���
	{                           
		delay_ms(100);                             	  //��ʱ100ms	
		if(strstr(WiFi_RX_BUF, "MQTTCONNECTED"))      //������ܵ�MQTTCONNECTED��ʾ���ӳɹ�
			break;                                    //����whileѭ��
		if(strstr(WiFi_RX_BUF, "ERROR"))             //������ܵ�CLOSED��ʾ������δ����
		{
			printf("AT+MQTTCONN=ERROR\r\n");
			return 1;                                 //������δ��������1
	
		}

		printf("%d ", timeout);                       //����������ڵĳ�ʱʱ��  
	}
	printf("\r\n");                                   
	if(timeout <= 0)return 3;                         //��ʱ���󣬷���3

	return 0;	                                      //�ɹ�����0	
}
/*-------------------------------------------------*/
/*��������WiFi_Smartconfig                         */
/*��  ����timeout����ʱʱ�䣨1s�ı�����            */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char WiFi_Smartconfig(int timeout)
{
	
	WiFi_RxCounter = 0;                           		//WiFi������������������                        
	memset(WiFi_RX_BUF,0,WiFi_RXBUFF_SIZE);     		//���WiFi���ջ�����     
	while(timeout--)									//�ȴ���ʱʱ�䵽0
	{                           		
		delay_ms(1000);                         		//��ʱ1s
		if(strstr(WiFi_RX_BUF, "connected"))    	 	//������ڽ��ܵ�connected��ʾ�ɹ�
			break;                                  	//����whileѭ��  
		printf("%d ", timeout);                 		//����������ڵĳ�ʱʱ��  
	}	
	printf("\r\n");                          			
	if(timeout <= 0)return 1;                     		//��ʱ���󣬷���1
	return 0;                                   		//��ȷ����0
}
/*-------------------------------------------------*/
/*���������ȴ�����·����                           */
/*��  ����timeout����ʱʱ�䣨1s�ı�����            */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char WiFi_WaitAP(int timeout)
{		
	while(timeout--){                               //�ȴ���ʱʱ�䵽0
		delay_ms(1000);                             //��ʱ1s
		if(strstr(WiFi_RX_BUF, "WIFI GOT IP"))      //������յ�WIFI GOT IP��ʾ�ɹ�
			break;       						 
		printf("%d ", timeout);                     //����������ڵĳ�ʱʱ��
	}
	printf("\r\n");                             	//���������Ϣ
	if(timeout <= 0)return 1;                       //���timeout<=0��˵����ʱʱ�䵽�ˣ�Ҳû���յ�WIFI GOT IP������1
	return 0;                                       //��ȷ������0
}
/*-------------------------------------------------*/
/*��������WiFi���ӷ�����                           */
/*��  ������                                       */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
char WiFi_Connect_IoTServer(void)
{	
	
	
	printf("׼������STAģʽ\r\n");                
	if(WiFi_SendCmd("AT+CWMODE=1",20))			  //����STAģʽ��100ms��ʱ��λ���ܼ�2s��ʱʱ��
	{             
		printf("����STAģʽʧ�ܣ�׼������\r\n");  //���ط�0ֵ������if
		return 2;                                 //����2
	}else printf("����STAģʽ�ɹ�\r\n");          
	                            
//	printf("׼��ȡ���Զ�����\r\n");            	  
//	if(WiFi_SendCmd("AT+CWAUTOCONN=0",50))		  //ȡ���Զ����ӣ�100ms��ʱ��λ���ܼ�5s��ʱʱ��
//	{       
//		printf("ȡ���Զ�����ʧ�ܣ�׼������\r\n"); //���ط�0ֵ������if
//		return 3;                                 //����3
//	}else printf("ȡ���Զ����ӳɹ�\r\n");         
			
	printf("׼������·����\r\n");                 	
	if(WiFi_JoinAP(10))							  //����·����,1s��ʱ��λ���ܼ�10s��ʱʱ��
	{                          
		printf("����·����ʧ�ܣ�׼������\r\n");   //���ط�0ֵ������if
		return 4;                                 //����4	
	}else printf("����·�����ɹ�\r\n");       		

//	printf("׼������͸��\r\n");                    
//	if(WiFi_SendCmd("AT+CIPMODE=1",50)) 		  //����͸����100ms��ʱ��λ���ܼ�5s��ʱʱ��
//	{           
//		printf("����͸��ʧ�ܣ�׼������\r\n");     //���ط�0ֵ������if
//		return 8;                                 //����8
//	}else printf("����͸���ɹ�\r\n");              
	
//	printf("׼���رն�·����\r\n");               
//	if(WiFi_SendCmd("AT+CIPMUX=0",50)) 		      //�رն�·���ӣ�100ms��ʱ��λ���ܼ�5s��ʱʱ��
//	{            
//		printf("�رն�·����ʧ�ܣ�׼������\r\n"); //���ط�0ֵ������if
//		return 9;                                 //����9
//	}else printf("�رն�·���ӳɹ�\r\n");  

	printf("MQTT�������Ͽ�����\r\n");                   
	if(MQTT_Reset("AT+MQTTCLEAN=0",100))							  //��λ��100ms��ʱ��λ���ܼ�5s��ʱʱ��
	{                             
		printf("�Ͽ�MQTT������ʧ�ܣ�׼������\r\n");	      //���ط�0ֵ������if
		return 1;                                 //����1
	}else printf("�Ͽ�MQTT�������ɹ�\r\n");                 


	
	printf("׼������MQTT�û�����\r\n"); 
	
	if(MQTT_Cfg_Server(100))      			  //���ӷ�������100ms��ʱ��λ���ܼ�10s��ʱʱ��
	{            
		printf("����MQTT�û�����ʧ�ܣ�׼������\r\n");   //���ط�0ֵ������if
		return 9;                                //����10
	}else printf("����MQTT�û����Գɹ�\r\n");     
	
	
	printf("׼�����ӷ�����\r\n");                 
	if(MQTT_Connect_Server(150))      			  //���ӷ�������100ms��ʱ��λ���ܼ�10s��ʱʱ��
	{            
		printf("���ӷ�����ʧ�ܣ�׼������\r\n");   //���ط�0ֵ������if
		return 10;                                //����10
	}else printf("���ӷ������ɹ�\r\n");           
	return 0;                                     //��ȷ����0
}
	

