
#ifndef __WIFI_H
#define __WIFI_H

#include "uart4.h"	   	

#define RESET_IO(x)    GPIO_WriteBit(GPIOA, GPIO_Pin_4, (BitAction)x)  //PA4����WiFi�ĸ�λ

#define WiFi_printf       u4_printf           //����4���� WiFi
#define WiFi_RxCounter    Usart4_RxCounter    //����4���� WiFi
#define WiFi_RX_BUF       Usart4_RxBuff       //����4���� WiFi
#define WiFi_RXBUFF_SIZE  USART4_RXBUFF_SIZE  //����4���� WiFi

extern const char SSID[];                     //·�������ƣ���main.c������
extern const char PASS[];                     //·�������룬��main.c������

extern int ServerPort;
extern char ServerIP[128];                    //��ŷ�����IP��������

void wifi_reset_io_init(void);
char WiFi_SendCmd(char *cmd, int timeout);
char WiFi_Reset(int timeout);
char WiFi_JoinAP(int timeout);
char WiFi_Connect_Server(int timeout);
char WiFi_Smartconfig(int timeout);
char WiFi_WaitAP(int timeout);
char WiFi_Connect_IoTServer(void);


#endif


