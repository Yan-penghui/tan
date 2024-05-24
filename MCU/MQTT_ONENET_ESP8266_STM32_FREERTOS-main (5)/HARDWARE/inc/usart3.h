
#ifndef __USART3_H
#define __USART3_H

#include "stdio.h"      
#include "stdarg.h"		 
#include "string.h"    

#define USART3_TXBUFF_SIZE   50   		   //���崮��3 ���ͻ�������С 1024�ֽ�
#define USART3_RXBUFF_SIZE   20            //���崮��3 ���ջ�������С 20�ֽ�


extern unsigned int Usart3_RxCounter;          	//�ⲿ�����������ļ����Ե��øñ���
extern u8 Usart3_RxBuff[USART3_RXBUFF_SIZE]; 	//�ⲿ�����������ļ����Ե��øñ���



void usart3_init(unsigned int);       
void u3_printf(char*,...) ;          
void u3_TxData(unsigned char *data);

#endif


