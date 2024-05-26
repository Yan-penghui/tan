
#ifndef __USART4_H
#define __USART4_H

#include "stdio.h"      
#include "stdarg.h"		 
#include "string.h"    

#define USART4_TXBUFF_SIZE   500   		   //���崮��4 ���ͻ�������С 100�ֽ�
#define USART4_RXBUFF_SIZE   500              //���崮��4 ���ջ�������С 100�ֽ�


extern unsigned int Usart4_RxCounter;          //�ⲿ�����������ļ����Ե��øñ���
extern char Usart4_RxBuff[USART4_RXBUFF_SIZE]; //�ⲿ�����������ļ����Ե��øñ���

void usart4_init(unsigned int);       
void u4_printf(char*,...) ;          
void u4_TxData(unsigned char *data);

#endif

