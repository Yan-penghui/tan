
#ifndef __USART1_H
#define __USART1_H

#include "stdio.h"     
#include "stdarg.h"			
#include "string.h"    

#define USART1_TXBUFF_SIZE   70   		   //���崮��1 ���ͻ�������С 1024�ֽ�
#define USART1_RXBUFF_SIZE   70              //���崮��1 ���ջ�������С 1024�ֽ�



extern unsigned int Usart1_RxCounter;          //�ⲿ�����������ļ����Ե��øñ���
//extern char Usart1_RxBuff[USART1_RXBUFF_SIZE]; //�ⲿ�����������ļ����Ե��øñ���






void usart1_init(unsigned int);     	//����1 ��ʼ������
#endif


