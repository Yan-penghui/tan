
#ifndef __USART3_H
#define __USART3_H

#include "stdio.h"      
#include "stdarg.h"		 
#include "string.h"    

#define USART3_TXBUFF_SIZE   50   		   //定义串口3 发送缓冲区大小 1024字节
#define USART3_RXBUFF_SIZE   20            //定义串口3 接收缓冲区大小 20字节


extern unsigned int Usart3_RxCounter;          	//外部声明，其他文件可以调用该变量
extern u8 Usart3_RxBuff[USART3_RXBUFF_SIZE]; 	//外部声明，其他文件可以调用该变量



void usart3_init(unsigned int);       
void u3_printf(char*,...) ;          
void u3_TxData(unsigned char *data);

#endif


