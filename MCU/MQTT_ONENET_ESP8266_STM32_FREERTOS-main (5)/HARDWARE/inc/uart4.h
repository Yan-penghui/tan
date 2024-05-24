
#ifndef __USART4_H
#define __USART4_H

#include "stdio.h"      
#include "stdarg.h"		 
#include "string.h"    

#define USART4_TXBUFF_SIZE   500   		   //定义串口4 发送缓冲区大小 100字节
#define USART4_RXBUFF_SIZE   500              //定义串口4 接收缓冲区大小 100字节


extern unsigned int Usart4_RxCounter;          //外部声明，其他文件可以调用该变量
extern char Usart4_RxBuff[USART4_RXBUFF_SIZE]; //外部声明，其他文件可以调用该变量

void usart4_init(unsigned int);       
void u4_printf(char*,...) ;          
void u4_TxData(unsigned char *data);

#endif

