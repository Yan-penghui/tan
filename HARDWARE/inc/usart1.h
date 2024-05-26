
#ifndef __USART1_H
#define __USART1_H

#include "stdio.h"     
#include "stdarg.h"			
#include "string.h"    

#define USART1_TXBUFF_SIZE   70   		   //定义串口1 发送缓冲区大小 1024字节
#define USART1_RXBUFF_SIZE   70              //定义串口1 接收缓冲区大小 1024字节



extern unsigned int Usart1_RxCounter;          //外部声明，其他文件可以调用该变量
//extern char Usart1_RxBuff[USART1_RXBUFF_SIZE]; //外部声明，其他文件可以调用该变量






void usart1_init(unsigned int);     	//串口1 初始化函数
#endif


