
/*-------------------------------------------------*/
/*                                                 */
/*          		  串口1                    	   */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f10x.h"  
#include "usart1.h"  
#include "FreeRTOS.h"	 //FreeRTOS配置头文件
#include "queue.h"		 //队列

unsigned int Usart1_RxCounter = 0;      //定义一个变量，记录串口1总共接收了多少字节的数据
char Usart1_RxBuff[USART1_RXBUFF_SIZE]; //定义一个数组，用于保存串口1接收到的数据  
extern QueueHandle_t U1_xQueue;
u8 input_data[70];
/*-------------------------------------------------*/
/*函数名：printf重定向      				           */
/*参  数：                                         */
/*返回值：                                         */
/*-------------------------------------------------*/  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40) == 0);//循环发送,直到发送完毕   
    USART1->DR = (u8)ch;      
	return ch;
}
#endif 


/*-------------------------------------------------*/
/*函数名：初始化串口1发送功能                       */
/*参  数：bound：波特率                            */
/*返回值：无                                       */
/*-------------------------------------------------*/
void usart1_init(unsigned int bound)
{  	 	
	GPIO_InitTypeDef GPIO_InitStructure;     //定义一个设置GPIO功能的变量
	USART_InitTypeDef USART_InitStructure;   //定义一个设置串口功能的变量
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //使能串口1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;              //准备设置PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO速率50M
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       //复用推挽输出，用于串口1的发送
	GPIO_Init(GPIOA, &GPIO_InitStructure);                 //设置PA9
   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;             //准备设置PA10 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入，用于串口1的接收
	GPIO_Init(GPIOA, &GPIO_InitStructure);                 //设置PA10
	
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;//抢占优先级5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	USART_InitStructure.USART_BaudRate = bound;                                    //波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //8个数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
                                                                                   //如果不使能接收模式
	USART_InitStructure.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx;				//收发模式	                                
	USART_Init(USART1, &USART_InitStructure);                                      //设置串口1	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);									//开启串口接受中断
	USART_Cmd(USART1, ENABLE);                            						 	//使能串口1
	USART_ClearFlag(USART1,USART_FLAG_TC); 											//清除USARTx的待处理标志位 
}

/*---------------------------------------------------------------*/
/*函数名：void USART1_IRQHandler(void) 			      			 */
/*功  能：串口1中断处理函数										 */
/*		  1.与串口屏通信											 */
/*参  数：无                                       				 */
/*返回值：无                                     				 */
/*---------------------------------------------------------------*/
void USART1_IRQHandler(void)   
{   
	u8 Usart1_RxBuff[140]; //定义一个数组，用于保存串口1接收到的数据 
	
	int i;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)        	 //如果USART_IT_RXNE标志置位，表示有数据到了，进入if分支	   
	{   
		Usart1_RxBuff[Usart1_RxCounter] = USART1->DR;				//把接收到的数据保存到Usart1_RxBuff中
		if(Usart1_RxBuff[0]!=0x24)				     				//如果包头不等于24	
		{    								
			Usart1_RxCounter = 0; 					 				//丢弃数据
			printf("丢弃数据：%2x  ",Usart1_RxBuff[0]);
		}
		else														 //else分支，表示果Usart1_RxCounter不等于0，不是接收的第一个数据
		{                        									    
			Usart1_RxCounter=Usart1_RxCounter+1;         			//每接收1个字节的数据，Usart1_RxCounter加1，表示接收的数据总量+1 		
		}	
	}

	if(Usart1_RxBuff[Usart1_RxCounter-1] ==0xFF)					//接收到包尾0xff
	{
		xQueueSendToFrontFromISR( U1_xQueue,&Usart1_RxBuff,NULL);	//写队列
				
	}
	
}
