
/*-------------------------------------------------*/
/*                                                 */
/*          	       串口3               	   */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f10x.h"  //包含需要的头文件
#include "usart3.h"     //包含需要的头文件
#include "Thermostat.h" 
#include "FreeRTOS.h"	 //FreeRTOS配置头文件
#include "queue.h"		 //队
unsigned int Usart3_RxCounter = 0;      //定义一个变量，记录串口3总共接收了多少字节的数据
u8 Usart3_RxBuff[USART3_RXBUFF_SIZE]; //定义一个数组，用于保存串口3接收到的数据    	
extern QueueHandle_t U3_xQueue;
//温控器


/*-------------------------------------------------*/
/*函数名：初始化串口3发送功能                      */
/*参  数：bound：波特率                            */
/*返回值：无                                       */
/*-------------------------------------------------*/
void usart3_init(unsigned int bound)
{  	 	
      //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3，GPIOB时钟
  
	//USART3_TX   GPIOB.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOC.10
	   
	//USART3_RX	  GPIOB.11初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO3.11 

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;//抢占优先级5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  

  
  
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART3, &USART_InitStructure); //初始化串口1
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART3, ENABLE);                    //使能串口3 



	USART_ClearFlag(USART3,USART_FLAG_TC);//清除USARTx的待处理标志位 
}



/*-------------------------------------------------*/
/*函数名：串口2接收中断                              */
/*参  数：data：数据                               */
/*返回值：无                                       */
/*-------------------------------------------------*/

//void USART3_IRQHandler(void)                	//串口3中断服务程序
//{
//	printf("USART3_IRQHandler=get! \r\n");
//	if(USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)
//	{
//		USART_ReceiveData(USART3);
//	}
//	if(USART_GetITStatus(USART3,USART_IT_RXNE)) //收到一个数据
//	{
//		modbus_LastRevIntervalTime=0;
//		modbus_Rvedata[modbus_RevLen] = USART_ReceiveData(USART3);	//读取接收到的数据

//		modbus_RevLen++;
//	}	 
//		
//	USART_ClearFlag(USART3,USART_IT_RXNE); //一定要清除接收中断	
//} 

/*-------------------------------------------------*/
/*函数名：串口3 printf函数                         */
/*参  数：char* fmt,...  格式化输出字符串和参数    */
/*返回值：无                                       */
/*-------------------------------------------------*/

__align(8) char USART3_TxBuff[USART3_TXBUFF_SIZE];  

void u3_printf(char* fmt, ...) 
{  
	unsigned int i, length;
	
	va_list ap;
	va_start(ap, fmt);
	vsprintf(USART3_TxBuff, fmt, ap);
	va_end(ap);	
	
	length=strlen((const char*)USART3_TxBuff);		
	while((USART3->SR&0X40) == 0);
	for(i = 0; i < length; i++)
	{			
		USART3->DR = USART3_TxBuff[i];
		while((USART3->SR&0X40) == 0);	
	}	
}

/*-------------------------------------------------*/
/*函数名：串口3发送缓冲区中的数据                  */
/*参  数：data：数据                               */
/*返回值：无                                       */
/*-------------------------------------------------*/
void u3_TxData(unsigned char *data)
{
	int	i;	
	while((USART3->SR&0X40) == 0);
	for(i = 1; i <= (data[0] * 256 + data[1]); i++)
	{			
		USART3->DR = data[i+1];
		while((USART3->SR&0X40) == 0);	
	}
}

/*---------------------------------------------------------------*/
/*函数名：void USART3_IRQHandler(void) 			      			 */
/*功  能：串口3中断处理函数										 */
/*		  1.与esp8266通信，已经连接服务器控制定时器4，未连接服务器 */
/*			不控制定时器4（通过事件标志组的位0 WIFI_CONECT判断）	 */
/*参  数：无                                       				 */
/*返回值：无                                     				 */
/*---------------------------------------------------------------*/
void USART3_IRQHandler(void)   
{   

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)         //如果USART_IT_RXNE标志置位，表示有数据到了，进入if分支	   
	{   
			Usart3_RxBuff[Usart3_RxCounter] = USART3->DR;//把接收到的数据保存到Usart4_RxBuff中
		//	if(Usart3_RxBuff[0]!=0x01)				     //如果包头不等于01	
		//	{    								
		//		Usart3_RxCounter = 0; 					 //丢弃数据
		//		printf("丢弃数据：%d  ",Usart3_RxBuff[0]);
		//	}
		//	else										 //else分支，表示果Usart1_RxCounter不等于0，不是接收的第一个数据
		//	{ 	
				Usart3_RxCounter++;         				 //每接收1个字节的数据，Usart3_RxCounter加1，表示接收的数据总量+1 				
		//	}	 
	}
	
	if(Usart3_RxCounter ==9)
	{
		//printf("Usart1_RxBuff[Usart1_RxCounter-1] ==0xFF");
		xQueueSendToBackFromISR( U3_xQueue,&Usart3_RxBuff,NULL);
	
	}
	
	if(Usart3_RxCounter >10)
	{
		Usart3_RxCounter = 0; 
	
	}
	
	
	
}
