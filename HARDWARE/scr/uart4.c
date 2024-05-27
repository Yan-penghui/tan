
/*-------------------------------------------------*/
/*                                                 */
/*          	       串口4                  	   */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f10x.h"     //包含需要的头文件
#include "FreeRTOS.h"	   //FreeRTOS使用		  
#include "task.h"
#include "event_groups.h"
#include "stm32f10x_it.h"  //包含需要的头文件
#include "usart1.h"        //包含需要的头文件
#include "usart2.h"        //包含需要的头文件
#include "uart4.h"        //包含需要的头文件
#include "timer3.h"        //包含需要的头文件
#include "mqtt.h"          //包含需要的头文件
#include "dht11.h"         //包含需要的头文件                  

extern void xPortSysTickHandler(void);
extern TaskHandle_t WIFI_Task_Handler;
extern  EventGroupHandle_t Event_Handle;
extern const int PING_MODE;

unsigned int Usart4_RxCounter = 0;      //定义一个变量，记录串口4总共接收了多少字节的数据
char Usart4_RxBuff[USART4_RXBUFF_SIZE]; //定义一个数组，用于保存串口4接收到的数据
/*-------------------------------------------------*/
/*函数名：初始化串口4发送功能                      */
/*参  数：bound：波特率                            */
/*返回值：无                                       */
/*-------------------------------------------------*/
void usart4_init(unsigned int bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;     //定义一个设置GPIO功能的变量
  USART_InitTypeDef USART_InitStructure;   //定义一个设置串口功能的变量
  NVIC_InitTypeDef NVIC_InitStructure;     //如果使能接收功能，定义一个设置中断的变量

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //使能串口4时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //使能GPIOC时钟
  USART_DeInit(UART4);                                  //串口4寄存器重新设置为默认值

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;              //准备设置PC10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO速率50M
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       //复用推挽输出，用于串口4的发送
  GPIO_Init(GPIOC, &GPIO_InitStructure);                 //设置PC10

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;              //准备设置PC11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;			//上拉输入 ，用于串口4的接收
  GPIO_Init(GPIOC, &GPIO_InitStructure);                 //设置PC11

  USART_InitStructure.USART_BaudRate = bound;                                    //波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //8个数据位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //1个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	               //收发模式

  USART_Init(UART4, &USART_InitStructure);                                      //设置串口4

  USART_ClearFlag(UART4, USART_FLAG_RXNE);	              //清除接收标志位
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);            //开启接收中断

  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;         //设置串口4中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //中断通道使能
  NVIC_Init(&NVIC_InitStructure);	                          //设置串口2中断

  USART_Cmd(UART4, ENABLE);                                //使能串口4
}

/*-------------------------------------------------*/
/*函数名：串口4 printf函数                         */
/*参  数：char* fmt,...  格式化输出字符串和参数    */
/*返回值：无                                       */
/*-------------------------------------------------*/

__align(8) char USART4_TxBuff[USART4_TXBUFF_SIZE];

void u4_printf(char* fmt, ...)
{
  unsigned int i, length;

  va_list ap;
  va_start(ap, fmt);
  vsprintf(USART4_TxBuff, fmt, ap);
  va_end(ap);

  length=strlen((const char*)USART4_TxBuff);
  while((UART4->SR&0X40) == 0);
  for(i = 0; i < length; i++)
    {
      UART4->DR = USART4_TxBuff[i];
      while((UART4->SR&0X40) == 0);
    }
}

/*-------------------------------------------------*/
/*函数名：串口4发送缓冲区中的数据                  */
/*参  数：data：数据                               */
/*返回值：无                                       */
/*-------------------------------------------------*/
void u4_TxData(unsigned char *data)
{
  int	i;
  while((UART4->SR&0X40) == 0);
  for(i = 1; i <= (data[0] * 256 + data[1]); i++)
    {
      UART4->DR = data[i+1];
      while((UART4->SR&0X40) == 0);
    }
}

/*---------------------------------------------------------------*/
/*函数名：void USART4_IRQHandler(void) 			      			 */
/*功  能：串口4中断处理函数										 */
/*		  1.与esp8266通信，已经连接服务器控制定时器4，未连接服务器 */
/*			不控制定时器4（通过事件标志组的位0 WIFI_CONECT判断）	 */
/*参  数：无                                       				 */
/*返回值：无                                     				 */
/*---------------------------------------------------------------*/
void UART4_IRQHandler(void)
{

  if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)         //如果USART_IT_RXNE标志置位，表示有数据到了，进入if分支
    {

      if ((xEventGroupGetBitsFromISR(Event_Handle) & 0x01) == 0)//获取事件标志组数据，等于0说明未连接服务器，不开启定时器4（MQTT接收数据处理）定时器
        {
          if(UART4->DR)                                        //处于指令配置状态时，非零值才保存到缓冲区
            {
              Usart4_RxBuff[Usart4_RxCounter] = UART4->DR;	  //保存到缓冲区
              Usart4_RxCounter++; 						      //每接收1个字节的数据，Usart4_RxCounter加1，表示接收的数据总量+1
            }
        }
      else
        {
          Usart4_RxBuff[Usart4_RxCounter] = UART4->DR;//把接收到的数据保存到Usart4_RxBuff中

          if(Usart4_RxCounter == 0)				     //如果Usart4_RxCounter等于0，表示是接收的第1个数据，进入if分支
            {
              TIM_Cmd(TIM4, ENABLE); 					 //使能定时器4
            }
          else										 //else分支，表示果Usart4_RxCounter不等于0，不是接收的第一个数据
            {
              TIM_SetCounter(TIM4, 0);  				 //置位定时器4
            }
          Usart4_RxCounter++;         				 //每接收1个字节的数据，Usart4_RxCounter加1，表示接收的数据总量+1
        }

    }

}

