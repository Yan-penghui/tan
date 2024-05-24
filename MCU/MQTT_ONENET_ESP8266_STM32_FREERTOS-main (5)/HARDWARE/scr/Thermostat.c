
/*-------------------------------------------------*/
/*                                                 */
/*   		     宇电485温控器                      */
/*                                                 */
/*-------------------------------------------------*/

// 硬件连接：
// usart3转485

#include "stm32f10x.h"  //包含需要的头文件
#include "Thermostat.h"      //包含需要的头文件 
#include "delay.h"      //包含需要的头文件
#include "usart3.h"     //包含需要的头文件
#include "switch.h"
#include "stdio.h"      //包含需要的头文件
#include "string.h"	    //包含需要的头文件
#include "FreeRTOS.h"	 //FreeRTOS配置头文件
#include "queue.h"		 //队
#include "task.h"


extern QueueHandle_t U3_xQueue;

int PVTem = 0 ,SVTem = 0,RunTime = 0 ,Srun = 0,Pno=0, Step = 0, Output=0 , Value=0;

/*-----------------------------------------------------------*/
/*	函数名：Thermostat_init初始化 温控器                      */
/*	参  数：												 	 */
/*	通讯模式			AFC: 0 - 标准modbus						 */
/*	输入			lnp：6 - B热电偶                         */ 
/*	升温速率限制		SPr：50 - 度/分钟                        */ 
/*	状态控制			Srun: 0-运行 ，1-停止 ，2-保持hold		 */
/*-----------------------------------------------------------*/  
void Thermostat_init(void)	   
{    	
	Send_Thermostat('w',0x0b,6);			//lnp寄存器地址0x0b
	 
	Send_Thermostat('w',0x2a,50*10); 		//SPr寄存器地址0x2a
	
	//Send_Thermostat('w',0x50,50*10); 		//SP1寄存器地址0x50
		 
	Send_Thermostat('w',0x1b,1);			//1b-Srun： 0-run 1-stop 2-hold

	printf("温控器参数初始化成功\r\n");
}


/*-------------------------------------------------*/
/*函数名：串口发送字节                          	   */
/*参  数：Byte：数据                                */
/*参  数：usart：串口号						       */
/*返回值：						                   */
/*-------------------------------------------------*/

//发送数据
void Serial_SendByte(u8 Byte,int usart)
{	
	//发送数据的函数
	if(usart==1){
	USART_SendData(USART1,Byte);}
	else if(usart==3){
	USART_SendData(USART3,Byte);}
	
	
	//检查某个寄存器的中断标志位
	while (USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);
	/*
	在完成数据发送的时候,TXE置1;
	下次发送数据自动置0,所以不用手动清除中断标志位.
	
	TXE:发送数据寄存器:
	当TDR寄存器中的数据被硬件转移到移位寄存器的时候，该位被硬件置位。
	如果USART_CR1寄存器中的TXEIE为1，则产生中断。对USART DR的写操作，将该位清零。
	0:数据还没有被转移到移位寄存器
	1:数据已经被转移到移位寄存器。
	注意:单缓冲器传输中使用该位。
	*/
}

/*-------------------------------------------------*/
/*函数名：串口发送多个字节                          */
/*参  数：Array：数据组                             */
/*参  数：Length：字节数                            */
/*参  数：usart：串口号						       */
/*返回值：						                   */
/*-------------------------------------------------*/

void Serial_SendArray(u8 *Array, int Length,int usart){
	
	int i;
	//delay_us(800);
	for(i=0;i<Length;i++)
	{
		Serial_SendByte(Array[i],usart);
		//delay_us(500);               //波特率，设定发送时间间隔
	}
}
 

/*-------------------------------------------------*/
/*函数名：温控器发送设置指令                         */
/*参  数：operate：W/R  写/读                       */
/*参  数：name：寄存器地址        					 */
/*参  数：w_data：写入数据值       					  */
/*返回值：0：正确   其他：错误                     */
/*-------------------------------------------------*/
int Send_Thermostat(char operate,u8 name,u16 w_data)
{
	u8 send_data[8];
	u8 re_data[30];
	u16 sendCRC=0;
	int i=5;		//超时计数
	
	send_data[0] = 0x81;               		  	//仪表地址80+1
	send_data[1] = 0x81;
	if(operate=='w')
	{
		send_data[2] = 0x43;            		 	//写指令43
		send_data[4] = w_data;               		//写入数据(高位）
		send_data[5] = w_data>>8;               	//写入数据(低位）
	}
	if(operate=='r')
	{
		send_data[2] = 0x52;					//读指令52
		send_data[4] = 0x00;					//读数据长度（高位都是0x00）
		send_data[5] = 0x00;					//读数据长度（统一长读为1）
	}
	
	send_data[3] = name;

	//校验和
	if(operate=='r'){sendCRC = name*256+82+1;}
	if(operate=='w'){sendCRC = name*256+67+w_data +1;}
	send_data[7] = sendCRC>>8;					//高8位
    send_data[6] = sendCRC;						//低8位
	
	//发送指令数据，发送失败重试次数：5
	
	while(i)
	{
		
		RS485=1;													//使能发送1使能接收0
		Thermostat_RxCounter = 0;                           		//Thermostat接收数据量变量清零                        
		memset(Thermostat_RX_BUF, 0, Thermostat_RXBUFF_SIZE);     	//清空Thermostat接收缓冲区 
		
		Serial_SendArray(send_data, 9,3);							//发送指令数据Serial_SendArray(data, 字计数量,串口)
		
		RS485=0;													//使能发送1使能接收0
	
	
		//50ms内接收到正确返回值
		if (pdPASS == xQueueReceive( U3_xQueue,&re_data,200 ))
		{ 
			u16 modbusCRC,myCRC;
			delay_ms(100);
			//计算接收数据的校验和
			modbusCRC =	((Thermostat_RX_BUF[0] + Thermostat_RX_BUF[1]*256
				+	Thermostat_RX_BUF[2] + Thermostat_RX_BUF[3]*256
				+	Thermostat_RX_BUF[4] + Thermostat_RX_BUF[5]*256
				+	Thermostat_RX_BUF[6] + Thermostat_RX_BUF[7]*256)+1)&0xffff;
			
			
			//实际接收到的校验和
			myCRC     = Thermostat_RX_BUF[8]+Thermostat_RX_BUF[9]*256;
			if( myCRC == modbusCRC)              						//如果CRC校验通过
			{
				
				PVTem	=	(Thermostat_RX_BUF[0] + Thermostat_RX_BUF[1]*256)/10;
				SVTem	=	(Thermostat_RX_BUF[2] + Thermostat_RX_BUF[3]*256)/10;
				Output	=	Thermostat_RX_BUF[4];
				Srun	=	Thermostat_RX_BUF[5];
				Value	=	Thermostat_RX_BUF[6] + Thermostat_RX_BUF[7]*256;
				
				
				//printf("温控器 %c  %02x 成功+++++++++\r\n",operate,name);
				//printf("SVTem.val=%d℃\xff\xff\xff",SVTem);
				//printf("Output.val=%d\r\n",Output);
				//printf("Srun.val=%d\r\n",Srun);
				//printf("Value.val=%d\r\n",Value);
				
				printf("wendu.val=%d\xff\xff\xff",PVTem+15);
				Usart3_RxCounter=0;
				//if(i<5){printf("第%d次重试------成功******\r\n",5-i);}
				delay_ms(100);
				return 1;
												
			}else
			{
				printf("%02x CRC校验失败,myCRC:%x,modbusCRC:%x\r\n",name,myCRC,modbusCRC);	
			}
				
		}else{
		
			printf("温控器 %c  %02x 返回数据超时******\r\n",operate,name);
		}
		
		//未正确获取返回数据：串口3缓存计数清零
		Usart3_RxCounter=0;
		//printf("第%d次重试******\r\n",6-i);
		i=i-1;
		delay_ms(100);
	}
	
	// i<0 严重故障：温控器连接异常
	printf("\r\n\r\n严重故障：温控器连接异常---------------------\r\n\r\n");
	return 0;
}


/****************************************/
/*										*/
/*										*/
/*										*/
/*										*/
/****************************************/
	                                           
