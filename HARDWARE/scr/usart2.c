
/*-------------------------------------------------*/
/*                                                 */
/*          	       串口2                  	   */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f10x.h"  //包含需要的头文件
#include "usart2.h"     //包含需要的头文件


//unsigned int Usart2_RxCounter = 0;      //定义一个变量，记录串口2总共接收了多少字节的数据
//char Usart2_RxBuff[USART2_RXBUFF_SIZE]; //定义一个数组，用于保存串口2接收到的数据   	

//电压电流检测


u8 getbuf[30];
u8 length,flag=0,flag3=1; //数据长度 接收标志
u32 U_arg,I_arg;    //电压、电流参数
u32 U_val,I_val;    //电压、电流寄存器值
volatile float U,I;          //电压、电流实际值
float U_cof=1.88,I_cof=1;   //电压、电流系数
/*-------------------------------------------------*/
/*函数名：初始化串口2发送功能                      */
/*参  数：bound：波特率                            */
/*返回值：无                                       */
/*-------------------------------------------------*/
void usart2_init(unsigned int bound)
{  	 	
    GPIO_InitTypeDef GPIO_InitStructure;     //定义一个设置GPIO功能的变量
	USART_InitTypeDef USART_InitStructure;   //定义一个设置串口功能的变量
	NVIC_InitTypeDef NVIC_InitStructure;     //如果使能接收功能，定义一个设置中断的变量
      	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能串口2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOA时钟
	USART_DeInit(USART2);                                  //串口2寄存器重新设置为默认值
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;              //准备设置PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO速率50M
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       //复用推挽输出，用于串口2的发送
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //设置PA2
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;              //准备设置PA3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //浮空输入，用于串口2的接收
    GPIO_Init(GPIOA, &GPIO_InitStructure);                 //设置PA3
	
	USART_InitStructure.USART_BaudRate = bound;                                    //波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //8个数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	               //收发模式
      
    USART_Init(USART2, &USART_InitStructure);                                      //设置串口2	

	USART_ClearFlag(USART2, USART_FLAG_RXNE);	              //清除接收标志位
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            //开启接收中断
	
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;         //设置串口2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //中断通道使能
	NVIC_Init(&NVIC_InitStructure);	                          //设置串口2中断 

	
	USART_Cmd(USART2, ENABLE);                                //使能串口2
}



/*-------------------------------------------------*/
/*函数名：串口2接收中断                              */
/*参  数：data：数据                               */
/*返回值：无                                       */
/*-------------------------------------------------*/

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
u8 Res;
//u8 i;
	
	if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
	{
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_ORE);
	}

	if(USART_GetITStatus(USART2,USART_IT_RXNE)!= RESET)
  {
		 Res=USART_ReceiveData(USART2); 
			
		 getbuf[length]=Res;
		 length++;
		if(flag==0&&getbuf[0]!=0xaa&&getbuf[0]!=0x55&&(getbuf[0]|0x0f)!=0xff){
			length=0;
		}
		if(length==2){
			if(getbuf[0]==0xaa){
				flag=3;//故障
			}else if(getbuf[0]==0x55&&getbuf[1]==0x5a){
				flag=1;//正常
			}else if((getbuf[0]|0x0f)==0xff){
				flag=2; //溢出
				
			}else{
				length=0;
				flag=0;//接收错误
				
			}
		}
		if(flag==1&&length==24){//正确的情况下一次接收24字节
				U_arg=getbuf[2]<<16|getbuf[3]<<8|getbuf[4];
				U_val=getbuf[5]<<16|getbuf[6]<<8|getbuf[7];
				I_arg=getbuf[8]<<16|getbuf[9]<<8|getbuf[10];
				I_val=getbuf[11]<<16|getbuf[12]<<8|getbuf[13];
				U=(float)U_arg/U_val*U_cof;
				I=(float)I_arg/I_val*I_cof;
		
			length=0;
			flag=0;
		}
		if(flag==2&&length==24){//溢出的情况

			U=225;
			if((getbuf[0]|0xfb)!=0xff){
				I_arg=getbuf[8]<<16|getbuf[9]<<8|getbuf[10];
				I_val=getbuf[11]<<16|getbuf[12]<<8|getbuf[13];
				I=(float)I_arg/I_val*I_cof;

			}else{
			  I=0;
			}

			length=0;
			flag=0;
		}
		if(flag==3&&length==24){//故障的情况
			//相应的操作
			length=0;
			flag=0;
		}
			
	}	 
	
} 

void getVI()
{

	printf("shiwangdianya=%d\xff\xff\xff",(int)U);
	printf("shiwangdianliu=%d\xff\xff\xff",(int)I);
}
// 定义一个函数，用于轮询方式接收 USART 数据
//void USART_Receive_Polling(void) {
//    // 进入一个无限循环
//    while(1) {
//        // 检查接收缓冲区是否有数据
//        if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET) {
//            // 如果接收缓冲区有数据
//            // 从接收缓冲区读取数据
//            uint8_t data = USART_ReceiveData(USART2);
//            // 处理接收到的数据
//            // 这里可以添加代码来解析和处理接收到的数据
//        }
//        // 如果接收缓冲区无数据，则继续轮询
//        // 可以添加延时以降低 CPU 占用率
//        // delay_ms(10);
//    }
//}

