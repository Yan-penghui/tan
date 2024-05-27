/*------------------------------------------------------*/
/*                                                      */
/*            程序main函数，入口函数源文件               */
/*                                                      */
/*------------------------------------------------------*/

#include "sys.h"
#include "delay.h"	     //包含需要的头文件
#include "usart1.h"      //包含需要的头文件
#include "usart2.h" 
#include "usart3.h" 
#include "uart4.h"      //包含需要的头文件
#include "timer3.h"      //包含需要的头文件
#include "timer4.h"      //包含需要的头文件
#include "stdio.h"      //包含需要的头文件
#include "FreeRTOS.h"	 //FreeRTOS配置头文件
#include "semphr.h" 	 //信号量
#include "queue.h"		 //队列
#include "event_groups.h"//事件标志组
#include "Thermostat.h"  
#include "wifi.h"	     //包含需要的头文件
#include "mqtt.h"        //包含需要的头文件
#include "control.h"     //包含需要的头文件 控制模块相关数据发送给服务器
#include "switch.h"	     //包含需要的头文件 LED
#include "AHT20.h"       //包含需要的头文件 空气温湿度
#include "stdlib.h"
#include "door.h" 
/*-------------------------------------------------------------*/
/*          	WIFI网络与ONENET配置（配置）			      	   */
/*-------------------------------------------------------------*/
const char SSID[] 			 = "taizidejia";          //路由器名称
const char PASS[] 			 = "y1032979127.";    //路由器密码

const char PRODUCTID[] 	     = "394499";  	   //产品ID
const char DEVICEID []	     = "661126800";    //设备ID  
const char AUTHENTICATION[]  = "123456";       //鉴权信息  
const char DATA_TOPIC_NAME[] = "$dp";		   //topic，Onenet数据点上传topic（不用改）
const char SERVER_IP[]	     = "kf30cdfc.ala.cn-hangzhou.emqxsl.cn";//存放服务器IP或域名（不用改）
const int  SERVER_PORT 		 = 8883;		   //存放服务器的端口号（不用改）

/*-------------------------------------------------------------*/
/*          控制命令以及控制模块初始状态设置（配置）		   	   */
/*-------------------------------------------------------------*/
	/* 消息体：
	 *  {
	 *		"data_1":"value_1",
	 *		"data_2":"value_2"
	 *	}
	 *	消息体示例：
	 *	{"led1_flag":"LED1ON"}
	 */

const char *LED1_LABER  = "led1_flag";//LED1标签，发送给ONENET的数据流名称
const char *CMD_LED1ON  = "LED1ON";   //LED1打开
const char *CMD_LED1OFF = "LED1OFF";  //LED1关闭
char 	   *led1_flag   = "LED1OFF";  //LED1状态，初始化为关闭状态

const char *LED2_LABER 	= "led2_flag";//LED2标签
const char *CMD_LED2ON  = "LED2ON";   //LED2打开
const char *CMD_LED2OFF = "LED2OFF";  //LED2关闭
char 	   *led2_flag   = "LED2ON";   //LED2状态，初始化为打开状态
/*-------------------------------------------------------------*/
/*               freerto任务通信控制（固定）			      	   */
/*-------------------------------------------------------------*/

/*	二值信号量句柄                         
 *	作用：用于控制MQTT命令缓冲处理任务，在MQTT数据接收发送缓冲处理任务中发出
 *		  当命令缓冲区收到命令数据时，发出信号量		 
 */
SemaphoreHandle_t BinarySemaphore;
	
/*	事件标志组                         
 *	作用：标志WIFI连接，PING心跳包发送模式控制wifi是否重启连接，是否发送数据，传感器是否运行 
 *  具体：1.事件标志组位1为0，位0为1时，即0x03（0000 0001），wifi连接至服务器时位0置位1，此时connect报文还未发送。 
 *		  2.事件标志组位1为1，位0为1时，即0x03（0000 0011），connect报文发送，返回连接成功报文时位1置位1，PING心
 *			跳包开启30s发送模式，传感器任务开启，数据开始上传，设备远程控制（LED控制）功能开启。 
 */
EventGroupHandle_t Event_Handle = NULL;     //事件标志组（位0：WIFI连接状态 位1：PING心跳包2S快速发送模式）
const int WIFI_CONECT = (0x01 << 0);        //设置事件掩码的位 0；服务器连接模式，值1表示已经连接，0表示未连接
const int PING_MODE   = (0x01 << 1);        //设置事件掩码的位 1；PING心跳包发送模式，1表示开启30S发送模式，0表示未开启发送或开启2S快速发送模式

/*	传感器数据发送消息队列                         
 *	作用：将传感器的数据发送到传感器消息队列  
 */
QueueHandle_t Message_Queue;		 		//消息队列句柄  
const UBaseType_t MESSAGE_DATA_TX_NUM = 5;	//消息队列最大消息数目  
const UBaseType_t MESSAGE_DATA_TX_LEN = 100;//消息队列单元大小，单位为字节  


QueueHandle_t U1_xQueue,U3_xQueue,runcode_xQueue;


//全局变量
u8 door_flag=0;
int runingtime,Alltime=0;
u8 Runcode_flag=0;
/*-------------------------------------------------------------*/
/*               任务句柄及任务函数声明1（配置）		      	   */
/*-------------------------------------------------------------*/

//开始任务
TaskHandle_t StartTask_Handler;
void my_start_task(void *pvParameters);

//Showhmi任务，串口屏
TaskHandle_t Showhmi_Task_Handler;
void my_showhmi_task(void *pvParameters);

//GetVI任务，串口屏
TaskHandle_t GetVI_Task_Handler;
void my_getVI_task(void *pvParameters);

//解析串口屏指令任务
TaskHandle_t Instruct_Task_Handler;
void my_instruct_task(void *pvParameters);

//WIFI任务
TaskHandle_t WIFI_Task_Handler;
void wifi_task(void *pvParameters);

//运行加热程序任务
TaskHandle_t Runcode_Task_Handler;
void my_runcode_task(void *pvParameters);

//运行炉门动作任务
TaskHandle_t Door_Task_Handler;
void my_door_task(void *pvParameters);

//堆栈检测任务
TaskHandle_t Stack_Task_Handler;
void stack_task(void *pvParameters);

/*---------------------------------------------------------------*/
/*函数名：int main()                                             */
/*功  能：主函数							                         */
/*		  1.初始化各功能模块  				     				 */
/*		  2.创建开始任务（在开始任务里创建所有其他任务）           */
/*		  3.开启任务调度				       			 		     */
/*参  数：无                          			   				 */
/*返回值：无                                       			     */
/*---------------------------------------------------------------*/
int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		//设置系统中断优先级分组4
	delay_init();	      							 	//延时函数初始化
	usart1_init(115200);  								 //串口1功能初始化，波特率115200，与串口助手/串口屏通信
	usart2_init(4800);      							 //串口2功能初始化，波特率4800，与电压电流检测模块通信
	usart3_init(9600);   								//串口3功能初始化，波特率9600，与温控器通信
	usart4_init(115200);  								 //串口4功能初始化，波特率115200，wifi通信	
	tim4_init(500,7200);   								//TIM4初始化，定时时间 500*7200*1000/72000000 = 50ms	
	
	Switch_Init();    									//初始化外部控制 
//	IoT_parameter_init();  								//初始化MQTT服务器的参数	
	Init_I2C_Sensor_Port(); 							//IIC初始化
	delay_ms(500);
	
	if((AHT20_Read_Status()&0x18)!=0x18)
	{
		AHT20_Start_Init();
		Delay_1ms(10);
	}	
	
		YELLOW_LED = 0;
		RED_LED    = 1;
		GREEN_LED  = 1;

	//创建开始任务
	xTaskCreate((TaskFunction_t	) my_start_task,		//任务函数
			    (const char* 	)"my_start_task",		//任务名称
				(uint16_t 		) 128,				  	//任务堆栈大小
				(void* 		  	) NULL,				 	//传递给任务函数的参数
				(UBaseType_t 	) 1, 				  	//任务优先级
				(TaskHandle_t*  ) &StartTask_Handler);	//任务控制块 
			
	vTaskStartScheduler();  							//开启任务调度
}

/*---------------------------------------------------------------*/
/*函数名：void my_start_task(void *pvParameters)                 */
/*功  能：开始任务（配置）							             */
/*		  1.创建信号量，消息队列等任务通信方式   				     */
/*		  2.创建所有任务       			 						 */
/*		  3.删除本身       			 		    				 */
/*参  数：无                          			   				 */
/*返回值：无                                       			     */
/*---------------------------------------------------------------*/
void my_start_task(void *pvParameters)
{
	//taskENTER_CRITICAL(); //进入临界区
	
	//创建队列：从串口1中断读取指令数据
	 U1_xQueue = xQueueCreate(1, 70 );

	//创建队列：从串口3中断读取返回数据
	 U3_xQueue = xQueueCreate(1, 30 );	
	
	//创建队列：发送程序参数给runcode任务
	 runcode_xQueue = xQueueCreate(1, 80 );
	
	//创建二值信号量
	BinarySemaphore = xSemaphoreCreateBinary();	
	//事件标志组，用于标志wifi连接状态以及ping发送状态
	Event_Handle = xEventGroupCreate(); 
	//创建传感器消息体消息队列
	//Message_Queue = xQueueCreate(MESSAGE_DATA_TX_NUM, MESSAGE_DATA_TX_LEN); 
	
	//任务创建函数参数；1.任务函数 			2.任务名称 			3.任务堆栈大小 3.传递给任务函数的参数 4.任务优先级 5.任务控制块
	//创建WIFI任务
    //xTaskCreate(wifi_task, 				"wifi_task", 				128, NULL, 1, &WIFI_Task_Handler); 			
			
	//创建每秒发送系统状态任务 ，给串口屏
    xTaskCreate(my_showhmi_task, 			"my_showhmi_task", 			128, NULL, 1, &Showhmi_Task_Handler);
	
	//创建每秒发送V、I、环境温度、湿度任务 ，给串口屏
    xTaskCreate(my_getVI_task, 			"my_getVI_task", 			128, NULL, 1, &GetVI_Task_Handler);
	
	
	//创建解析指令任务
	xTaskCreate(my_instruct_task, 			"my_instruct_task", 		128, NULL, 3, &Instruct_Task_Handler);
	
//	//创建运行程序任务
//	xTaskCreate(my_runcode_task, 			"my_runcode_task", 			128*2, NULL, 1, &Runcode_Task_Handler);
	
	//创建炉门动作任务
	xTaskCreate(my_door_task, 			"my_door_task", 			128, NULL, 1, &Door_Task_Handler);
	
	//创建检测任务
	//xTaskCreate(stack_task,    			    "my_stack_task",         128, NULL, 1, &Stack_Task_Handler);	
		
    vTaskDelete(StartTask_Handler); //删除开始任务
//	taskEXIT_CRITICAL();            //退出临界区
}


void rehoting(int rehottem,int abstem,int i)
{
	//暂停其他modbus指令发送
	//发送预热指令：写pno=0恒温模式，写sv温度值rehottem
	Send_Thermostat('w', 0x1b, 0 );					//写运行
	Send_Thermostat('w', 0x2b, 0 );  				//写恒温模式
	Send_Thermostat('w', 0x50, rehottem*10 );		//写恒温温度


	
	
	//printf("预干燥升温开始，干燥温度：%d度\r\n",rehottem);
	
	
	//printf("运行第%d步,等待温度到设定值%d度的 %d以内 \r\n",i,rehottem,abstem);
	
	while(abs(PVTem-rehottem) > abstem )
	{
		//printf("运行第%d步,等待温度到设定值%d度的 %d以内 \r\n",i,rehottem,abstem);
		Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度
		delay_ms(1000);
	}
	door_flag=1;
	updoor(1);
	delay_ms(2000);
		
}


/*auto_doordown(u8 allstep,int atuodoor_downTem（炉门开启温度）,u8 stoptime(ms),int rehoting_doorrun_alltime)  */
void auto_doordown(u8 allstep,int atuodoor_downTem,int stoptime,int rehoting_doorrun_alltime)
{
	int i=0;
	for(i=0 ; i<allstep ;i++)
		{
			delay_ms(stoptime);
			//实际温度与炉门温差小于指定温差时，等待
			while( abs(PVTem-atuodoor_downTem) < abs(atuodoor_downTem-400)/allstep*(i+1) )
			{
				Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度
		
				delay_ms(1000);
			
			}
	
			//未触发下限位开关可运行
			while(rehoting_doorrun_alltime/allstep--)
			{
				if(LIM2)break;
				DOWN=0;
				UP = 1; //低电平有效.	
				delay_ms(100); 
			}
			
			DOWN=1;
			UP = 1; //低电平有效
		}
}

void heating(u8 run_p_num,u8 all_p_num,float starttem,float endtem,u16 time)
{
	TickType_t xLastWakeTime;
	int i=0,j=0;
	float pvtem,prograss_time;
	float v=(endtem - starttem)/time;
	prograss_time=time;
	//while (Send_Thermostat('w',0x1b,0)){ break;}			//1b-Srun： 0-run 1-stop 2-hold
    Send_Thermostat('w',0x1b,0);			//1b-Srun： 0-run 1-stop 2-hold
	delay_ms(1000);
	Send_Thermostat('w', 0x50, starttem*10 );  				
	
    xLastWakeTime = xTaskGetTickCount();
	printf("zhuangtai.val=2\xff\xff\xff");		//0-待机 1-预烘干 2-运行 3-故障
	//printf("page yunxing\xff\xff\xff");
	
	while(time--)
	{
		
		pvtem = starttem +	v*i + 1;
		//printf("起始温度：%f ，最终温度：%f ，当前设定温度：%f  升温速率：%f 度/s \r\n",starttem,endtem,pvtem,v);
		Send_Thermostat('w', 0x50, pvtem*10 );
		vTaskDelayUntil(&xLastWakeTime,1000); 
		i=i+1;
		runingtime = runingtime + 1;
		
		
		printf("zhuangtai.val=2\xff\xff\xff");		//0-待机 1-预烘干 2-运行 3-故障
		//printf("page yunxing\xff\xff\xff");	

		printf("zongduan.val=%d\xff\xff\xff",all_p_num);
		printf("dangqianduan.val=%d\xff\xff\xff",run_p_num+1);
		
		printf("benduanfen.val=%d\xff\xff\xff",((int)prograss_time-i)%3600/60);
		printf("benduanmiao.val=%d\xff\xff\xff",((int)prograss_time-i)%3600%60);	
		printf("shengyushi.val=%d\xff\xff\xff",(Alltime-runingtime)/3600);	
		printf("shengyufen.val=%d\xff\xff\xff",(Alltime-runingtime)%3600/60);	
		printf("shengyumiao.val=%d\xff\xff\xff",(Alltime-runingtime)%3600%60);		
		printf("jindu.val=%d\xff\xff\xff",runingtime*100/Alltime);
		
	}
	
	//终了温度未到预设温度+-3内，进入等待
	if( PVTem < endtem-3 || PVTem > endtem+3 ){		
		printf("未达到预设值，进入等待++++++++++++++++++++++++");
					
	}
	j=0;
	while(abs(PVTem-endtem)>3 && endtem!=0){		
		Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度
		j=j+1;
		delay_ms(1000);
		printf("等待%d  ,当前%d， 时间%ds++++++++++++++++++++++++",(int)endtem,PVTem,j);					
	}	
		
	printf("达到预设值，结束等待++++++++++++++++++++++++");	
	
}

/*---------------------------------------------------------------*/
/*函数名：void my_runcode_task(void *pvParameters)   		    */
/*功  能：运行程序任务       							 */
/*参  数：无                          			   				 */
/*返回值：无                                       			     */
/*其  他：                                                       */
/*---------------------------------------------------------------*/
void my_runcode_task(void *pvParameters)	
{
	int i,j,nowtem,progress_num,progress_time;
	float percent;
	u8 rehot = 0 ,atuodoor = 0,fan =0,all_p_num;
	u16 rehottem = 0 ;
	int rehottime = 0 ,atuodoor_downTem=0;
	u16 runcode[40];
	u16 Program_data[17][2];
	
	#define rehoting_doorrun_alltime 25*10			//单位：0.1s
	while(1)
	{
		//printf("创建任务------Runcode------\r\n");
		
		if (pdPASS == xQueueReceive( runcode_xQueue,&runcode,portMAX_DELAY ) )    	//读队列
		{    

			for(i=0;i<17;i+=1)
			{
				if(runcode[2*i]!=0){Program_data[i][0]	=	runcode[2*i]-15;}
				else{Program_data[i][0]	=	0;}
				Program_data[i][1]	=	runcode[2*i+1]*60;
				printf("[%d][0] = %d     [%d][1] = %d\r\n", i,Program_data[i][0],i,Program_data[i][1]);
				
			}
			
			fan  				=	runcode[34];
			atuodoor			=	runcode[35] ;
			atuodoor_downTem	=	runcode[36]-15 ;
			rehot				=	runcode[37] ;
			rehottem			=	runcode[38]-15 ;
			rehottime			=	runcode[39]*6 ;	//*60 测试：修改为*6
			printf("fan  = %d\r\n",fan);
			printf("atuodoor  = %d\r\n",atuodoor);
			printf("atuodoor_downTem  = %d\r\n",atuodoor_downTem);
			printf("rehot  = %d\r\n",rehot);
			printf("rehottem  = %d\r\n",rehottem);
			printf("rehottime  = %d\r\n",rehottime);
			
			
			if(!LIM1&&!rehot)
			{
				printf("page yunxing\xff\xff\xff");		//跳转至运行界面
				printf("page yunxing\xff\xff\xff");		//跳转至运行界面
				printf("page guanbilumen\xff\xff\xff");
				Runcode_flag=0;	
				vTaskDelete(NULL);
			}
				
			printf("page yunxing\xff\xff\xff");		//跳转至运行界面
			printf("page yunxing\xff\xff\xff");		//跳转至运行界面
			printf("vis b3,0\xff\xff\xff");			//隐藏 开始 按钮
			printf("vis up,0\xff\xff\xff");			//隐藏 上下 按钮
			printf("vis down,0\xff\xff\xff");
			printf("jindu.val=0\xff\xff\xff");
			//数据写入Program_data[tem][time:s]
			printf("\r\n");	
				
				
			
			if(fan==0)
			{
				FAN_DOWN=0;
			}
			
			
//			/*debug*/
//				//完全关闭炉门
//				door_flag=1;
//				updoor(0);
//				delay_ms(1000);
//			/**/
			
		//	printf("开始运行\r\n");
			
		{
			YELLOW_LED = 1;
			RED_LED    = 1;
			GREEN_LED  = 0;
		}
			vTaskSuspend(Showhmi_Task_Handler); //LED2任务由就绪态（运行态）转为挂起态，LED2任务挂起（停止）  
			//若：开启预热--先完全关闭再下降
			if(rehot)
				{
					printf("jindu.val=0\xff\xff\xff");
					printf("zhuangtai.val=1\xff\xff\xff");		//0-待机 1-预烘干 2-运行 3-故障
					printf("page yunxing\xff\xff\xff");
					printf("zhuangtai.val=1\xff\xff\xff");		//0-待机 1-预烘干 2-运行 3-故障
					printf("page yunxing\xff\xff\xff");
					//下降到最低
					door_flag=2;
					downdoor(0);
					delay_ms(2000);
					
					//温度预热到指定值
					printf("开始预干燥\r\n");
					
					//分三次上升到指定位置，每次2s
			  
					//第一次运行：温差200度以内
					rehoting(rehottem+3,200,1);
					
					//第二次运行：温差100度以内
					rehoting(rehottem+2,100,2);
					
					//第三次运行：温差10度以内
					rehoting(rehottem+1,10,3);

					//若炉门异常(完全关闭)，下降2s
					if(LIM1)
					{
						//下降到指定位置
						DOWN=0;
						UP = 1; //低电平有效
						delay_ms(2000);   
						DOWN=1;
						UP = 1; //低电平有效
					}
					
					//烘干指定时间
					while(rehottime--)
					{
						Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度
						delay_ms(1000); 			//烘干开始计时，单位：分钟
						//printf("预干燥剩余时间：%ds\r\n",rehottime);
						printf("zhuangtai.val=1\xff\xff\xff");		//0-待机 1-预烘干 2-运行 3-故障
						printf("jindu.val=0\xff\xff\xff");
						printf("shengyushi.val=%d\xff\xff\xff",rehottime/60/60);	
						printf("shengyufen.val=%d\xff\xff\xff",rehottime%3600/60);	
						printf("shengyumiao.val=%d\xff\xff\xff",rehottime%3600%60);
						
						
					}
					
					
					//烘干结束分六步上升
					for(i=0;i<7;i++)
					{
						vTaskResume(Showhmi_Task_Handler);  //Showhmi_Task由挂起态转为就绪态，Showhmi_Task任务运行
						
						door_flag=1;
						updoor(2);
						delay_ms(4000);	
						vTaskSuspend(Showhmi_Task_Handler); //Showhmi_Task由就绪态（运行态）转为挂起态，Showhmi_Task由挂起（停止）
						
					}

					//结束完全关闭炉门
					door_flag=1;
					updoor(0);
				}
				
				
				
				
				
				
				
//开启屏保				
printf("yunxing.tm0.en=1\xff\xff\xff");
printf("yunxing.tm0.en=1\xff\xff\xff");				
			
	delay_ms(500);
	//寻找当前温度对应的起点温度段-温度点
	nowtem=PVTem;
	
	//程序理论总时长
	Alltime=0 ;
	for(j=0;j<16;j++)
	{
		Alltime=Alltime + Program_data[j][1] ;
	}
	printf("--------------总时长%ds\r\n",Alltime);	
	
	//程序理论总段数
	all_p_num=0 ;
	for(j=0;j<16;j++)
	{
		if(Program_data[j][0]==0){all_p_num=j-1;break;} ;
	}
	
	printf("--------------总段数: %d\r\n",all_p_num);	
					
	//寻找加热段序号
	for(i=0 ;i<17 ;i++)
	{
		//printf("当%d段检查\r\n",i);
		//当前温度为恒温段
		if(nowtem==Program_data[i][0])
		{
			progress_num = i+2;	
			printf("当前温度%d，进入第%d段*******恒温段\n",nowtem,progress_num+1);	
			progress_time=Program_data[progress_num][1] ;					//获取当前段总时长s
			//计算已运行时长
			runingtime=0;
			for(j=0;j<=i;j++){
				runingtime=runingtime + Program_data[j][1] ;
			}

			printf("总时长%ds,当前已运行%ds\xff\xff\xff",Alltime,runingtime);	
			
			heating(i,all_p_num,nowtem,Program_data[i+2][0],Program_data[i+1][1]);
			break;
				
		}
			//当前温度为升温段 i为目标温度点的坐标
		if(nowtem<Program_data[i+1][0]&&nowtem>Program_data[i][0])
		{
					
			progress_num = i;	
			//printf("当前温度%d，进入第%d段*******升温段\n",nowtem,progress_num);
			percent=1-(float)(nowtem-Program_data[i][0])/(Program_data[i+1][0]-Program_data[i][0]);	
			
			//计算已运行时长
			runingtime=0;
			for(j=0;j<i;j++){
				runingtime=runingtime + Program_data[j][1] ;
			}
				
			runingtime = runingtime + Program_data[i][1] * (1-percent);
		//	printf("总时长%ds,当前已运行%ds\xff\xff\xff",Alltime,runingtime);	
		//	printf("jindu.val=%d\xff\xff\xff",runingtime*100/Alltime);	
				//printf("总时长%ds\xff\xff\xff",alltime);	
				
		//	printf("当前程序段进度时间:  %2f \r\n",percent*100);
		//	printf("当前温度%d，进入第%d段\n",nowtem,progress_num);	
		//	printf("目标温度%d，总时长：%d，剩余时长：%fs\n",Program_data[i+1][0],Program_data[i][1],Program_data[i][1]*percent);
				
			progress_time=Program_data[i][1]*percent;//获取当前段总时长
			
			heating(i,all_p_num,nowtem,Program_data[i+1][0],(int)(Program_data[i][1]*percent));	 //当前温度   目标温度   时间s
			
			break;
		}	
					
	}	

		printf("剩余加热段\n");	
		//剩余加热段
		i=i+1;	
		//遇见0数据退出
		if(Program_data[i][0]==0){
			
			i=17;
			//printf("Program_data[i+1][0]==0--------结束----------\r\n");
		}
		for(i=i;i<17 ;i++){
			
			progress_time=Program_data[i][1];		//获取当前段总时长
			//printf("进入第%d段，当前温度%d，目标温度%d，总时长：%ds\n",
			//	i,Program_data[i][0],Program_data[i+1][0],progress_time);
			heating(i,all_p_num,Program_data[i][0],Program_data[i+1][0],progress_time);		
			
			//遇见0数据退出
			if(Program_data[i+1][0]==0){
				
				i=20 ;
				//printf("Program_data[i+1][0]==0--------结束----------\r\n");
			}	
		 }

		 //烧结次数+1
		 printf("wepo shaojiecishu.val,272\xff\xff\xff");
		 printf("shaojiecishu.val=shaojiecishu.val+1\xff\xff\xff");
		 printf("wepo shaojiecishu.val,272\xff\xff\xff");
		
		 //关闭屏保				
		printf("yunxing.tm0.en=0\xff\xff\xff");
		printf("yunxing.tm0.en=0\xff\xff\xff");	

		 
			if(atuodoor)
			{
				//printf("atuodoor-------atuodoor_downTem=%d\r\n",atuodoor_downTem);
				
				//温度高于1000，不动作
				while(PVTem > 1000)
				{
					Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度
					delay_ms(1000);
					//printf("等待温度降至1000以下\r\n");
				}	
				
				//开一条缝隙：下降1s
				//printf("温度降至1000以下，开一条缝隙\r\n");
				delay_ms(1000);
				door_flag=2;
				downdoor(2);
				
				//开一条缝隙：等待温度到达设定的开启温度
				while(PVTem > atuodoor_downTem)
				{
					Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度
					delay_ms(1000);
					//printf("等待温度降至%d\r\n",atuodoor_downTem);
				}
				
				
				for(i=0;i<5;i++)
				{
					//printf("第%d次下降\r\n",i+1);
					
					delay_ms(5000);
					door_flag=2;
					downdoor(3);
					//printf("等待温度降至%d\r\n",800-i*100);
					
					while (PVTem > 800-i*100)
					{
						Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度
						delay_ms(1000);
						//printf("等待温度降至%d\r\n",800-i*100);
					}
		
				}

				//结束完全打开炉门
				door_flag=2;
				downdoor(100);
			}
		
		Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度
		Send_Thermostat('w',0x1b,1);				//1b-Srun： 0-run 1-stop 2-hold	
		delay_ms(1000); 
		//printf("-------------结束----------\r\n");
		//delay_ms(1000); 
		
	                               
		vTaskResume(Showhmi_Task_Handler);  //Showhmi_Task由挂起态转为就绪态，Showhmi_Task任务运行
		
		
	}
		

	
	//删除runcode任务
		printf("zhuangtai.val=0\xff\xff\xff");		//0-待机 1-预烘干 2-运行 3-故障
		printf("page yunxing\xff\xff\xff");
		printf("zhuangtai.val=0\xff\xff\xff");		//0-待机 1-预烘干 2-运行 3-故障
		printf("page yunxing\xff\xff\xff");
		
		printf("shengyushi.val=0\xff\xff\xff");	
		printf("shengyufen.val=0\xff\xff\xff");	
		printf("shengyumiao.val=0\xff\xff\xff");		
		printf("jindu.val=100\xff\xff\xff");
	
	
	
	
		{
			YELLOW_LED = 0;
			RED_LED    = 1;
			GREEN_LED  = 1;
		}
	
	
		printf("vis b3,1\xff\xff\xff");			//显示 开始 按钮
		printf("vis b3,1\xff\xff\xff");			//显示 开始 按钮
		printf("vis up,1\xff\xff\xff");			//显示 上下 按钮
		printf("vis down,1\xff\xff\xff");
		printf("page jieshu\xff\xff\xff");
		BUZZER=1;
		delay_ms(50);
		BUZZER=0;
		delay_ms(50);
		BUZZER=1;
		delay_ms(50);
		BUZZER=0;
		delay_ms(50);
		BUZZER=1;
		delay_ms(50);
		BUZZER=0;
	
		printf("结束任务------Runcode------\r\n");
		Runcode_flag=0;	
		vTaskDelete(NULL);
		
	}
}


void my_getVI_task(void *pvParameters)	
{
	
	delay_ms(4000);	
	while(1)
	{
		//printf("----------my_getVI_task----------\r\n");
		delay_ms(1000);	 
		//读取温湿度
		getCT();
		delay_ms(1000);	
		//读取电压电流
		getVI();

	}
}

/*---------------------------------------------------------------*/
/*函数名：void my_showhmi_task(void *pvParameters)   		    */
/*功  能：串口屏显示系统状态任务       							 */
/*参  数：无                          			   				 */
/*返回值：无                                       			     */
/*其  他：                                                       */
/*---------------------------------------------------------------*/
void my_showhmi_task(void *pvParameters)	
{
	
	//温控器初始化
	delay_ms(5000);	  
	Thermostat_init();
	
	while(1)
	{
		//printf("----------my_showhmi_task----------\r\n");
		delay_ms(1000);	 
		
		Send_Thermostat('r',0x4a,0);              	//4a-PV：测量温度

		if(Srun==0){Send_Thermostat('w',0x1b,1);}				//1b-Srun： 0-run 1-stop 2-hold		
		
	}
}

/*---------------------------------------------------------------*/
/*函数名：void my_instruct_task(void *pvParameters)   		     */
/*功  能：串口屏指令解析状态任务       							 */
/*参  数：无                          			   				 */
/*返回值：无                                       			     */
/*其  他：读队列 U1_xQueue                                                 */
/*---------------------------------------------------------------*/
void my_instruct_task(void *pvParameters)	
{
	int i,instruct;
	u16 TEM_data[8];
	u16 V_data[8];
	u16 H_data[8];
    u16 myProgram_data[17][2];
	u16 runcode[41];
	u8 instruct_RxBuff[70];
	
	while(1)
	{
		//printf("----------my_instruct_task----------\r\n");
		if (pdPASS == xQueueReceive( U1_xQueue,&instruct_RxBuff,portMAX_DELAY ))    	//读队列
		{    								
//			
			BUZZER=1;
			delay_ms(100);  
			BUZZER=0;
			//printf("instruct_RxBuff=\r\n");
//			
//			for (i=0;i<70;i++)
//			{
//				printf("%2x  ",instruct_RxBuff[i]);
//			}	
			
			instruct=instruct_RxBuff[1];
			//printf("接受指令%02X：\r\n",instruct);
			if(instruct == 0x01)
			{	
				
				//创建runcode任务	
				if(Runcode_flag==0)
				{
					xTaskCreate(my_runcode_task, "my_runcode_task", 128*2, NULL, 4, &Runcode_Task_Handler);
				}else{printf("已有任务被创建，无法再次执行------Runcode------\r\n");}
				
				Runcode_flag=1;
				//程序参数整理 Program_data[17][2]:[温度][时间]
				for(i=2;i<17;i+=2)
				{
					TEM_data[(i-2)/2]		=(instruct_RxBuff[i+1]<<8|instruct_RxBuff[i]);          //温度：摄氏度
					V_data[(i-2)/2]		=instruct_RxBuff[i+1+16]<<8|instruct_RxBuff[i+16];		//速率：摄氏度/分钟
					H_data[(i-2)/2]		=instruct_RxBuff[i+1+32]<<8|instruct_RxBuff[i+32];		//保持时间：分钟
				}	
				
				myProgram_data[0][0]	= 50;													//第一个温度点：50摄氏度
				myProgram_data[0][1]	= (TEM_data[0]-50)/V_data[0];
				
				for(i=0;i<8;i+=1)
				{
					myProgram_data[2*i+1][0]	= TEM_data[i];						//温度
					myProgram_data[2*i+1][1]	= H_data[i];
					
					myProgram_data[2*i+2][0]	= TEM_data[i];						//温度
					myProgram_data[2*i+2][1]	= abs(TEM_data[i+1]-TEM_data[i])/V_data[i+1];
					if(i>=7){myProgram_data[2*i+2][1]	= 0;}
				}

//				//测试：显示构成程序的数组参数
				for(i=0;i<17;i+=1)
				{
//						printf("Program_data%d=%d   %d\r\n",i,Program_data[i][0],Program_data[i][1]);	
//						//if(Program_data[i][1]==0){i=50;}
					runcode[2*i] = myProgram_data[i][0];
					//printf("runcode[%d]=%d \r\n",2*i,runcode[2*i] );
					runcode[2*i+1] = myProgram_data[i][1];
					//printf("runcode[%d]=%d  \r\n",2*i+1,runcode[2*i+1]);
				}
	
				
				//高级设置参数写入
				runcode[34] = instruct_RxBuff[51]<<8|instruct_RxBuff[50] ;				//fan
				runcode[35] = instruct_RxBuff[53]<<8|instruct_RxBuff[52];				//atuodoor
				runcode[36] = instruct_RxBuff[55]<<8|instruct_RxBuff[54];				//atuodoor_downTem
				runcode[37] = instruct_RxBuff[57]<<8|instruct_RxBuff[56];				//rehot
				runcode[38] = instruct_RxBuff[59]<<8|instruct_RxBuff[58];				//rehottem
				runcode[39] = instruct_RxBuff[61]<<8|instruct_RxBuff[60] ;				//rehottime
				//runcode_flag = 1;
				
//				for(i=0;i<40;i+=1)
//				{
//					printf("runcode[%d]=%d  \r\n",i,runcode[i]);
//				}	
				
				//写队列给runcode任务
				
				xQueueSendToBack( runcode_xQueue,&runcode,30);	//写队列
					
			}
			else if(instruct == 0x10)
			{
				door_flag=1;      //0 停止，1 up ,2 down	
			}
			else if(instruct == 0x11)
			{
		
				door_flag=0;      //0 停止，1 up ,2 down
				DOWN=1;
				UP = 1; /*低电平有效*/
				printf("up.pic=0\xff\xff\xff");
				printf("down.pic=2\xff\xff\xff");
					
			}	
			else if(instruct == 0x12)
			{	
				door_flag=2;      //0 停止，1 up ,2 down
			}	

			else if(instruct == 0x13)
			{	
				     
			}			
			else if(instruct == 0x14)
			{	
				
				if(Runcode_flag){
					//删除runcode任务	
					door_flag=0;      //0 停止，1 up ,2 down
					vTaskDelete(Runcode_Task_Handler);
					
					Send_Thermostat('w',0x1b,1);		//1b-Srun： 0-run 1-stop 2-hold	
					vTaskResume(Showhmi_Task_Handler);  //Showhmi_Task由挂起态转为就绪态，Showhmi_Task任务运行
					Runcode_flag=0;
					printf("zhuangtai.val=0\xff\xff\xff");		//0-待机 1-预烘干 2-运行 3-故障
					printf("page yunxing\xff\xff\xff");
					{
						YELLOW_LED = 0;
						RED_LED    = 1;
						GREEN_LED  = 1;
					}
					
					printf("vis b3,1\xff\xff\xff");			//显示 开始 按钮				
					printf("vis b3,1\xff\xff\xff");			//显示 开始 按钮
					
					printf("vis up,1\xff\xff\xff");			//显示 上下 按钮
					printf("vis down,1\xff\xff\xff");
					printf("主动删除------Runcode------\r\n");
				}else{printf("无任务可删除------Runcode------\r\n");}
				
			}			
			else if(instruct == 0x15)
			{	
//				YELLOW=1;
//				RED =1;
//				GREEN=1 ;
//				
//				delay_ms(200);        //RED
//				YELLOW=1;
//				RED =0;
//				GREEN=1 ;
			}
			else if(instruct == 0x16)
			{					
//				YELLOW=1;
//				RED =1;
//				GREEN=1 ;
//				delay_ms(200);
//				YELLOW=1;
//				RED =1;
//				GREEN=0 ;     //GREEN
			}
			else if(instruct == 0x17)
			{	
//				YELLOW=1;
//				RED =1;
//				GREEN=1 ;
//				delay_ms(200);
//				YELLOW=0;
//				RED =1;
//				GREEN=1 ;      //YELLOW
			}
			
			
			
		}
		Usart1_RxCounter=0;	
	
	
	}
	
}


//  vTaskSuspend(Led2_Task_Handler); //LED2任务由就绪态（运行态）转为挂起态，LED2任务挂起（停止）                                         
//	vTaskResume(Led2_Task_Handler);  //LED2任务由挂起态转为就绪态，LED2任务运行



/*---------------------------------------------------------------*/
/*函数名：void wifi_task(void *pvParameters)                     */
/*功  能：WIFI任务（固定）										 */
/*		  1.连接wifi以及云服务器       							 */
/*		  2.断线重连        									     */
/*参  数：无                          			   				 */
/*返回值：无                                       			     */
/*其  他：1.服务器连接前关闭发送ping包的定时器3，清除事件标志位	 */
/*		  2.服务器已连接，抛出事件标志，挂起自己，进入挂起态		 */
/*		  3.服务器或者wifi已断开，清除事件标志，继续执行本任务重新 */
/*			连接	 											 */
/*---------------------------------------------------------------*/
void wifi_task(void *pvParameters)
{
	while(1)
	{ 
		printf("需要连接服务器\r\n");                 
		TIM_Cmd(TIM4, DISABLE);                       //关闭TIM4 
		TIM_Cmd(TIM3, DISABLE);                       //关闭TIM3
		xEventGroupClearBits(Event_Handle, PING_MODE);//关闭发送PING包的定时器3，清除事件标志位
		WiFi_RxCounter = 0;                           //WiFi接收数据量变量清零                        
		memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);     //清空WiFi接收缓冲区 
		if(WiFi_Connect_IoTServer() == 0)			  //如果WiFi连接云服务器函数返回0，表示正确，进入if
		{   			     
			printf("MQTT服务器连接成功\r\n");            
			WiFi_RxCounter = 0;                       //WiFi接收数据量变量清零                        
			memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE); //清空WiFi接收缓冲区 
			MQTT_Buff_Init();                         //初始化发送缓冲区  
			
			xEventGroupSetBits(Event_Handle, WIFI_CONECT);  //服务器已连接，抛出事件标志 
			vTaskSuspend(NULL);	    						//服务器已连接，挂起自己，进入挂起态（任务由挂起转为就绪态时在这继续执行下去）
			xEventGroupClearBits(Event_Handle, WIFI_CONECT);//服务器或者wifi已断开，清除事件标志，继续执行本任务，重新连接 
			xEventGroupClearBits(Event_Handle, PING_MODE);  //关闭发送PING包的定时器3，清除事件标志位
		}
		
		delay_ms(1000);	    //延时1s
	}
}

/*---------------------------------------------------------------*/
/*函数名：void stack_task(void *pvParameters)                    */
/*功  能：任务堆栈大小测试（固定）							     */			
/*		  1.查看任务运行时堆栈大小，用于调试          			 */
/*参  数：无                          			   				 */
/*返回值：无                                       			     */
/*---------------------------------------------------------------*/
//void stack_task(void *pvParameters)
//{
//	TaskHandle_t TaskHandle;	
//	TaskStatus_t TaskStatus;
//	int i = 0;
//	while(1)
//	{
////		xEventGroupWaitBits((EventGroupHandle_t	)Event_Handle,		
////							(EventBits_t		)WIFI_CONECT|PING_MODE,
////							(BaseType_t			)pdFALSE,				
////							(BaseType_t			)pdTRUE,
////							(TickType_t			)portMAX_DELAY);
////		LED_On();
////		delay_ms(500);			//延时0.5s
////		LED_Off();
////		delay_ms(500);			//延时0.5s
//	
//		for(i = 0; i < 5; i++)
//		{
//			if (i == 0)
//			{
//				TaskHandle = Instruct_Task_Handler;			//根据任务名获取任务句柄。
//			}                                                                    
//			else if (i == 1)                                                     
//			{
//				TaskHandle = Runcode_Task_Handler;		//根据任务名获取任务句柄。
//			}
//			else if (i == 2)
//			{
//				TaskHandle = MQTT_RxTx_Task_Handler;	//根据任务名获取任务句柄。
//			}	
//			else if (i == 3)
//			{
//				TaskHandle = AHT20_Task_Handler;		//根据任务名获取任务句柄。
//			}	
//			else if (i == 4)
//			{
//				TaskHandle = DATA_TX_Task_Handler;		//根据任务名获取任务句柄。
//			}				
//			
//			//获取任务信息
//			vTaskGetInfo((TaskHandle_t	)TaskHandle, 	//任务句柄
//						 (TaskStatus_t*	)&TaskStatus, 	//任务信息结构体
//						 (BaseType_t	)pdTRUE,		//允许统计任务堆栈历史最小剩余大小
//						 (eTaskState	)eInvalid);		//函数自己获取任务运行壮态
//			//通过串口打印出指定任务的有关信息。
//			printf("任务名:                %s\r\n",TaskStatus.pcTaskName);
//			printf("任务编号:              %d\r\n",(int)TaskStatus.xTaskNumber);
//			printf("任务壮态:              %d\r\n",TaskStatus.eCurrentState);
//			printf("任务当前优先级:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
//			printf("任务基优先级:          %d\r\n",(int)TaskStatus.uxBasePriority);
//			printf("任务堆栈基地址:        %#x\r\n",(int)TaskStatus.pxStackBase);
//			printf("任务堆栈历史剩余最小值:%d\r\n",TaskStatus.usStackHighWaterMark);
//		}
//		delay_ms(10 * 1000);	    //延时10s

//	}
//}
