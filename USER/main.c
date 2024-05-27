/*------------------------------------------------------*/
/*                                                      */
/*            ����main��������ں���Դ�ļ�               */
/*                                                      */
/*------------------------------------------------------*/

#include "sys.h"
#include "delay.h"	     //������Ҫ��ͷ�ļ�
#include "usart1.h"      //������Ҫ��ͷ�ļ�
#include "usart2.h" 
#include "usart3.h" 
#include "uart4.h"      //������Ҫ��ͷ�ļ�
#include "timer3.h"      //������Ҫ��ͷ�ļ�
#include "timer4.h"      //������Ҫ��ͷ�ļ�
#include "stdio.h"      //������Ҫ��ͷ�ļ�
#include "FreeRTOS.h"	 //FreeRTOS����ͷ�ļ�
#include "semphr.h" 	 //�ź���
#include "queue.h"		 //����
#include "event_groups.h"//�¼���־��
#include "Thermostat.h"  
#include "wifi.h"	     //������Ҫ��ͷ�ļ�
#include "mqtt.h"        //������Ҫ��ͷ�ļ�
#include "control.h"     //������Ҫ��ͷ�ļ� ����ģ��������ݷ��͸�������
#include "switch.h"	     //������Ҫ��ͷ�ļ� LED
#include "AHT20.h"       //������Ҫ��ͷ�ļ� ������ʪ��
#include "stdlib.h"
#include "door.h" 
/*-------------------------------------------------------------*/
/*          	WIFI������ONENET���ã����ã�			      	   */
/*-------------------------------------------------------------*/
const char SSID[] 			 = "taizidejia";          //·��������
const char PASS[] 			 = "y1032979127.";    //·��������

const char PRODUCTID[] 	     = "394499";  	   //��ƷID
const char DEVICEID []	     = "661126800";    //�豸ID  
const char AUTHENTICATION[]  = "123456";       //��Ȩ��Ϣ  
const char DATA_TOPIC_NAME[] = "$dp";		   //topic��Onenet���ݵ��ϴ�topic�����øģ�
const char SERVER_IP[]	     = "kf30cdfc.ala.cn-hangzhou.emqxsl.cn";//��ŷ�����IP�����������øģ�
const int  SERVER_PORT 		 = 8883;		   //��ŷ������Ķ˿ںţ����øģ�

/*-------------------------------------------------------------*/
/*          ���������Լ�����ģ���ʼ״̬���ã����ã�		   	   */
/*-------------------------------------------------------------*/
	/* ��Ϣ�壺
	 *  {
	 *		"data_1":"value_1",
	 *		"data_2":"value_2"
	 *	}
	 *	��Ϣ��ʾ����
	 *	{"led1_flag":"LED1ON"}
	 */

const char *LED1_LABER  = "led1_flag";//LED1��ǩ�����͸�ONENET������������
const char *CMD_LED1ON  = "LED1ON";   //LED1��
const char *CMD_LED1OFF = "LED1OFF";  //LED1�ر�
char 	   *led1_flag   = "LED1OFF";  //LED1״̬����ʼ��Ϊ�ر�״̬

const char *LED2_LABER 	= "led2_flag";//LED2��ǩ
const char *CMD_LED2ON  = "LED2ON";   //LED2��
const char *CMD_LED2OFF = "LED2OFF";  //LED2�ر�
char 	   *led2_flag   = "LED2ON";   //LED2״̬����ʼ��Ϊ��״̬
/*-------------------------------------------------------------*/
/*               freerto����ͨ�ſ��ƣ��̶���			      	   */
/*-------------------------------------------------------------*/

/*	��ֵ�ź������                         
 *	���ã����ڿ���MQTT����崦��������MQTT���ݽ��շ��ͻ��崦�������з���
 *		  ����������յ���������ʱ�������ź���		 
 */
SemaphoreHandle_t BinarySemaphore;
	
/*	�¼���־��                         
 *	���ã���־WIFI���ӣ�PING����������ģʽ����wifi�Ƿ��������ӣ��Ƿ������ݣ��������Ƿ����� 
 *  ���壺1.�¼���־��λ1Ϊ0��λ0Ϊ1ʱ����0x03��0000 0001����wifi������������ʱλ0��λ1����ʱconnect���Ļ�δ���͡� 
 *		  2.�¼���־��λ1Ϊ1��λ0Ϊ1ʱ����0x03��0000 0011����connect���ķ��ͣ��������ӳɹ�����ʱλ1��λ1��PING��
 *			��������30s����ģʽ�������������������ݿ�ʼ�ϴ����豸Զ�̿��ƣ�LED���ƣ����ܿ����� 
 */
EventGroupHandle_t Event_Handle = NULL;     //�¼���־�飨λ0��WIFI����״̬ λ1��PING������2S���ٷ���ģʽ��
const int WIFI_CONECT = (0x01 << 0);        //�����¼������λ 0������������ģʽ��ֵ1��ʾ�Ѿ����ӣ�0��ʾδ����
const int PING_MODE   = (0x01 << 1);        //�����¼������λ 1��PING����������ģʽ��1��ʾ����30S����ģʽ��0��ʾδ�������ͻ���2S���ٷ���ģʽ

/*	���������ݷ�����Ϣ����                         
 *	���ã��������������ݷ��͵���������Ϣ����  
 */
QueueHandle_t Message_Queue;		 		//��Ϣ���о��  
const UBaseType_t MESSAGE_DATA_TX_NUM = 5;	//��Ϣ���������Ϣ��Ŀ  
const UBaseType_t MESSAGE_DATA_TX_LEN = 100;//��Ϣ���е�Ԫ��С����λΪ�ֽ�  


QueueHandle_t U1_xQueue,U3_xQueue,runcode_xQueue;


//ȫ�ֱ���
u8 door_flag=0;
int runingtime,Alltime=0;
u8 Runcode_flag=0;
/*-------------------------------------------------------------*/
/*               ������������������1�����ã�		      	   */
/*-------------------------------------------------------------*/

//��ʼ����
TaskHandle_t StartTask_Handler;
void my_start_task(void *pvParameters);

//Showhmi���񣬴�����
TaskHandle_t Showhmi_Task_Handler;
void my_showhmi_task(void *pvParameters);

//GetVI���񣬴�����
TaskHandle_t GetVI_Task_Handler;
void my_getVI_task(void *pvParameters);

//����������ָ������
TaskHandle_t Instruct_Task_Handler;
void my_instruct_task(void *pvParameters);

//WIFI����
TaskHandle_t WIFI_Task_Handler;
void wifi_task(void *pvParameters);

//���м��ȳ�������
TaskHandle_t Runcode_Task_Handler;
void my_runcode_task(void *pvParameters);

//����¯�Ŷ�������
TaskHandle_t Door_Task_Handler;
void my_door_task(void *pvParameters);

//��ջ�������
TaskHandle_t Stack_Task_Handler;
void stack_task(void *pvParameters);

/*---------------------------------------------------------------*/
/*��������int main()                                             */
/*��  �ܣ�������							                         */
/*		  1.��ʼ��������ģ��  				     				 */
/*		  2.������ʼ�����ڿ�ʼ�����ﴴ��������������           */
/*		  3.�����������				       			 		     */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*---------------------------------------------------------------*/
int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);		//����ϵͳ�ж����ȼ�����4
	delay_init();	      							 	//��ʱ������ʼ��
	usart1_init(115200);  								 //����1���ܳ�ʼ����������115200���봮������/������ͨ��
	usart2_init(4800);      							 //����2���ܳ�ʼ����������4800�����ѹ�������ģ��ͨ��
	usart3_init(9600);   								//����3���ܳ�ʼ����������9600�����¿���ͨ��
	usart4_init(115200);  								 //����4���ܳ�ʼ����������115200��wifiͨ��	
	tim4_init(500,7200);   								//TIM4��ʼ������ʱʱ�� 500*7200*1000/72000000 = 50ms	
	
	Switch_Init();    									//��ʼ���ⲿ���� 
//	IoT_parameter_init();  								//��ʼ��MQTT�������Ĳ���	
	Init_I2C_Sensor_Port(); 							//IIC��ʼ��
	delay_ms(500);
	
	if((AHT20_Read_Status()&0x18)!=0x18)
	{
		AHT20_Start_Init();
		Delay_1ms(10);
	}	
	
		YELLOW_LED = 0;
		RED_LED    = 1;
		GREEN_LED  = 1;

	//������ʼ����
	xTaskCreate((TaskFunction_t	) my_start_task,		//������
			    (const char* 	)"my_start_task",		//��������
				(uint16_t 		) 128,				  	//�����ջ��С
				(void* 		  	) NULL,				 	//���ݸ��������Ĳ���
				(UBaseType_t 	) 1, 				  	//�������ȼ�
				(TaskHandle_t*  ) &StartTask_Handler);	//������ƿ� 
			
	vTaskStartScheduler();  							//�����������
}

/*---------------------------------------------------------------*/
/*��������void my_start_task(void *pvParameters)                 */
/*��  �ܣ���ʼ�������ã�							             */
/*		  1.�����ź�������Ϣ���е�����ͨ�ŷ�ʽ   				     */
/*		  2.������������       			 						 */
/*		  3.ɾ������       			 		    				 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*---------------------------------------------------------------*/
void my_start_task(void *pvParameters)
{
	//taskENTER_CRITICAL(); //�����ٽ���
	
	//�������У��Ӵ���1�ж϶�ȡָ������
	 U1_xQueue = xQueueCreate(1, 70 );

	//�������У��Ӵ���3�ж϶�ȡ��������
	 U3_xQueue = xQueueCreate(1, 30 );	
	
	//�������У����ͳ��������runcode����
	 runcode_xQueue = xQueueCreate(1, 80 );
	
	//������ֵ�ź���
	BinarySemaphore = xSemaphoreCreateBinary();	
	//�¼���־�飬���ڱ�־wifi����״̬�Լ�ping����״̬
	Event_Handle = xEventGroupCreate(); 
	//������������Ϣ����Ϣ����
	//Message_Queue = xQueueCreate(MESSAGE_DATA_TX_NUM, MESSAGE_DATA_TX_LEN); 
	
	//���񴴽�����������1.������ 			2.�������� 			3.�����ջ��С 3.���ݸ��������Ĳ��� 4.�������ȼ� 5.������ƿ�
	//����WIFI����
    //xTaskCreate(wifi_task, 				"wifi_task", 				128, NULL, 1, &WIFI_Task_Handler); 			
			
	//����ÿ�뷢��ϵͳ״̬���� ����������
    xTaskCreate(my_showhmi_task, 			"my_showhmi_task", 			128, NULL, 1, &Showhmi_Task_Handler);
	
	//����ÿ�뷢��V��I�������¶ȡ�ʪ������ ����������
    xTaskCreate(my_getVI_task, 			"my_getVI_task", 			128, NULL, 1, &GetVI_Task_Handler);
	
	
	//��������ָ������
	xTaskCreate(my_instruct_task, 			"my_instruct_task", 		128, NULL, 3, &Instruct_Task_Handler);
	
//	//�������г�������
//	xTaskCreate(my_runcode_task, 			"my_runcode_task", 			128*2, NULL, 1, &Runcode_Task_Handler);
	
	//����¯�Ŷ�������
	xTaskCreate(my_door_task, 			"my_door_task", 			128, NULL, 1, &Door_Task_Handler);
	
	//�����������
	//xTaskCreate(stack_task,    			    "my_stack_task",         128, NULL, 1, &Stack_Task_Handler);	
		
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
//	taskEXIT_CRITICAL();            //�˳��ٽ���
}


void rehoting(int rehottem,int abstem,int i)
{
	//��ͣ����modbusָ���
	//����Ԥ��ָ�дpno=0����ģʽ��дsv�¶�ֵrehottem
	Send_Thermostat('w', 0x1b, 0 );					//д����
	Send_Thermostat('w', 0x2b, 0 );  				//д����ģʽ
	Send_Thermostat('w', 0x50, rehottem*10 );		//д�����¶�


	
	
	//printf("Ԥ�������¿�ʼ�������¶ȣ�%d��\r\n",rehottem);
	
	
	//printf("���е�%d��,�ȴ��¶ȵ��趨ֵ%d�ȵ� %d���� \r\n",i,rehottem,abstem);
	
	while(abs(PVTem-rehottem) > abstem )
	{
		//printf("���е�%d��,�ȴ��¶ȵ��趨ֵ%d�ȵ� %d���� \r\n",i,rehottem,abstem);
		Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�
		delay_ms(1000);
	}
	door_flag=1;
	updoor(1);
	delay_ms(2000);
		
}


/*auto_doordown(u8 allstep,int atuodoor_downTem��¯�ſ����¶ȣ�,u8 stoptime(ms),int rehoting_doorrun_alltime)  */
void auto_doordown(u8 allstep,int atuodoor_downTem,int stoptime,int rehoting_doorrun_alltime)
{
	int i=0;
	for(i=0 ; i<allstep ;i++)
		{
			delay_ms(stoptime);
			//ʵ���¶���¯���²�С��ָ���²�ʱ���ȴ�
			while( abs(PVTem-atuodoor_downTem) < abs(atuodoor_downTem-400)/allstep*(i+1) )
			{
				Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�
		
				delay_ms(1000);
			
			}
	
			//δ��������λ���ؿ�����
			while(rehoting_doorrun_alltime/allstep--)
			{
				if(LIM2)break;
				DOWN=0;
				UP = 1; //�͵�ƽ��Ч.	
				delay_ms(100); 
			}
			
			DOWN=1;
			UP = 1; //�͵�ƽ��Ч
		}
}

void heating(u8 run_p_num,u8 all_p_num,float starttem,float endtem,u16 time)
{
	TickType_t xLastWakeTime;
	int i=0,j=0;
	float pvtem,prograss_time;
	float v=(endtem - starttem)/time;
	prograss_time=time;
	//while (Send_Thermostat('w',0x1b,0)){ break;}			//1b-Srun�� 0-run 1-stop 2-hold
    Send_Thermostat('w',0x1b,0);			//1b-Srun�� 0-run 1-stop 2-hold
	delay_ms(1000);
	Send_Thermostat('w', 0x50, starttem*10 );  				
	
    xLastWakeTime = xTaskGetTickCount();
	printf("zhuangtai.val=2\xff\xff\xff");		//0-���� 1-Ԥ��� 2-���� 3-����
	//printf("page yunxing\xff\xff\xff");
	
	while(time--)
	{
		
		pvtem = starttem +	v*i + 1;
		//printf("��ʼ�¶ȣ�%f �������¶ȣ�%f ����ǰ�趨�¶ȣ�%f  �������ʣ�%f ��/s \r\n",starttem,endtem,pvtem,v);
		Send_Thermostat('w', 0x50, pvtem*10 );
		vTaskDelayUntil(&xLastWakeTime,1000); 
		i=i+1;
		runingtime = runingtime + 1;
		
		
		printf("zhuangtai.val=2\xff\xff\xff");		//0-���� 1-Ԥ��� 2-���� 3-����
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
	
	//�����¶�δ��Ԥ���¶�+-3�ڣ�����ȴ�
	if( PVTem < endtem-3 || PVTem > endtem+3 ){		
		printf("δ�ﵽԤ��ֵ������ȴ�++++++++++++++++++++++++");
					
	}
	j=0;
	while(abs(PVTem-endtem)>3 && endtem!=0){		
		Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�
		j=j+1;
		delay_ms(1000);
		printf("�ȴ�%d  ,��ǰ%d�� ʱ��%ds++++++++++++++++++++++++",(int)endtem,PVTem,j);					
	}	
		
	printf("�ﵽԤ��ֵ�������ȴ�++++++++++++++++++++++++");	
	
}

/*---------------------------------------------------------------*/
/*��������void my_runcode_task(void *pvParameters)   		    */
/*��  �ܣ����г�������       							 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ����                                                       */
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
	
	#define rehoting_doorrun_alltime 25*10			//��λ��0.1s
	while(1)
	{
		//printf("��������------Runcode------\r\n");
		
		if (pdPASS == xQueueReceive( runcode_xQueue,&runcode,portMAX_DELAY ) )    	//������
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
			rehottime			=	runcode[39]*6 ;	//*60 ���ԣ��޸�Ϊ*6
			printf("fan  = %d\r\n",fan);
			printf("atuodoor  = %d\r\n",atuodoor);
			printf("atuodoor_downTem  = %d\r\n",atuodoor_downTem);
			printf("rehot  = %d\r\n",rehot);
			printf("rehottem  = %d\r\n",rehottem);
			printf("rehottime  = %d\r\n",rehottime);
			
			
			if(!LIM1&&!rehot)
			{
				printf("page yunxing\xff\xff\xff");		//��ת�����н���
				printf("page yunxing\xff\xff\xff");		//��ת�����н���
				printf("page guanbilumen\xff\xff\xff");
				Runcode_flag=0;	
				vTaskDelete(NULL);
			}
				
			printf("page yunxing\xff\xff\xff");		//��ת�����н���
			printf("page yunxing\xff\xff\xff");		//��ת�����н���
			printf("vis b3,0\xff\xff\xff");			//���� ��ʼ ��ť
			printf("vis up,0\xff\xff\xff");			//���� ���� ��ť
			printf("vis down,0\xff\xff\xff");
			printf("jindu.val=0\xff\xff\xff");
			//����д��Program_data[tem][time:s]
			printf("\r\n");	
				
				
			
			if(fan==0)
			{
				FAN_DOWN=0;
			}
			
			
//			/*debug*/
//				//��ȫ�ر�¯��
//				door_flag=1;
//				updoor(0);
//				delay_ms(1000);
//			/**/
			
		//	printf("��ʼ����\r\n");
			
		{
			YELLOW_LED = 1;
			RED_LED    = 1;
			GREEN_LED  = 0;
		}
			vTaskSuspend(Showhmi_Task_Handler); //LED2�����ɾ���̬������̬��תΪ����̬��LED2�������ֹͣ��  
			//��������Ԥ��--����ȫ�ر����½�
			if(rehot)
				{
					printf("jindu.val=0\xff\xff\xff");
					printf("zhuangtai.val=1\xff\xff\xff");		//0-���� 1-Ԥ��� 2-���� 3-����
					printf("page yunxing\xff\xff\xff");
					printf("zhuangtai.val=1\xff\xff\xff");		//0-���� 1-Ԥ��� 2-���� 3-����
					printf("page yunxing\xff\xff\xff");
					//�½������
					door_flag=2;
					downdoor(0);
					delay_ms(2000);
					
					//�¶�Ԥ�ȵ�ָ��ֵ
					printf("��ʼԤ����\r\n");
					
					//������������ָ��λ�ã�ÿ��2s
			  
					//��һ�����У��²�200������
					rehoting(rehottem+3,200,1);
					
					//�ڶ������У��²�100������
					rehoting(rehottem+2,100,2);
					
					//���������У��²�10������
					rehoting(rehottem+1,10,3);

					//��¯���쳣(��ȫ�ر�)���½�2s
					if(LIM1)
					{
						//�½���ָ��λ��
						DOWN=0;
						UP = 1; //�͵�ƽ��Ч
						delay_ms(2000);   
						DOWN=1;
						UP = 1; //�͵�ƽ��Ч
					}
					
					//���ָ��ʱ��
					while(rehottime--)
					{
						Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�
						delay_ms(1000); 			//��ɿ�ʼ��ʱ����λ������
						//printf("Ԥ����ʣ��ʱ�䣺%ds\r\n",rehottime);
						printf("zhuangtai.val=1\xff\xff\xff");		//0-���� 1-Ԥ��� 2-���� 3-����
						printf("jindu.val=0\xff\xff\xff");
						printf("shengyushi.val=%d\xff\xff\xff",rehottime/60/60);	
						printf("shengyufen.val=%d\xff\xff\xff",rehottime%3600/60);	
						printf("shengyumiao.val=%d\xff\xff\xff",rehottime%3600%60);
						
						
					}
					
					
					//��ɽ�������������
					for(i=0;i<7;i++)
					{
						vTaskResume(Showhmi_Task_Handler);  //Showhmi_Task�ɹ���̬תΪ����̬��Showhmi_Task��������
						
						door_flag=1;
						updoor(2);
						delay_ms(4000);	
						vTaskSuspend(Showhmi_Task_Handler); //Showhmi_Task�ɾ���̬������̬��תΪ����̬��Showhmi_Task�ɹ���ֹͣ��
						
					}

					//������ȫ�ر�¯��
					door_flag=1;
					updoor(0);
				}
				
				
				
				
				
				
				
//��������				
printf("yunxing.tm0.en=1\xff\xff\xff");
printf("yunxing.tm0.en=1\xff\xff\xff");				
			
	delay_ms(500);
	//Ѱ�ҵ�ǰ�¶ȶ�Ӧ������¶ȶ�-�¶ȵ�
	nowtem=PVTem;
	
	//����������ʱ��
	Alltime=0 ;
	for(j=0;j<16;j++)
	{
		Alltime=Alltime + Program_data[j][1] ;
	}
	printf("--------------��ʱ��%ds\r\n",Alltime);	
	
	//���������ܶ���
	all_p_num=0 ;
	for(j=0;j<16;j++)
	{
		if(Program_data[j][0]==0){all_p_num=j-1;break;} ;
	}
	
	printf("--------------�ܶ���: %d\r\n",all_p_num);	
					
	//Ѱ�Ҽ��ȶ����
	for(i=0 ;i<17 ;i++)
	{
		//printf("��%d�μ��\r\n",i);
		//��ǰ�¶�Ϊ���¶�
		if(nowtem==Program_data[i][0])
		{
			progress_num = i+2;	
			printf("��ǰ�¶�%d�������%d��*******���¶�\n",nowtem,progress_num+1);	
			progress_time=Program_data[progress_num][1] ;					//��ȡ��ǰ����ʱ��s
			//����������ʱ��
			runingtime=0;
			for(j=0;j<=i;j++){
				runingtime=runingtime + Program_data[j][1] ;
			}

			printf("��ʱ��%ds,��ǰ������%ds\xff\xff\xff",Alltime,runingtime);	
			
			heating(i,all_p_num,nowtem,Program_data[i+2][0],Program_data[i+1][1]);
			break;
				
		}
			//��ǰ�¶�Ϊ���¶� iΪĿ���¶ȵ������
		if(nowtem<Program_data[i+1][0]&&nowtem>Program_data[i][0])
		{
					
			progress_num = i;	
			//printf("��ǰ�¶�%d�������%d��*******���¶�\n",nowtem,progress_num);
			percent=1-(float)(nowtem-Program_data[i][0])/(Program_data[i+1][0]-Program_data[i][0]);	
			
			//����������ʱ��
			runingtime=0;
			for(j=0;j<i;j++){
				runingtime=runingtime + Program_data[j][1] ;
			}
				
			runingtime = runingtime + Program_data[i][1] * (1-percent);
		//	printf("��ʱ��%ds,��ǰ������%ds\xff\xff\xff",Alltime,runingtime);	
		//	printf("jindu.val=%d\xff\xff\xff",runingtime*100/Alltime);	
				//printf("��ʱ��%ds\xff\xff\xff",alltime);	
				
		//	printf("��ǰ����ν���ʱ��:  %2f \r\n",percent*100);
		//	printf("��ǰ�¶�%d�������%d��\n",nowtem,progress_num);	
		//	printf("Ŀ���¶�%d����ʱ����%d��ʣ��ʱ����%fs\n",Program_data[i+1][0],Program_data[i][1],Program_data[i][1]*percent);
				
			progress_time=Program_data[i][1]*percent;//��ȡ��ǰ����ʱ��
			
			heating(i,all_p_num,nowtem,Program_data[i+1][0],(int)(Program_data[i][1]*percent));	 //��ǰ�¶�   Ŀ���¶�   ʱ��s
			
			break;
		}	
					
	}	

		printf("ʣ����ȶ�\n");	
		//ʣ����ȶ�
		i=i+1;	
		//����0�����˳�
		if(Program_data[i][0]==0){
			
			i=17;
			//printf("Program_data[i+1][0]==0--------����----------\r\n");
		}
		for(i=i;i<17 ;i++){
			
			progress_time=Program_data[i][1];		//��ȡ��ǰ����ʱ��
			//printf("�����%d�Σ���ǰ�¶�%d��Ŀ���¶�%d����ʱ����%ds\n",
			//	i,Program_data[i][0],Program_data[i+1][0],progress_time);
			heating(i,all_p_num,Program_data[i][0],Program_data[i+1][0],progress_time);		
			
			//����0�����˳�
			if(Program_data[i+1][0]==0){
				
				i=20 ;
				//printf("Program_data[i+1][0]==0--------����----------\r\n");
			}	
		 }

		 //�ս����+1
		 printf("wepo shaojiecishu.val,272\xff\xff\xff");
		 printf("shaojiecishu.val=shaojiecishu.val+1\xff\xff\xff");
		 printf("wepo shaojiecishu.val,272\xff\xff\xff");
		
		 //�ر�����				
		printf("yunxing.tm0.en=0\xff\xff\xff");
		printf("yunxing.tm0.en=0\xff\xff\xff");	

		 
			if(atuodoor)
			{
				//printf("atuodoor-------atuodoor_downTem=%d\r\n",atuodoor_downTem);
				
				//�¶ȸ���1000��������
				while(PVTem > 1000)
				{
					Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�
					delay_ms(1000);
					//printf("�ȴ��¶Ƚ���1000����\r\n");
				}	
				
				//��һ����϶���½�1s
				//printf("�¶Ƚ���1000���£���һ����϶\r\n");
				delay_ms(1000);
				door_flag=2;
				downdoor(2);
				
				//��һ����϶���ȴ��¶ȵ����趨�Ŀ����¶�
				while(PVTem > atuodoor_downTem)
				{
					Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�
					delay_ms(1000);
					//printf("�ȴ��¶Ƚ���%d\r\n",atuodoor_downTem);
				}
				
				
				for(i=0;i<5;i++)
				{
					//printf("��%d���½�\r\n",i+1);
					
					delay_ms(5000);
					door_flag=2;
					downdoor(3);
					//printf("�ȴ��¶Ƚ���%d\r\n",800-i*100);
					
					while (PVTem > 800-i*100)
					{
						Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�
						delay_ms(1000);
						//printf("�ȴ��¶Ƚ���%d\r\n",800-i*100);
					}
		
				}

				//������ȫ��¯��
				door_flag=2;
				downdoor(100);
			}
		
		Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�
		Send_Thermostat('w',0x1b,1);				//1b-Srun�� 0-run 1-stop 2-hold	
		delay_ms(1000); 
		//printf("-------------����----------\r\n");
		//delay_ms(1000); 
		
	                               
		vTaskResume(Showhmi_Task_Handler);  //Showhmi_Task�ɹ���̬תΪ����̬��Showhmi_Task��������
		
		
	}
		

	
	//ɾ��runcode����
		printf("zhuangtai.val=0\xff\xff\xff");		//0-���� 1-Ԥ��� 2-���� 3-����
		printf("page yunxing\xff\xff\xff");
		printf("zhuangtai.val=0\xff\xff\xff");		//0-���� 1-Ԥ��� 2-���� 3-����
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
	
	
		printf("vis b3,1\xff\xff\xff");			//��ʾ ��ʼ ��ť
		printf("vis b3,1\xff\xff\xff");			//��ʾ ��ʼ ��ť
		printf("vis up,1\xff\xff\xff");			//��ʾ ���� ��ť
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
	
		printf("��������------Runcode------\r\n");
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
		//��ȡ��ʪ��
		getCT();
		delay_ms(1000);	
		//��ȡ��ѹ����
		getVI();

	}
}

/*---------------------------------------------------------------*/
/*��������void my_showhmi_task(void *pvParameters)   		    */
/*��  �ܣ���������ʾϵͳ״̬����       							 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ����                                                       */
/*---------------------------------------------------------------*/
void my_showhmi_task(void *pvParameters)	
{
	
	//�¿�����ʼ��
	delay_ms(5000);	  
	Thermostat_init();
	
	while(1)
	{
		//printf("----------my_showhmi_task----------\r\n");
		delay_ms(1000);	 
		
		Send_Thermostat('r',0x4a,0);              	//4a-PV�������¶�

		if(Srun==0){Send_Thermostat('w',0x1b,1);}				//1b-Srun�� 0-run 1-stop 2-hold		
		
	}
}

/*---------------------------------------------------------------*/
/*��������void my_instruct_task(void *pvParameters)   		     */
/*��  �ܣ�������ָ�����״̬����       							 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ���������� U1_xQueue                                                 */
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
		if (pdPASS == xQueueReceive( U1_xQueue,&instruct_RxBuff,portMAX_DELAY ))    	//������
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
			//printf("����ָ��%02X��\r\n",instruct);
			if(instruct == 0x01)
			{	
				
				//����runcode����	
				if(Runcode_flag==0)
				{
					xTaskCreate(my_runcode_task, "my_runcode_task", 128*2, NULL, 4, &Runcode_Task_Handler);
				}else{printf("�������񱻴������޷��ٴ�ִ��------Runcode------\r\n");}
				
				Runcode_flag=1;
				//����������� Program_data[17][2]:[�¶�][ʱ��]
				for(i=2;i<17;i+=2)
				{
					TEM_data[(i-2)/2]		=(instruct_RxBuff[i+1]<<8|instruct_RxBuff[i]);          //�¶ȣ����϶�
					V_data[(i-2)/2]		=instruct_RxBuff[i+1+16]<<8|instruct_RxBuff[i+16];		//���ʣ����϶�/����
					H_data[(i-2)/2]		=instruct_RxBuff[i+1+32]<<8|instruct_RxBuff[i+32];		//����ʱ�䣺����
				}	
				
				myProgram_data[0][0]	= 50;													//��һ���¶ȵ㣺50���϶�
				myProgram_data[0][1]	= (TEM_data[0]-50)/V_data[0];
				
				for(i=0;i<8;i+=1)
				{
					myProgram_data[2*i+1][0]	= TEM_data[i];						//�¶�
					myProgram_data[2*i+1][1]	= H_data[i];
					
					myProgram_data[2*i+2][0]	= TEM_data[i];						//�¶�
					myProgram_data[2*i+2][1]	= abs(TEM_data[i+1]-TEM_data[i])/V_data[i+1];
					if(i>=7){myProgram_data[2*i+2][1]	= 0;}
				}

//				//���ԣ���ʾ���ɳ�����������
				for(i=0;i<17;i+=1)
				{
//						printf("Program_data%d=%d   %d\r\n",i,Program_data[i][0],Program_data[i][1]);	
//						//if(Program_data[i][1]==0){i=50;}
					runcode[2*i] = myProgram_data[i][0];
					//printf("runcode[%d]=%d \r\n",2*i,runcode[2*i] );
					runcode[2*i+1] = myProgram_data[i][1];
					//printf("runcode[%d]=%d  \r\n",2*i+1,runcode[2*i+1]);
				}
	
				
				//�߼����ò���д��
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
				
				//д���и�runcode����
				
				xQueueSendToBack( runcode_xQueue,&runcode,30);	//д����
					
			}
			else if(instruct == 0x10)
			{
				door_flag=1;      //0 ֹͣ��1 up ,2 down	
			}
			else if(instruct == 0x11)
			{
		
				door_flag=0;      //0 ֹͣ��1 up ,2 down
				DOWN=1;
				UP = 1; /*�͵�ƽ��Ч*/
				printf("up.pic=0\xff\xff\xff");
				printf("down.pic=2\xff\xff\xff");
					
			}	
			else if(instruct == 0x12)
			{	
				door_flag=2;      //0 ֹͣ��1 up ,2 down
			}	

			else if(instruct == 0x13)
			{	
				     
			}			
			else if(instruct == 0x14)
			{	
				
				if(Runcode_flag){
					//ɾ��runcode����	
					door_flag=0;      //0 ֹͣ��1 up ,2 down
					vTaskDelete(Runcode_Task_Handler);
					
					Send_Thermostat('w',0x1b,1);		//1b-Srun�� 0-run 1-stop 2-hold	
					vTaskResume(Showhmi_Task_Handler);  //Showhmi_Task�ɹ���̬תΪ����̬��Showhmi_Task��������
					Runcode_flag=0;
					printf("zhuangtai.val=0\xff\xff\xff");		//0-���� 1-Ԥ��� 2-���� 3-����
					printf("page yunxing\xff\xff\xff");
					{
						YELLOW_LED = 0;
						RED_LED    = 1;
						GREEN_LED  = 1;
					}
					
					printf("vis b3,1\xff\xff\xff");			//��ʾ ��ʼ ��ť				
					printf("vis b3,1\xff\xff\xff");			//��ʾ ��ʼ ��ť
					
					printf("vis up,1\xff\xff\xff");			//��ʾ ���� ��ť
					printf("vis down,1\xff\xff\xff");
					printf("����ɾ��------Runcode------\r\n");
				}else{printf("�������ɾ��------Runcode------\r\n");}
				
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


//  vTaskSuspend(Led2_Task_Handler); //LED2�����ɾ���̬������̬��תΪ����̬��LED2�������ֹͣ��                                         
//	vTaskResume(Led2_Task_Handler);  //LED2�����ɹ���̬תΪ����̬��LED2��������



/*---------------------------------------------------------------*/
/*��������void wifi_task(void *pvParameters)                     */
/*��  �ܣ�WIFI���񣨹̶���										 */
/*		  1.����wifi�Լ��Ʒ�����       							 */
/*		  2.��������        									     */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
/*��  ����1.����������ǰ�رշ���ping���Ķ�ʱ��3������¼���־λ	 */
/*		  2.�����������ӣ��׳��¼���־�������Լ����������̬		 */
/*		  3.����������wifi�ѶϿ�������¼���־������ִ�б��������� */
/*			����	 											 */
/*---------------------------------------------------------------*/
void wifi_task(void *pvParameters)
{
	while(1)
	{ 
		printf("��Ҫ���ӷ�����\r\n");                 
		TIM_Cmd(TIM4, DISABLE);                       //�ر�TIM4 
		TIM_Cmd(TIM3, DISABLE);                       //�ر�TIM3
		xEventGroupClearBits(Event_Handle, PING_MODE);//�رշ���PING���Ķ�ʱ��3������¼���־λ
		WiFi_RxCounter = 0;                           //WiFi������������������                        
		memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE);     //���WiFi���ջ����� 
		if(WiFi_Connect_IoTServer() == 0)			  //���WiFi�����Ʒ�������������0����ʾ��ȷ������if
		{   			     
			printf("MQTT���������ӳɹ�\r\n");            
			WiFi_RxCounter = 0;                       //WiFi������������������                        
			memset(WiFi_RX_BUF, 0, WiFi_RXBUFF_SIZE); //���WiFi���ջ����� 
			MQTT_Buff_Init();                         //��ʼ�����ͻ�����  
			
			xEventGroupSetBits(Event_Handle, WIFI_CONECT);  //�����������ӣ��׳��¼���־ 
			vTaskSuspend(NULL);	    						//�����������ӣ������Լ����������̬�������ɹ���תΪ����̬ʱ�������ִ����ȥ��
			xEventGroupClearBits(Event_Handle, WIFI_CONECT);//����������wifi�ѶϿ�������¼���־������ִ�б������������� 
			xEventGroupClearBits(Event_Handle, PING_MODE);  //�رշ���PING���Ķ�ʱ��3������¼���־λ
		}
		
		delay_ms(1000);	    //��ʱ1s
	}
}

/*---------------------------------------------------------------*/
/*��������void stack_task(void *pvParameters)                    */
/*��  �ܣ������ջ��С���ԣ��̶���							     */			
/*		  1.�鿴��������ʱ��ջ��С�����ڵ���          			 */
/*��  ������                          			   				 */
/*����ֵ����                                       			     */
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
////		delay_ms(500);			//��ʱ0.5s
////		LED_Off();
////		delay_ms(500);			//��ʱ0.5s
//	
//		for(i = 0; i < 5; i++)
//		{
//			if (i == 0)
//			{
//				TaskHandle = Instruct_Task_Handler;			//������������ȡ��������
//			}                                                                    
//			else if (i == 1)                                                     
//			{
//				TaskHandle = Runcode_Task_Handler;		//������������ȡ��������
//			}
//			else if (i == 2)
//			{
//				TaskHandle = MQTT_RxTx_Task_Handler;	//������������ȡ��������
//			}	
//			else if (i == 3)
//			{
//				TaskHandle = AHT20_Task_Handler;		//������������ȡ��������
//			}	
//			else if (i == 4)
//			{
//				TaskHandle = DATA_TX_Task_Handler;		//������������ȡ��������
//			}				
//			
//			//��ȡ������Ϣ
//			vTaskGetInfo((TaskHandle_t	)TaskHandle, 	//������
//						 (TaskStatus_t*	)&TaskStatus, 	//������Ϣ�ṹ��
//						 (BaseType_t	)pdTRUE,		//����ͳ�������ջ��ʷ��Сʣ���С
//						 (eTaskState	)eInvalid);		//�����Լ���ȡ��������׳̬
//			//ͨ�����ڴ�ӡ��ָ��������й���Ϣ��
//			printf("������:                %s\r\n",TaskStatus.pcTaskName);
//			printf("������:              %d\r\n",(int)TaskStatus.xTaskNumber);
//			printf("����׳̬:              %d\r\n",TaskStatus.eCurrentState);
//			printf("����ǰ���ȼ�:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
//			printf("��������ȼ�:          %d\r\n",(int)TaskStatus.uxBasePriority);
//			printf("�����ջ����ַ:        %#x\r\n",(int)TaskStatus.pxStackBase);
//			printf("�����ջ��ʷʣ����Сֵ:%d\r\n",TaskStatus.usStackHighWaterMark);
//		}
//		delay_ms(10 * 1000);	    //��ʱ10s

//	}
//}
