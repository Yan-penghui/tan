
/*-------------------------------------------------*/
/*                                                 */
/*   		     ���485�¿���                      */
/*                                                 */
/*-------------------------------------------------*/

// Ӳ�����ӣ�
// usart3ת485

#include "stm32f10x.h"  //������Ҫ��ͷ�ļ�
#include "Thermostat.h"      //������Ҫ��ͷ�ļ� 
#include "delay.h"      //������Ҫ��ͷ�ļ�
#include "usart3.h"     //������Ҫ��ͷ�ļ�
#include "switch.h"
#include "stdio.h"      //������Ҫ��ͷ�ļ�
#include "string.h"	    //������Ҫ��ͷ�ļ�
#include "FreeRTOS.h"	 //FreeRTOS����ͷ�ļ�
#include "queue.h"		 //��
#include "task.h"


extern QueueHandle_t U3_xQueue;

int PVTem = 0 ,SVTem = 0,RunTime = 0 ,Srun = 0,Pno=0, Step = 0, Output=0 , Value=0;

/*-----------------------------------------------------------*/
/*	��������Thermostat_init��ʼ�� �¿���                      */
/*	��  ����												 	 */
/*	ͨѶģʽ			AFC: 0 - ��׼modbus						 */
/*	����			lnp��6 - B�ȵ�ż                         */ 
/*	������������		SPr��50 - ��/����                        */ 
/*	״̬����			Srun: 0-���� ��1-ֹͣ ��2-����hold		 */
/*-----------------------------------------------------------*/  
void Thermostat_init(void)	   
{    	
	Send_Thermostat('w',0x0b,6);			//lnp�Ĵ�����ַ0x0b
	 
	Send_Thermostat('w',0x2a,50*10); 		//SPr�Ĵ�����ַ0x2a
	
	//Send_Thermostat('w',0x50,50*10); 		//SP1�Ĵ�����ַ0x50
		 
	Send_Thermostat('w',0x1b,1);			//1b-Srun�� 0-run 1-stop 2-hold

	printf("�¿���������ʼ���ɹ�\r\n");
}


/*-------------------------------------------------*/
/*�����������ڷ����ֽ�                          	   */
/*��  ����Byte������                                */
/*��  ����usart�����ں�						       */
/*����ֵ��						                   */
/*-------------------------------------------------*/

//��������
void Serial_SendByte(u8 Byte,int usart)
{	
	//�������ݵĺ���
	if(usart==1){
	USART_SendData(USART1,Byte);}
	else if(usart==3){
	USART_SendData(USART3,Byte);}
	
	
	//���ĳ���Ĵ������жϱ�־λ
	while (USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);
	/*
	��������ݷ��͵�ʱ��,TXE��1;
	�´η��������Զ���0,���Բ����ֶ�����жϱ�־λ.
	
	TXE:�������ݼĴ���:
	��TDR�Ĵ����е����ݱ�Ӳ��ת�Ƶ���λ�Ĵ�����ʱ�򣬸�λ��Ӳ����λ��
	���USART_CR1�Ĵ����е�TXEIEΪ1��������жϡ���USART DR��д����������λ���㡣
	0:���ݻ�û�б�ת�Ƶ���λ�Ĵ���
	1:�����Ѿ���ת�Ƶ���λ�Ĵ�����
	ע��:��������������ʹ�ø�λ��
	*/
}

/*-------------------------------------------------*/
/*�����������ڷ��Ͷ���ֽ�                          */
/*��  ����Array��������                             */
/*��  ����Length���ֽ���                            */
/*��  ����usart�����ں�						       */
/*����ֵ��						                   */
/*-------------------------------------------------*/

void Serial_SendArray(u8 *Array, int Length,int usart){
	
	int i;
	//delay_us(800);
	for(i=0;i<Length;i++)
	{
		Serial_SendByte(Array[i],usart);
		//delay_us(500);               //�����ʣ��趨����ʱ����
	}
}
 

/*-------------------------------------------------*/
/*���������¿�����������ָ��                         */
/*��  ����operate��W/R  д/��                       */
/*��  ����name���Ĵ�����ַ        					 */
/*��  ����w_data��д������ֵ       					  */
/*����ֵ��0����ȷ   ����������                     */
/*-------------------------------------------------*/
int Send_Thermostat(char operate,u8 name,u16 w_data)
{
	u8 send_data[8];
	u8 re_data[30];
	u16 sendCRC=0;
	int i=5;		//��ʱ����
	
	send_data[0] = 0x81;               		  	//�Ǳ��ַ80+1
	send_data[1] = 0x81;
	if(operate=='w')
	{
		send_data[2] = 0x43;            		 	//дָ��43
		send_data[4] = w_data;               		//д������(��λ��
		send_data[5] = w_data>>8;               	//д������(��λ��
	}
	if(operate=='r')
	{
		send_data[2] = 0x52;					//��ָ��52
		send_data[4] = 0x00;					//�����ݳ��ȣ���λ����0x00��
		send_data[5] = 0x00;					//�����ݳ��ȣ�ͳһ����Ϊ1��
	}
	
	send_data[3] = name;

	//У���
	if(operate=='r'){sendCRC = name*256+82+1;}
	if(operate=='w'){sendCRC = name*256+67+w_data +1;}
	send_data[7] = sendCRC>>8;					//��8λ
    send_data[6] = sendCRC;						//��8λ
	
	//����ָ�����ݣ�����ʧ�����Դ�����5
	
	while(i)
	{
		
		RS485=1;													//ʹ�ܷ���1ʹ�ܽ���0
		Thermostat_RxCounter = 0;                           		//Thermostat������������������                        
		memset(Thermostat_RX_BUF, 0, Thermostat_RXBUFF_SIZE);     	//���Thermostat���ջ����� 
		
		Serial_SendArray(send_data, 9,3);							//����ָ������Serial_SendArray(data, �ּ�����,����)
		
		RS485=0;													//ʹ�ܷ���1ʹ�ܽ���0
	
	
		//50ms�ڽ��յ���ȷ����ֵ
		if (pdPASS == xQueueReceive( U3_xQueue,&re_data,200 ))
		{ 
			u16 modbusCRC,myCRC;
			delay_ms(100);
			//����������ݵ�У���
			modbusCRC =	((Thermostat_RX_BUF[0] + Thermostat_RX_BUF[1]*256
				+	Thermostat_RX_BUF[2] + Thermostat_RX_BUF[3]*256
				+	Thermostat_RX_BUF[4] + Thermostat_RX_BUF[5]*256
				+	Thermostat_RX_BUF[6] + Thermostat_RX_BUF[7]*256)+1)&0xffff;
			
			
			//ʵ�ʽ��յ���У���
			myCRC     = Thermostat_RX_BUF[8]+Thermostat_RX_BUF[9]*256;
			if( myCRC == modbusCRC)              						//���CRCУ��ͨ��
			{
				
				PVTem	=	(Thermostat_RX_BUF[0] + Thermostat_RX_BUF[1]*256)/10;
				SVTem	=	(Thermostat_RX_BUF[2] + Thermostat_RX_BUF[3]*256)/10;
				Output	=	Thermostat_RX_BUF[4];
				Srun	=	Thermostat_RX_BUF[5];
				Value	=	Thermostat_RX_BUF[6] + Thermostat_RX_BUF[7]*256;
				
				
				//printf("�¿��� %c  %02x �ɹ�+++++++++\r\n",operate,name);
				//printf("SVTem.val=%d��\xff\xff\xff",SVTem);
				//printf("Output.val=%d\r\n",Output);
				//printf("Srun.val=%d\r\n",Srun);
				//printf("Value.val=%d\r\n",Value);
				
				printf("wendu.val=%d\xff\xff\xff",PVTem+15);
				Usart3_RxCounter=0;
				//if(i<5){printf("��%d������------�ɹ�******\r\n",5-i);}
				delay_ms(100);
				return 1;
												
			}else
			{
				printf("%02x CRCУ��ʧ��,myCRC:%x,modbusCRC:%x\r\n",name,myCRC,modbusCRC);	
			}
				
		}else{
		
			printf("�¿��� %c  %02x �������ݳ�ʱ******\r\n",operate,name);
		}
		
		//δ��ȷ��ȡ�������ݣ�����3�����������
		Usart3_RxCounter=0;
		//printf("��%d������******\r\n",6-i);
		i=i-1;
		delay_ms(100);
	}
	
	// i<0 ���ع��ϣ��¿��������쳣
	printf("\r\n\r\n���ع��ϣ��¿��������쳣---------------------\r\n\r\n");
	return 0;
}


/****************************************/
/*										*/
/*										*/
/*										*/
/*										*/
/****************************************/
	                                           
