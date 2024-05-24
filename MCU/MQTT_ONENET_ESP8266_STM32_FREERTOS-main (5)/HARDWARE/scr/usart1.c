
/*-------------------------------------------------*/
/*                                                 */
/*          		  ����1                    	   */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f10x.h"  
#include "usart1.h"  
#include "FreeRTOS.h"	 //FreeRTOS����ͷ�ļ�
#include "queue.h"		 //����

unsigned int Usart1_RxCounter = 0;      //����һ����������¼����1�ܹ������˶����ֽڵ�����
char Usart1_RxBuff[USART1_RXBUFF_SIZE]; //����һ�����飬���ڱ��洮��1���յ�������  
extern QueueHandle_t U1_xQueue;
u8 input_data[70];
/*-------------------------------------------------*/
/*��������printf�ض���      				           */
/*��  ����                                         */
/*����ֵ��                                         */
/*-------------------------------------------------*/  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40) == 0);//ѭ������,ֱ���������   
    USART1->DR = (u8)ch;      
	return ch;
}
#endif 


/*-------------------------------------------------*/
/*����������ʼ������1���͹���                       */
/*��  ����bound��������                            */
/*����ֵ����                                       */
/*-------------------------------------------------*/
void usart1_init(unsigned int bound)
{  	 	
	GPIO_InitTypeDef GPIO_InitStructure;     //����һ������GPIO���ܵı���
	USART_InitTypeDef USART_InitStructure;   //����һ�����ô��ڹ��ܵı���
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //ʹ�ܴ���1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOAʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;              //׼������PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO����50M
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       //����������������ڴ���1�ķ���
	GPIO_Init(GPIOA, &GPIO_InitStructure);                 //����PA9
   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;             //׼������PA10 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  //�������룬���ڴ���1�Ľ���
	GPIO_Init(GPIOA, &GPIO_InitStructure);                 //����PA10
	
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;//��ռ���ȼ�5
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	
	USART_InitStructure.USART_BaudRate = bound;                                    //����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //8������λ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //1��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
                                                                                   //�����ʹ�ܽ���ģʽ
	USART_InitStructure.USART_Mode =  USART_Mode_Rx | USART_Mode_Tx;				//�շ�ģʽ	                                
	USART_Init(USART1, &USART_InitStructure);                                      //���ô���1	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);									//�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                            						 	//ʹ�ܴ���1
	USART_ClearFlag(USART1,USART_FLAG_TC); 											//���USARTx�Ĵ�������־λ 
}

/*---------------------------------------------------------------*/
/*��������void USART1_IRQHandler(void) 			      			 */
/*��  �ܣ�����1�жϴ�������										 */
/*		  1.�봮����ͨ��											 */
/*��  ������                                       				 */
/*����ֵ����                                     				 */
/*---------------------------------------------------------------*/
void USART1_IRQHandler(void)   
{   
	u8 Usart1_RxBuff[140]; //����һ�����飬���ڱ��洮��1���յ������� 
	
	int i;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)        	 //���USART_IT_RXNE��־��λ����ʾ�����ݵ��ˣ�����if��֧	   
	{   
		Usart1_RxBuff[Usart1_RxCounter] = USART1->DR;				//�ѽ��յ������ݱ��浽Usart1_RxBuff��
		if(Usart1_RxBuff[0]!=0x24)				     				//�����ͷ������24	
		{    								
			Usart1_RxCounter = 0; 					 				//��������
			printf("�������ݣ�%2x  ",Usart1_RxBuff[0]);
		}
		else														 //else��֧����ʾ��Usart1_RxCounter������0�����ǽ��յĵ�һ������
		{                        									    
			Usart1_RxCounter=Usart1_RxCounter+1;         			//ÿ����1���ֽڵ����ݣ�Usart1_RxCounter��1����ʾ���յ���������+1 		
		}	
	}

	if(Usart1_RxBuff[Usart1_RxCounter-1] ==0xFF)					//���յ���β0xff
	{
		xQueueSendToFrontFromISR( U1_xQueue,&Usart1_RxBuff,NULL);	//д����
				
	}
	
}