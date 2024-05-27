
/*-------------------------------------------------*/
/*                                                 */
/*          	       ����4                  	   */
/*                                                 */
/*-------------------------------------------------*/

#include "stm32f10x.h"     //������Ҫ��ͷ�ļ�
#include "FreeRTOS.h"	   //FreeRTOSʹ��		  
#include "task.h"
#include "event_groups.h"
#include "stm32f10x_it.h"  //������Ҫ��ͷ�ļ�
#include "usart1.h"        //������Ҫ��ͷ�ļ�
#include "usart2.h"        //������Ҫ��ͷ�ļ�
#include "uart4.h"        //������Ҫ��ͷ�ļ�
#include "timer3.h"        //������Ҫ��ͷ�ļ�
#include "mqtt.h"          //������Ҫ��ͷ�ļ�
#include "dht11.h"         //������Ҫ��ͷ�ļ�                  

extern void xPortSysTickHandler(void);
extern TaskHandle_t WIFI_Task_Handler;
extern  EventGroupHandle_t Event_Handle;
extern const int PING_MODE;

unsigned int Usart4_RxCounter = 0;      //����һ����������¼����4�ܹ������˶����ֽڵ�����
char Usart4_RxBuff[USART4_RXBUFF_SIZE]; //����һ�����飬���ڱ��洮��4���յ�������
/*-------------------------------------------------*/
/*����������ʼ������4���͹���                      */
/*��  ����bound��������                            */
/*����ֵ����                                       */
/*-------------------------------------------------*/
void usart4_init(unsigned int bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;     //����һ������GPIO���ܵı���
  USART_InitTypeDef USART_InitStructure;   //����һ�����ô��ڹ��ܵı���
  NVIC_InitTypeDef NVIC_InitStructure;     //���ʹ�ܽ��չ��ܣ�����һ�������жϵı���

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); //ʹ�ܴ���4ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //ʹ��GPIOCʱ��
  USART_DeInit(UART4);                                  //����4�Ĵ�����������ΪĬ��ֵ

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;              //׼������PC10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO����50M
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	       //����������������ڴ���4�ķ���
  GPIO_Init(GPIOC, &GPIO_InitStructure);                 //����PC10

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;              //׼������PC11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;			//�������� �����ڴ���4�Ľ���
  GPIO_Init(GPIOC, &GPIO_InitStructure);                 //����PC11

  USART_InitStructure.USART_BaudRate = bound;                                    //����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                    //8������λ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                         //1��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;                            //����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	               //�շ�ģʽ

  USART_Init(UART4, &USART_InitStructure);                                      //���ô���4

  USART_ClearFlag(UART4, USART_FLAG_RXNE);	              //������ձ�־λ
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);            //���������ж�

  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;         //���ô���4�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; //��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  //�����ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //�ж�ͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);	                          //���ô���2�ж�

  USART_Cmd(UART4, ENABLE);                                //ʹ�ܴ���4
}

/*-------------------------------------------------*/
/*������������4 printf����                         */
/*��  ����char* fmt,...  ��ʽ������ַ����Ͳ���    */
/*����ֵ����                                       */
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
/*������������4���ͻ������е�����                  */
/*��  ����data������                               */
/*����ֵ����                                       */
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
/*��������void USART4_IRQHandler(void) 			      			 */
/*��  �ܣ�����4�жϴ�����										 */
/*		  1.��esp8266ͨ�ţ��Ѿ����ӷ��������ƶ�ʱ��4��δ���ӷ����� */
/*			�����ƶ�ʱ��4��ͨ���¼���־���λ0 WIFI_CONECT�жϣ�	 */
/*��  ������                                       				 */
/*����ֵ����                                     				 */
/*---------------------------------------------------------------*/
void UART4_IRQHandler(void)
{

  if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)         //���USART_IT_RXNE��־��λ����ʾ�����ݵ��ˣ�����if��֧
    {

      if ((xEventGroupGetBitsFromISR(Event_Handle) & 0x01) == 0)//��ȡ�¼���־�����ݣ�����0˵��δ���ӷ���������������ʱ��4��MQTT�������ݴ�����ʱ��
        {
          if(UART4->DR)                                        //����ָ������״̬ʱ������ֵ�ű��浽������
            {
              Usart4_RxBuff[Usart4_RxCounter] = UART4->DR;	  //���浽������
              Usart4_RxCounter++; 						      //ÿ����1���ֽڵ����ݣ�Usart4_RxCounter��1����ʾ���յ���������+1
            }
        }
      else
        {
          Usart4_RxBuff[Usart4_RxCounter] = UART4->DR;//�ѽ��յ������ݱ��浽Usart4_RxBuff��

          if(Usart4_RxCounter == 0)				     //���Usart4_RxCounter����0����ʾ�ǽ��յĵ�1�����ݣ�����if��֧
            {
              TIM_Cmd(TIM4, ENABLE); 					 //ʹ�ܶ�ʱ��4
            }
          else										 //else��֧����ʾ��Usart4_RxCounter������0�����ǽ��յĵ�һ������
            {
              TIM_SetCounter(TIM4, 0);  				 //��λ��ʱ��4
            }
          Usart4_RxCounter++;         				 //ÿ����1���ֽڵ����ݣ�Usart4_RxCounter��1����ʾ���յ���������+1
        }

    }

}

