#ifndef __SWITCH_H
#define __SWITCH_H
//#include "sys.h"
#include "stm32f10x.h" 

#define LIM1 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0) //��λ����1 ��
#define LIM2 GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1) //��λ����2 ��
#define UP PBout(13)// �������
#define DOWN PBout(12)// �������
#define FAN_DOWN PBout(14)//  �·�����ͣ
#define FAN_UP PBout(15)//  �Ϸ�����ͣ 

#define BUZZER PCout(13)//������
#define POWER PAout(8)// ����ͨ��
#define RS485 PAout(7)

#define YELLOW_LED PBout(4)
#define RED_LED PBout(5)
#define GREEN_LED PBout(8)



void Switch_Init(void);



#endif
