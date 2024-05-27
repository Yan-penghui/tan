#include "switch.h"
#include "stm32f10x.h"





//4���̵��� 2����λ����
void Switch_Init()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��PORTC

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);


  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;//PA7 ʹ��485 �ߵ�ƽ�ɷ��� �͵�ƽ�ɽ��� ��ʼ��Ϊ�͵�ƽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA,GPIO_Pin_7);

  //����������λ���صĳ�ʼ��

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;//PA8 ����ͨ�� �͵�ƽ��Ч ��ʼ��Ϊ�ߵ�ƽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA,GPIO_Pin_8);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;//PB0 //����λ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
  GPIO_ResetBits(GPIOB,GPIO_Pin_0);



  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;//PB1 ����λ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
  GPIO_ResetBits(GPIOB,GPIO_Pin_1);


  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;//PB12 ����� �͵�ƽ��Ч ��ʼ��Ϊ�ߵ�ƽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_12);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;//PB13  ����� �͵�ƽ��Ч ��ʼ��Ϊ�ߵ�ƽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_13);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_14;//PB14 �·��� �͵�ƽ��Ч ��ʼ��Ϊ�ߵ�ƽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_14);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;//PB15  �Ϸ��� �͵�ƽ��Ч ��ʼ��Ϊ�ߵ�ƽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB,GPIO_Pin_15);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;    //PC13 ������ �ߵ�ƽ��Ч ��ʼ��Ϊ�͵�ƽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOC,GPIO_Pin_13);

  //3led  �ߵ�ƽ��Ч ��ʼ��Ϊ�͵�ƽ

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;    //PB4 YELLOW_LED
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_4);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;    //PB5 RED_LED
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_5);

  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;    //PB8 GREEN_LED
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_8);

}
