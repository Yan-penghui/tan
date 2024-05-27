
#include "stm32f10x.h"
#include "door.h"
#include "usart1.h"
#include "delay.h"
#include "FreeRTOS.h"	 //FreeRTOS����ͷ�ļ�
#include "queue.h"		 //����
#include "switch.h"
#include "timers.h"
#include "Thermostat.h"      //������Ҫ��ͷ�ļ� 
extern u8 door_flag;
void my_door_task()
{
  while(1)
    {
      u8 runtime=100;
      //����ر�100s
      //printf("----------my_door_task----------\r\n");
      if (door_flag == 1)
        {
          printf("up.pic=1\xff\xff\xff");
          printf("down.pic=2\xff\xff\xff");
          updoor(runtime);
          printf("up.pic=0\xff\xff\xff");
          printf("down.pic=2\xff\xff\xff");
          printf("up.pic=0\xff\xff\xff");
          printf("down.pic=2\xff\xff\xff");
        }

      if (door_flag != 1||door_flag != 2)
        {

          DOWN=1;
          UP = 1; /*�͵�ƽ��Ч*/
        }

      if (door_flag == 2)
        {
          printf("up.pic=0\xff\xff\xff");
          printf("down.pic=3\xff\xff\xff");
          downdoor(runtime);
          printf("up.pic=0\xff\xff\xff");
          printf("down.pic=2\xff\xff\xff");
        }
      delay_ms(200);
    }
}


void updoor(u8 run_time)//����¯��  ����ʱ�����룩
{
  int timeout = door_outtime*10;
  int runtime = run_time * 10;			/*������ʱ�䣺0.1��,��ʱʱ�䣺0.1��)*/
  if(run_time==0)
    {
      runtime =door_outtime*10;
    }

  if(LIM1 !=1 )
    {
      DOWN=1;
      UP = 0; //�͵�ƽ��Ч
      while(timeout--)
        {
          delay_ms(100);
          runtime = runtime -1;



          //���е�����λ�˳�
          if(LIM1==1)
            {
              printf("���е�����λ�˳�\r\n");
              break;
            }

          //���յ�ָֹͣ��ֶ��˳�
          if(door_flag==0)
            {
              printf("���յ�ָֹͣ��ֶ��˳�\r\n");
              break;
            }

          //���е����趨ʱ���˳�
          if(runtime <= 0)
            {
              printf("���е����趨ʱ���˳�\r\n");
              break;
            }

        }

      //��ʱ
      if(timeout<=0)
        {
          printf("¯��������ʱ����\r\n");
          stopdoor();
        }

    }
  DOWN=1;
  UP = 1; /*�͵�ƽ��Ч*/
  door_flag=0;
}

void downdoor(u8 run_time)//�½�¯��  ������ʱ�䣺��,��ʱʱ�䣺�룩
{
  int timeout = door_outtime*10;
  int runtime = run_time * 10;			/*������ʱ�䣺0.1��,��ʱʱ�䣺0.1��)*/
  if(run_time==0)
    {
      runtime =door_outtime*10;
    }



  if( PVTem > 1200 )
    {
      /*page luwenguogao*/
      //printf("¯�¹���\r\n");
      return;
    }

  if(LIM2 != 1 )
    {
      DOWN=0;
      UP = 1; //�͵�ƽ��Ч
      while(timeout--)
        {
          delay_ms(100);
          runtime = runtime -1;

          //���е����趨ʱ���˳�
          if(runtime <= 0 )
            {
              printf("���е����趨ʱ���˳�\r\n");
              break;
            }

          //���е�����λ�˳�
          if(LIM2==1)
            {
              printf("���е�����λ�˳�");
              break;
            }

          //���յ�ָֹͣ��ֶ��˳�
          if(door_flag==0)
            {
              printf("���յ�ָֹͣ��ֶ��˳�\r\n");
              break;
            }

        }

      //��ʱ����
      if(timeout<=0)
        {
          printf("¯���½���ʱ����\r\n");
          stopdoor();
        }
    }

  DOWN=1;
  UP = 1; /*�͵�ƽ��Ч*/
  door_flag=0;
}


void stopdoor()
{
//	printf("up.pic=0\xff\xff\xff");
//	printf("down.pic=2\xff\xff\xff");
  while(1)
    {
      DOWN=1;
      UP = 1; //�͵�ƽ��Ч

    }
}
