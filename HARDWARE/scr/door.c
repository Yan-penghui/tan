
#include "stm32f10x.h"
#include "door.h"
#include "usart1.h"
#include "delay.h"
#include "FreeRTOS.h"	 //FreeRTOS配置头文件
#include "queue.h"		 //队列
#include "switch.h"
#include "timers.h"
#include "Thermostat.h"      //包含需要的头文件 
extern u8 door_flag;
void my_door_task()
{
  while(1)
    {
      u8 runtime=100;
      //电机关闭100s
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
          UP = 1; /*低电平有效*/
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


void updoor(u8 run_time)//上升炉门  （超时报错：秒）
{
  int timeout = door_outtime*10;
  int runtime = run_time * 10;			/*（动作时间：0.1秒,超时时间：0.1秒)*/
  if(run_time==0)
    {
      runtime =door_outtime*10;
    }

  if(LIM1 !=1 )
    {
      DOWN=1;
      UP = 0; //低电平有效
      while(timeout--)
        {
          delay_ms(100);
          runtime = runtime -1;



          //运行到下限位退出
          if(LIM1==1)
            {
              printf("运行到下限位退出\r\n");
              break;
            }

          //接收到停止指令：手动退出
          if(door_flag==0)
            {
              printf("接收到停止指令：手动退出\r\n");
              break;
            }

          //运行到达设定时间退出
          if(runtime <= 0)
            {
              printf("运行到达设定时间退出\r\n");
              break;
            }

        }

      //超时
      if(timeout<=0)
        {
          printf("炉门上升超时故障\r\n");
          stopdoor();
        }

    }
  DOWN=1;
  UP = 1; /*低电平有效*/
  door_flag=0;
}

void downdoor(u8 run_time)//下降炉门  （动作时间：秒,超时时间：秒）
{
  int timeout = door_outtime*10;
  int runtime = run_time * 10;			/*（动作时间：0.1秒,超时时间：0.1秒)*/
  if(run_time==0)
    {
      runtime =door_outtime*10;
    }



  if( PVTem > 1200 )
    {
      /*page luwenguogao*/
      //printf("炉温过高\r\n");
      return;
    }

  if(LIM2 != 1 )
    {
      DOWN=0;
      UP = 1; //低电平有效
      while(timeout--)
        {
          delay_ms(100);
          runtime = runtime -1;

          //运行到达设定时间退出
          if(runtime <= 0 )
            {
              printf("运行到达设定时间退出\r\n");
              break;
            }

          //运行到下限位退出
          if(LIM2==1)
            {
              printf("运行到下限位退出");
              break;
            }

          //接收到停止指令：手动退出
          if(door_flag==0)
            {
              printf("接收到停止指令：手动退出\r\n");
              break;
            }

        }

      //超时故障
      if(timeout<=0)
        {
          printf("炉门下降超时故障\r\n");
          stopdoor();
        }
    }

  DOWN=1;
  UP = 1; /*低电平有效*/
  door_flag=0;
}


void stopdoor()
{
//	printf("up.pic=0\xff\xff\xff");
//	printf("down.pic=2\xff\xff\xff");
  while(1)
    {
      DOWN=1;
      UP = 1; //低电平有效

    }
}
