#ifndef __THERMOSTAT_H
#define  __THERMOSTAT_H

#define Thermostat_printf       u3_printf           //����3���� �¿���
#define Thermostat_RxCounter    Usart3_RxCounter    //����3���� �¿���
#define Thermostat_RX_BUF       Usart3_RxBuff       //����3���� �¿���
#define Thermostat_RXBUFF_SIZE  USART3_RXBUFF_SIZE  //����3���� �¿���

extern int PVTem ,SVTem,RunTime ,Srun ,Pno;
void Thermostat_init(void)	;	
int Send_Thermostat(char operate,u8 name,u16 w_data);

#endif



