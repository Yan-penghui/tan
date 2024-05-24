#ifndef __THERMOSTAT_H
#define  __THERMOSTAT_H

#define Thermostat_printf       u3_printf           //´®¿Ú3¿ØÖÆ ÎÂ¿ØÆ÷
#define Thermostat_RxCounter    Usart3_RxCounter    //´®¿Ú3¿ØÖÆ ÎÂ¿ØÆ÷
#define Thermostat_RX_BUF       Usart3_RxBuff       //´®¿Ú3¿ØÖÆ ÎÂ¿ØÆ÷
#define Thermostat_RXBUFF_SIZE  USART3_RXBUFF_SIZE  //´®¿Ú3¿ØÖÆ ÎÂ¿ØÆ÷

extern int PVTem ,SVTem,RunTime ,Srun ,Pno;
void Thermostat_init(void)	;	
int Send_Thermostat(char operate,u8 name,u16 w_data);

#endif



