#ifndef _DRV_USART_H_
#define _DRV_USART_H_

#include "DY_FcData.h"
//#include "stdio.h"

//#define USART_REC_LEN  			200  	//�����������ֽ��� 200
//#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
//
//extern u8 Rx_Buf[];
//void Usart2_Init(u32 br_num);
//void Usart2_IRQ(void);
//void Usart2_Send(unsigned char *DataToSend ,u8 data_num);
//
//void Uart4_Init(u32 br_num);
//void Uart4_IRQ(void);
//
//void Uart5_Init(u32 br_num);
//void Uart5_IRQ(void);
//void Uart5_Send(unsigned char *DataToSend ,u8 data_num);

void uart_init(u32 bound);
void UARTSend(const u8 *pui8Buffer, u32 ui32Count);

#endif
