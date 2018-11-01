/******************** (C) COPYRIGHT 2018 DY EleSc ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：串口驱动
**********************************************************************************/
#include "Drv_usart.h"
#include "include.h"
//#include "Ano_DT.h"
//#include "Ano_OF.h"
//
//#include "DY_Flight_Log.h"
//#include "Ano_AltCtrl.h"
//#include "Ano_AttCtrl.h"
//#include "Ano_FlightCtrl.h"
//
//void Usart2_Init ( u32 br_num )		//usart2	匿名数传
//{
//    USART_InitTypeDef USART_InitStructure;
//    USART_ClockInitTypeDef USART_ClockInitStruct;
//    NVIC_InitTypeDef NVIC_InitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//
//    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART2, ENABLE ); //开启USART2时钟
//    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );
//
//    //串口中断优先级
//    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init ( &NVIC_InitStructure );
//
//
//    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource5, GPIO_AF_USART2 );
//    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource6, GPIO_AF_USART2 );
//
//    //配置PD5作为USART2　Tx
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//    GPIO_Init ( GPIOD, &GPIO_InitStructure );
//    //配置PD6作为USART2　Rx
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init ( GPIOD, &GPIO_InitStructure );
//
//    //配置USART2
//    //中断被屏蔽了
//    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
//    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
//    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
//    //配置USART2时钟
//    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
//    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
//    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
//    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
//
//    USART_Init ( USART2, &USART_InitStructure );
//    USART_ClockInit ( USART2, &USART_ClockInitStruct );
//
//    //使能USART2接收中断
//    USART_ITConfig ( USART2, USART_IT_RXNE, ENABLE );
//    //使能USART2
//    USART_Cmd ( USART2, ENABLE );
////	//使能发送（进入移位）中断
////	if(!(USART2->CR1 & USART_CR1_TXEIE))
////	{
////		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
////	}
//
//
//}
//
//u8 TxBuffer[256];
//u8 TxCounter = 0;
//u8 count = 0;
//
//u8 Rx_Buf[256];	//串口接收缓存
//
//void Usart2_IRQ ( void )
//{
//    u8 com_data;
//
//    if ( USART2->SR & USART_SR_ORE ) //ORE中断
//    {
//        com_data = USART2->DR;
//    }
//
//    //接收中断
//    if ( USART_GetITStatus ( USART2, USART_IT_RXNE ) )
//    {
//        USART_ClearITPendingBit ( USART2, USART_IT_RXNE ); //清除中断标志
//
//        com_data = USART2->DR;
//        ANO_DT_Data_Receive_Prepare ( com_data );		//匿名数传
//    }
//    //发送（进入移位）中断
//    if ( USART_GetITStatus ( USART2, USART_IT_TXE ) )
//    {
//
//        USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
//        if ( TxCounter == count )
//        {
//            USART2->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
//        }
//
//
//        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
//    }
//
//
//
//}
//
//void Usart2_Send ( unsigned char *DataToSend , u8 data_num )
//{
//    u8 i;
//    for ( i = 0; i < data_num; i++ )
//    {
//        TxBuffer[count++] = * ( DataToSend + i );
//    }
//
//    if ( ! ( USART2->CR1 & USART_CR1_TXEIE ) )
//    {
//        USART_ITConfig ( USART2, USART_IT_TXE, ENABLE ); //打开发送中断
//    }
//
//}
//
//
//
//void Uart5_Init ( u32 br_num )		//usart5	PC12 TX PD2 Rx
//{
//    USART_InitTypeDef USART_InitStructure;
//    //USART_ClockInitTypeDef USART_ClockInitStruct;
//    NVIC_InitTypeDef NVIC_InitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//
//    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART5, ENABLE ); //开启USART2时钟
//    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOC, ENABLE );
//    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOD, ENABLE );
//
//    //串口中断优先级
//    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init ( &NVIC_InitStructure );
//
//
//    GPIO_PinAFConfig ( GPIOC, GPIO_PinSource12, GPIO_AF_UART5 );
//    GPIO_PinAFConfig ( GPIOD, GPIO_PinSource2, GPIO_AF_UART5 );
//
//    //配置PC12作为UART5　Tx
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//    GPIO_Init ( GPIOC, &GPIO_InitStructure );
//    //配置PD2作为UART5　Rx
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init ( GPIOD, &GPIO_InitStructure );
//
//    //配置UART5
//    //中断被屏蔽了
//    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
//    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
//    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
//    USART_Init ( UART5, &USART_InitStructure );
//
//
//
//    //使能UART5接收中断
//    USART_ITConfig ( UART5, USART_IT_RXNE, ENABLE );
//    //使能USART5
//    USART_Cmd ( UART5, ENABLE );
////	//使能发送（进入移位）中断
////	if(!(USART2->CR1 & USART_CR1_TXEIE))
////	{
////		USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
////	}
//
//}
//u8 Tx5Buffer[256];
//u8 Tx5Counter = 0;
//u8 count5 = 0;
//
//void Uart5_IRQ ( void )
//{
//    //接收中断
//    if ( USART_GetITStatus ( UART5, USART_IT_RXNE ) )
//    {
//        USART_ClearITPendingBit ( UART5, USART_IT_RXNE ); //清除中断标志
//
//        u8 com_data = UART5->DR;
//
//        //Ultra_Get ( com_data );
//    }
//
//    //发送（进入移位）中断
//    if ( USART_GetITStatus ( UART5, USART_IT_TXE ) )
//    {
//
//        UART5->DR = Tx5Buffer[Tx5Counter++]; //写DR清除中断标志
//
//        if ( Tx5Counter == count5 )
//        {
//            UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
//        }
//
//
//        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
//    }
//
//}
//
//void Uart5_Send ( unsigned char *DataToSend , u8 data_num )
//{
//    u8 i;
//    for ( i = 0; i < data_num; i++ )
//    {
//        Tx5Buffer[count5++] = * ( DataToSend + i );
//    }
//
//    if ( ! ( UART5->CR1 & USART_CR1_TXEIE ) )
//    {
//        USART_ITConfig ( UART5, USART_IT_TXE, ENABLE ); //打开发送中断
//    }
//
//}
//
//
//void Uart4_Init ( u32 br_num )		//usart4	匿名光流
//{
//    USART_InitTypeDef USART_InitStructure;
//    //USART_ClockInitTypeDef USART_ClockInitStruct;
//    NVIC_InitTypeDef NVIC_InitStructure;
//    GPIO_InitTypeDef GPIO_InitStructure;
//
//    RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART4, ENABLE ); //开启USART2时钟
//    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );
//
//    //串口中断优先级
//    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init ( &NVIC_InitStructure );
//
//
//    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource0, GPIO_AF_UART5 );
//    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource1, GPIO_AF_UART5 );
//
//    //配置PC12作为UART5　Tx
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
//    GPIO_Init ( GPIOA, &GPIO_InitStructure );
//    //配置PD2作为UART5　Rx
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//    GPIO_Init ( GPIOA, &GPIO_InitStructure );
//
//    //配置UART5
//    //中断被屏蔽了
//    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
//    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
//    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
//    USART_Init ( UART4, &USART_InitStructure );
//
//    //使能UART5接收中断
//    USART_ITConfig ( UART4, USART_IT_RXNE, ENABLE );
//    //使能USART5
//    USART_Cmd ( UART4, ENABLE );
//}
//u8 Tx4Buffer[256];
//u8 Tx4Counter = 0;
//u8 count4 = 0;
//
//void Uart4_IRQ ( void )
//{
//    u8 com_data;
//
//    //接收中断
//    if ( USART_GetITStatus ( UART4, USART_IT_RXNE ) )
//    {
//        USART_ClearITPendingBit ( UART4, USART_IT_RXNE ); //清除中断标志
//
//        com_data = UART4->DR;
//
//        AnoOF_GetOneByte ( com_data );		//匿名光流
//    }
//
//    //发送（进入移位）中断
//    if ( USART_GetITStatus ( UART4, USART_IT_TXE ) )
//    {
//
//        UART4->DR = Tx4Buffer[Tx4Counter++]; //写DR清除中断标志
//
//        if ( Tx4Counter == count4 )
//        {
//            UART4->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
//        }
//
//
//        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
//    }
//
//}
//
//
///******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
//
////////////////////////////////////////////////////////////////////
////加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
//#if 1
//#pragma import(__use_no_semihosting)             
////标准库需要的支持函数                 
//struct __FILE 
//{ 
//	int handle; 
//}; 
//
//FILE __stdout;       
////定义_sys_exit()以避免使用半主机模式    
//void _sys_exit(int x) 
//{ 
//	x = x; 
//} 
////重定义fputc函数 
//int fputc(int ch, FILE *f)
//{ 	
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//	USART1->DR = (u8) ch;      
//	return ch;
//}
//#endif
//
//#if EN_USART1_RX   //如果使能了接收
////串口1中断服务程序
////注意,读取USARTx->SR能避免莫名其妙的错误   	
//u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
////接收状态
////bit15，	接收完成标志
////bit14，	接收到0x0d
////bit13~0，	接收到的有效字节数目
//u16 USART_RX_STA=0;       //接收状态标记	

//初始化IO 串口0
//bound:波特率
void uart_init(u32 bound)
{
    //
        // Enable the GPIO Peripheral used by the UART.
        //
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);

        //
        // Enable UART0.
        //
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
        MAP_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
        
        //
        // Enable processor interrupts.
        //
        MAP_IntMasterEnable();

        //
        // Configure GPIO Pins for UART mode.
        //
        MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
        MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
        MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        //
        // Configure the UART for bound, 8-N-1 operation.
        //
        MAP_UARTConfigSetExpClk(UART0_BASE, systemClock, bound,
                                (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                                 UART_CONFIG_PAR_NONE));
        
        //
        // Enable the UART interrupt.
        //
        MAP_IntEnable(INT_UART0);
        MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void UARTSend(const u8 *pui8Buffer, u32 ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        MAP_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UART0_IRQHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    MAP_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(MAP_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        MAP_UARTCharPutNonBlocking(UART0_BASE,
                                   MAP_UARTCharGetNonBlocking(UART0_BASE));
        
        /*      中断服务函数  */
//        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 1);
    }
}

//void USART1_IRQHandler(void)                	//串口1中断服务程序
//{
//	u8 Res;
//
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{
//		USART_ClearITPendingBit ( USART1, USART_IT_RXNE ); //清除中断标志
//		
//		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
//		
//		if(Res == 'G')
//			dy_flag.pit_go = 1;
//		if(Res == 'B')
//			dy_flag.pit_back = 1;
//		if(Res == 'L')
//			dy_flag.rol_left = 1;
//		if(Res == 'R')
//			dy_flag.rol_right = 1;
//		if(Res == 'C')
//			dy_flag.yaw_cw = 1;
//		if(Res == 'F')
//			dy_flag.yaw_ccw = 1;
//		if(Res == 'U')
//			dy_flag.up = 1;
//		if(Res == 'D')
//			dy_flag.down = 1;
//		if(Res == 'S')
//			dy_flag.stop = 1;
//		if(Res == '1')
//			dy_flight_plan.plan1 = 1;
//		
////		if((USART_RX_STA&0x8000)==0)//接收未完成
////		{
////			if(USART_RX_STA&0x4000)//接收到了0x0d
////			{
////				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
////				else USART_RX_STA|=0x8000;	//接收完成了 
////			}
////			else //还没收到0X0D
////			{	
////				if(Res==0x0d)USART_RX_STA|=0x4000;
////				else
////				{
////					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
////					USART_RX_STA++;
////					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
////				}		 
////			}
////		}   		 
//  } 
//
//} 
//#endif
