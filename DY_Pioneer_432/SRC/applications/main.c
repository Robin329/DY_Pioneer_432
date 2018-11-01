/******************** (C) COPYRIGHT 2018 DY EleSc ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：主循环
**********************************************************************************/

#include "DY_FcData.h"
#include "include.h"

//=======================================================================================
int main(void)
{ 
	flag.start_ok = All_Init();		//进行所有设备的初始化，并将初始化结果保存
        
//	Scheduler_Setup();				//调度器初始化，系统为裸奔，这里人工做了一个时分调度器
//	UARTprintf("flag.start_ok:%d\r\n",flag.start_ok);
//        UARTSend((uint8_t *)"Hello World!", sizeof("Hello World!"));
//	ANO_DT_Send_Test_u8(flag.start_ok);
	
	while(1)
	{
//		Scheduler_Run();			//运行任务调度器，所有系统功能，除了中断服务函数，都在任务调度器内完成
	}
}
/******************* (C) COPYRIGHT 2018 DY EleSc *****END OF FILE************/

//volatile uint32_t gpioState = 0;
//
//void SysTick_Handler(void)
//{
//    if(gpioState != 3)
//    {
//        gpioState++;
//    }
//    else
//    {
//        gpioState = 0;
//    }
//
//    /* Write the new value of GPIO to the port-pin. The API will generate
//     * the bit banded address to write to the specific port pin */
//    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, gpioState);
//
//}
//
//int main(void)
//{
//    uint32_t systemClock;
//
//    /* Configure the system clock for 16 MHz */
//    systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
//                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
//                                          16000000);
//
//    /* Enable the clock to the GPIO Port N and wait for it to be ready */
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
//    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)))
//    {
//
//    }
//
//    /* Configure the GPIO PN0-PN1 as output */
//    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, (GPIO_PIN_0 | GPIO_PIN_1));
//    MAP_GPIOPinWrite(GPIO_PORTN_BASE, (GPIO_PIN_0 | GPIO_PIN_1), 0);
//
//    /* Enable the SysTick timer to generate an interrupt every 1 second */
//    MAP_SysTickPeriodSet(systemClock);
//    MAP_SysTickIntEnable();
//    MAP_SysTickEnable();
//
//    while(1)
//    {
//        
//    }
//}