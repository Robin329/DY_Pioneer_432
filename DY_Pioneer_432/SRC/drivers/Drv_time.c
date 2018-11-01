/******************** (C) COPYRIGHT 2018 DY EleSc ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：定时器驱动和滴答配置
**********************************************************************************/

#include "Drv_time.h"
#include "include.h"
//#include "Drv_led.h"
//
//#define SYS_TIMx					TIM2
//#define SYS_RCC_TIMx			RCC_APB1Periph_TIM2
//
//void TIM_CONF()   //APB1  84M
//{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//
//    /* 使能时钟 */
//    RCC_APB1PeriphClockCmd ( SYS_RCC_TIMx, ENABLE );
//
//    TIM_DeInit ( SYS_TIMx );
//
//    /* 自动重装载寄存器周期的值(计数值) */
//    TIM_TimeBaseStructure.TIM_Period = 1000;
//
//    /* 累计 TIM_Period个频率后产生一个更新或者中断 */
//    /* 时钟预分频数为72 */
//    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
//
//    /* 对外部时钟进行采样的时钟分频,这里没有用到 */
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数
//
//    TIM_TimeBaseInit ( SYS_TIMx, &TIM_TimeBaseStructure );
//
//    TIM_ClearFlag ( SYS_TIMx, TIM_FLAG_Update );
//
//    TIM_ITConfig ( SYS_TIMx, TIM_IT_Update, ENABLE );
//
//
//    TIM_Cmd ( SYS_TIMx, ENABLE );
//
//    RCC_APB1PeriphClockCmd ( SYS_RCC_TIMx , DISABLE );		/*先关闭等待使用*/
//}
//void TIM_NVIC()
//{
//    NVIC_InitTypeDef NVIC_InitStructure;
//
////    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
//    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TIME_P;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_TIME_S;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init ( &NVIC_InitStructure );
//}
//
//void TIM_INIT()
//{
//    TIM_CONF();
//    TIM_NVIC();
//
//    /* TIM2 重新开时钟，开始计时 */
//    RCC_APB1PeriphClockCmd ( SYS_RCC_TIMx , ENABLE );
//}

uint32_t systemClock;
volatile uint32_t sysTickUptime = 0;

void  SysTick_Configuration ( void )		//1ms产生一次异常请求
{
    /* Configure the system clock for 120 MHz */
    systemClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                          120000000);
    
    /* Enable the SysTick timer to generate an interrupt every 1/1000 second */
    MAP_SysTickPeriodSet(systemClock/1000);         //1ms
    MAP_SysTickIntEnable();
    MAP_SysTickEnable();
}

//uint32_t GetSysTime_us ( void )
//{
//    register uint32_t ms;		//register修饰符暗示编译程序相应的变量将被频繁地使用,如果可能的话,应将其保存在CPU的寄存器中，以加快其存储速度。
//    u32 value;
//    ms = sysTickUptime;
//    value = ms * TICK_US + ( SysTick->LOAD - SysTick->VAL ) * TICK_US / SysTick->LOAD;		//ms*1000+(重装载值-当前倒计数值)/重装载值[对应1ms]*1000=SysTime_us
//    return value;
//}
//
//void Delay_us ( uint32_t us )
//{
//    uint32_t now = GetSysTime_us();
//    while ( GetSysTime_us() - now < us );
//}
//
//void Delay_ms ( uint32_t ms )
//{
//    while ( ms-- )
//        Delay_us ( 1000 );
//}
//
u32 systime_ms;

void sys_time()
{
	systime_ms++;
}
//u32 SysTick_GetTick(void)
//{
//	return systime_ms;
//}
//
//
//
//u32 Get_Delta_T(_get_dT_st *data)
//{
//    data->old = data->now;	//上一次的时间
//    data->now = GetSysTime_us(); //本次的时间
//    data->dT = ( ( data->now - data->old ) );//间隔的时间（周期）
//	
//	if(data->init_flag == 0)
//	{
//		data->init_flag = 1;//第一次调用时输出 0 作为初始化，以后正常输出
//		return 0;
//	}
//	else
//	{
//    return data->dT;
//	}
//}

void SysTick_Handler(void)
{
//      uint32_t gpioState;
//
//      /* Read the current GPIO Pin Value */
//      gpioState = MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4);
//
//      /* Toggle the GPIO read value */
//      gpioState ^= GPIO_PIN_4;
//
//      /* Write the new value of GPIO to the port-pin */
//      MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, gpioState);
	sysTickUptime++;
	sys_time();		//systime_ms++		
//	LED_1ms_DRV();
}
/******************* (C) COPYRIGHT 2018 DY EleSc *****END OF FILE************/



