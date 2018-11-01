/******************** (C) COPYRIGHT 2018 DY EleSc ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：PWM输出
**********************************************************************************/
#include "Drv_pwm_out.h"
#include "include.h"
//#include "Ano_Math.h"

//21分频到 84000000/21 = 4M   0.25us

///*初始化高电平时间1000us（4000份）*/
//#define INIT_DUTY 4000 //u16(1000/0.25)
///*频率400hz*/
//#define HZ        400
///*精度10000，每份0.25us*/
//#define ACCURACY 10000 //u16(2500/0.25) //accuracy
///*设置飞控控制信号转换比例为4*/
//#define PWM_RADIO 4//(8000 - 4000)/1000.0

void PWM_Out_Init () //400hz	周期2.5ms
{
    /* The PWM peripheral must be enabled for use. */                           //PWM0外设使能
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)));

    /* Set the PWM clock to the system clock / 8. */                            //设置PWM0时钟
    MAP_PWMClockSet(PWM0_BASE,PWM_SYSCLK_DIV_8);

    /* Enable the clock to the GPIO Port F for PWM pins */                      //使能GPIOF时钟
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    MAP_GPIOPinConfigure(GPIO_PF0_M0PWM0);                                      //PF0、PF1
    MAP_GPIOPinConfigure(GPIO_PF1_M0PWM1);
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);                                      //PF2、PF3
    MAP_GPIOPinConfigure(GPIO_PF3_M0PWM3);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3));

    /* Configure the PWM0 to count up/down without synchronization. */          //PWM0计数方式
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                        PWM_GEN_MODE_NO_SYNC);
    
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |
                        PWM_GEN_MODE_NO_SYNC);
    
    /* Set the PWM period to 400Hz.  To calculate the appropriate parameter
     * use the following equation: N = (1 / f) * SysClk.  Where N is the
     * function parameter, f is the desired frequency, and SysClk is the
     * system clock frequency.
     * In this case you get: (1 / 400Hz) * 15MHz = 375000 cycles.  Note that
     * the maximum period you can set is 2^16 - 1. */
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 37500);
    
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 37500);
    
    /* Set PWM0 PF0 to a duty cycle of 25%.  You set the duty cycle as a
     * function of the period.  Since the period was set above, you can use the
     * PWMGenPeriodGet() function.  For this example the PWM will be high for
     * 25% of the time or 15000 clock cycles (37500 / 4). */
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,
                     MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 4);

    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1,
                     2*MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0) / 4);
    
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                     3*MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) / 4);
    
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3,
                     MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) / 4);

    MAP_IntMasterEnable();
    
    /* This timer is in up mode.  Interrupts will occur when the
     * counter for this PWM counts up to 37500/4 (PWM A Up). */
    MAP_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0,
                            PWM_INT_CNT_AU);
    
    MAP_PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_1,
                            PWM_INT_CNT_AU);
        
    MAP_IntEnable(INT_PWM0_0);
    MAP_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);
    
    MAP_PWMIntEnable(PWM0_BASE, PWM_INT_GEN_1);
    
    /* Enable the PWM0 Bit 0 (PF0) and Bit 1 (PF1) output signals. */
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);

    /* Enables the counter for a PWM generator block. */
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

/* PWM ISR */
void PWM0_0_IRQHandler(void)
{
    uint32_t getIntStatus;

    getIntStatus = MAP_PWMGenIntStatus(PWM0_BASE, PWM_GEN_0, true);

    MAP_PWMGenIntClear(PWM0_BASE, PWM_GEN_0, getIntStatus);

}


//void SetPwm ( int16_t pwm[MOTORSNUM] )
//{
//
//
//    TIM1->CCR4 = PWM_RADIO * ( pwm[0] ) + INIT_DUTY;				//1
//    TIM1->CCR3 = PWM_RADIO * ( pwm[1] ) + INIT_DUTY;				//2
//    TIM1->CCR2 = PWM_RADIO * ( pwm[2] ) + INIT_DUTY;				//3
//    TIM1->CCR1 = PWM_RADIO * ( pwm[3] ) + INIT_DUTY;				//4
//
//// 	TIM5->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[4]] ) + INIT_DUTY;				//5
//// 	TIM5->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[5]] ) + INIT_DUTY;				//6
//// 	TIM8->CCR4 = PWM_RADIO *( pwm_tem[CH_out_Mapping[6]] ) + INIT_DUTY;				//7
//// 	TIM8->CCR3 = PWM_RADIO *( pwm_tem[CH_out_Mapping[7]] ) + INIT_DUTY;				//8
//
//}


/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
