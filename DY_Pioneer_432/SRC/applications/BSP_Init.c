/******************** (C) COPYRIGHT 2018 DY EleSc ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：飞控初始化
**********************************************************************************/
#include "include.h"

#include "Drv_pwm_out.h"
//#include "Drv_led.h"
//#include "Drv_spi.h"
//#include "Drv_icm20602.h"
//#include "drv_ak8975.h"
//#include "drv_spl06.h"
//#include "Drv_w25qxx.h"
//#include "Drv_i2c_soft.h"
//#include "drv_vl53l0x.h"
//#include "Ano_FlightCtrl.h"
//#include "Drv_adc.h"

u8 All_Init()
{        
//	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置		3位抢占优先级（0~7），1位响应优先级
  
//        /* Enable the clock to the GPIO Port N and wait for it to be ready */
//        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
//        while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)))
//        {
//
//        }
//
//        /* Configure the GPIO PN0 as output */
//        MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
//        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);

	SysTick_Configuration(); 		//滴答时钟	滴答定时器是一个24位的倒计数定时器，当计到0时，将从RELOAD寄存器中自动重装载定时器初值，只要不把他在SysTick控制以及
						//			状态寄存器中的使能位清零，就将永久不息。SysTick的最大使命，就是定期的产生异常请求作为系统的时基。
        //
        // Configure the device pins.
        //
        PinoutSet(false, false);
        
	uart_init(115200);		        //串口0初始化
//	
//	Drv_LED_Init();					//LED功能初始化	PE3、PE2、PE1、PE0
//	
//	Flash_Init();             		//板载FLASH芯片(W25Q32)驱动初始化	5.25MHz
//	
	Para_Data_Init();		        // 参数数据初始化
//	
//	PWM_IN_Init();					//初始化接收机采集功能	输入捕获

	PWM_Out_Init();				//初始化电调输出功能		PWM输出	
	
//	Drv_SPI2_init();          		//SPI2初始化，用于读取飞控板上所有传感器，都用SPI读取
//	Drv_Icm20602CSPin_Init(); 		//SPI片选初始化
//	Drv_AK8975CSPin_Init();   		//SPI片选初始化
//
//	Drv_SPL06CSPin_Init();    		//SPI片选初始化
//	sens_hd_check.gyro_ok = sens_hd_check.acc_ok = 
//	Drv_Icm20602Reg_Init();   		//ICM20602陀螺仪加速度计初始化，若初始化成功，则将陀螺仪和加速度的初始化成功标志位赋值
//	sens_hd_check.mag_ok = 1;       //标记罗盘OK	
//	sens_hd_check.baro_ok = Drv_Spl0601_Init();       		//气压计初始化
//
//	Usb_Hid_Init();					//飞控USB接口的HID（Human Interface Device）初始化
//	Delay_ms(400);					//延时（可以实现us级延时，不准确）
//
//	Usart2_Init(500000);			//串口2初始化，函数参数为波特率	500Kbps		抢占优先级2，响应优先级0		匿名数传
//	Uart4_Init(500000);		//抢占优先级2，响应优先级1	匿名光流
//	
//	I2c_Soft_Init();          		//软件I2C初始化，因为飞控可以外接匿名激光定高模块，需要保留一个IIC，读取外置传感器
//	Drv_Vl53_Init();          		//TOF模块初始化，使用VL53L0X的激光测高，代替原有的超声波定高，效果更好	串口3
//
//	Drv_AdcInit();		//A/D采集电池电压
//	Delay_ms(100);					//延时
//
//	All_PID_Init();               		//PID初始化
	
//	ANO_DT_SendString("SYS init OK!",sizeof("SYS init OK!"));		//系统初始化完毕
	return (1);
}
/******************* (C) COPYRIGHT 2018 DY EleSc *****END OF FILE************/
