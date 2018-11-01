/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
  * 作者   ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：参数配置等
**********************************************************************************/
#include "include.h"
#include "Drv_w25qxx.h"
#include "Ano_FlightCtrl.h"
#include "Drv_led.h"
#include "Drv_icm20602.h"


union Parameter Ano_Parame;
_parameter_state_st para_sta;


void PID_Rest()
{
//---	姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KP] = 3.5f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KI] = 0; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[ROL][KD] = 0.1f; //姿态控制角速度环PID参数
	
	Ano_Parame.set.pid_att_1level[PIT][KP] = 3.5f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[PIT][KI] = 0; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[PIT][KD] = 0.1f; //姿态控制角速度环PID参数
	
	Ano_Parame.set.pid_att_1level[YAW][KP] = 4.0f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[YAW][KI] = 0.f; //姿态控制角速度环PID参数
	Ano_Parame.set.pid_att_1level[YAW][KD] = 0.0f; //姿态控制角速度环PID参数
//---	姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KP] = 5.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KI] = 3.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[ROL][KD] = 0.f; //姿态控制角度环PID参数
	
	Ano_Parame.set.pid_att_2level[PIT][KP] = 5.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[PIT][KI] = 3.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[PIT][KD] = 0.f; //姿态控制角度环PID参数
	
	Ano_Parame.set.pid_att_2level[YAW][KP] = 5.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[YAW][KI] = 1.0f; //姿态控制角度环PID参数
	Ano_Parame.set.pid_att_2level[YAW][KD] = 0.8; //姿态控制角度环PID参数	
//---	高度控制高度速度环PID参数	
	Ano_Parame.set.pid_alt_1level[KP] = 4.0f;          //高度控制高度速度环PID参数
	Ano_Parame.set.pid_alt_1level[KI] = 4.0f;          //高度控制高度速度环PID参数
	Ano_Parame.set.pid_alt_1level[KD] = 0.05f;          //高度控制高度速度环PID参数
//---	高度控制高度环PID参数
	Ano_Parame.set.pid_alt_2level[KP] = 1.0f;           //高度控制高度环PID参数
	Ano_Parame.set.pid_alt_2level[KI] = 0;           //高度控制高度环PID参数
	Ano_Parame.set.pid_alt_2level[KD] = 0;           //高度控制高度环PID参数
//---	位置控制位置速度环PID参数	
	Ano_Parame.set.pid_loc_1level[KP] = 0.2f;          //位置控制位置速度环PID参数
	Ano_Parame.set.pid_loc_1level[KI] = 0.1f;          //位置控制位置速度环PID参数
	Ano_Parame.set.pid_loc_1level[KD] = 0.002f;          //位置控制位置速度环PID参数
//---	位置控制位置环PID参数
	Ano_Parame.set.pid_loc_2level[KP] = 0;           //位置控制位置环PID参数
	Ano_Parame.set.pid_loc_2level[KI] = 0;           //位置控制位置环PID参数
	Ano_Parame.set.pid_loc_2level[KD] = 0;           //位置控制位置环PID参数

	ANO_DT_SendString("PID reset!",sizeof("PID reset!"));
}


static void Parame_Copy_Para2fc()
{
	for(u8 i = 0;i<3;i++)
	{	
		save.acc_offset[i]		=	Ano_Parame.set.acc_offset[i];
		save.gyro_offset[i]		=	Ano_Parame.set.gyro_offset[i];
		save.mag_offset[i]		=	Ano_Parame.set.mag_offset[i];  
		save.mag_gain[i]		=	Ano_Parame.set.mag_gain[i];  
		
		Center_Pos_Set();
			
	}
}

static void Parame_Copy_Fc2para()
{

	for(u8 i = 0;i<3;i++)
	{	
		Ano_Parame.set.acc_offset[i]	=	save.acc_offset[i];		//加速度计零偏
		Ano_Parame.set.gyro_offset[i]	=	save.gyro_offset[i];		//陀螺仪零偏
		Ano_Parame.set.mag_offset[i]	=	save.mag_offset[i];  		//磁力计零偏
		Ano_Parame.set.mag_gain[i]		=	save.mag_gain[i];   		//磁力计校正比例
			
		
		//center_pos参数不需要反向赋值
	}
}

void Parame_Reset(void)
{
	//参数初始化
	Ano_Parame.set.pwmInMode = PWM;		//PWM模式
	Ano_Parame.set.warn_power_voltage = 3.55f *3;		//警告电压	10.65V
	Ano_Parame.set.return_home_power_voltage = 3.7f *3;		//返航电压	11.1V
	Ano_Parame.set.lowest_power_voltage = 3.5f *3;		//最低电压	10.5V
	
	Ano_Parame.set.auto_take_off_height = 0;//cm	自动起飞高度 0cm
	
	for(u8 i = 0;i<3;i++)
	{
		Ano_Parame.set.acc_offset[i] = 0;		//加速度计零偏
		Ano_Parame.set.gyro_offset[i] = 0;		//陀螺仪零偏
		Ano_Parame.set.mag_offset[i] = 0;  		//磁力计零偏
		Ano_Parame.set.mag_gain[i] = 1;		//磁力计校正比例
		
		Ano_Parame.set.center_pos_cm[i] = 0;		//重心相对传感器位置偏移量
	}
	
	Parame_Copy_Para2fc();
		
	ANO_DT_SendString("parameter reset!",sizeof("parameter reset!"));
}



static void Ano_Parame_Write(void)
{
	All_PID_Init();	//////存储PID参数后，重新初始化PID	
	Ano_Parame.set.frist_init = SOFT_VER;	//32	初始化完毕

	Parame_Copy_Fc2para();
	
	Flash_SectorErase ( 0x000000, 1 );							//擦除第一扇区
	Flash_SectorsWrite ( 0x000000, &Ano_Parame.byte[0], 1 );	//将参数写入第一扇区
}

void Ano_Parame_Read(void)
{
	Flash_SectorsRead ( 0x000000, &Ano_Parame.byte[0], 1 );		//读取第一扇区内的参数
	
	if(Ano_Parame.set.frist_init != SOFT_VER)	//32	内容没有被初始化，则进行参数初始化工作
	{		
		Parame_Reset();		//传感器参数默认情况
		PID_Rest();		//PID参数默认情况
		Ano_Parame_Write();		//写入Flash
	}
	
	Parame_Copy_Para2fc();
	
	
}


void Ano_Parame_Write_task(u16 dT_ms)
{
	//因为写入flash耗时较长，我们飞控做了一个特殊逻辑，在解锁后，是不进行参数写入的，此时会置一个需要写入标志位，等飞机降落锁定后，再写入参数，提升飞行安全性
	//为了避免连续更新两个参数，造成flash写入两次，我们飞控加入一个延时逻辑，参数改变后一秒，才进行写入操作，可以一次写入多项参数，降低flash擦写次数
	if(para_sta.save_en )				//允许存储
	{
		if(para_sta.save_trig == 1) 	//如果触发存储标记1
		{
			LED_state = 17;
			
			para_sta.time_delay = 0;  	//计时复位
			para_sta.save_trig = 2;   	//触发存储标记2
		}
		
		if(para_sta.save_trig == 2) 	//如果触发存储标记2
		{
			if(para_sta.time_delay<3000) //计时小于3000ms=3s
			{
				para_sta.time_delay += dT_ms; //计时
			}
			else
			{
				
				para_sta.save_trig = 0;  //存储标记复位
				Ano_Parame_Write();      //执行存储
//				ANO_DT_SendString("Set save OK!",sizeof("Set save OK!"));
			}
		}
		else
		{
			para_sta.time_delay = 0;		//计时复位
		}
		
	}
	else
	{
		para_sta.time_delay = 0;
		para_sta.save_trig = 0;
	}
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
