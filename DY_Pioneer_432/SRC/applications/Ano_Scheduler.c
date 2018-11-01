/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：任务调度
**********************************************************************************/
#include "Ano_Scheduler.h"
#include "include.h"
#include "Ano_RC.h"
#include "Ano_Parameter.h"
#include "Drv_time.h"
#include "Drv_led.h"
#include "Drv_pwm_in.h"
#include "Drv_icm20602.h"
#include "Drv_ak8975.h"
#include "Drv_spl06.h"
#include "Ano_FlightCtrl.h"
#include "Ano_FlightDataCal.h"
#include "Ano_AttCtrl.h"
#include "Ano_Imu.h"
#include "Drv_vl53l0x.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Ano_Parameter.h"
#include "Ano_MagProcess.h"
#include "Ano_Power.h"
#include "Ano_OF.h"

#include "DY_Flight_Log.h"

//#include "Drv_usart.h"

u32 test_dT_1000hz[3],test_rT[6];

static void Loop_1000Hz(void)	//1ms执行一次
{
	test_dT_1000hz[0] = test_dT_1000hz[1];
	test_rT[3] = test_dT_1000hz[1] = GetSysTime_us ();
	test_dT_1000hz[2] = (u32)(test_dT_1000hz[1] - test_dT_1000hz[0]) ;
//////////////////////////////////////////////////////////////////////	
	/*传感器数据读取*/
	Fc_Sensor_Get();		//ICM20602+AK8975+SPL06原始数据（未处理）
	
	/*惯性传感器数据准备*/
	Sensor_Data_Prepare(1);		//输出==>sensor.Acc_cmss[i]加速度计：cm/s^2	sensor.Gyro_rad[i]陀螺仪：rad/s	sensor.Gyro_deg[i]陀螺仪：°/s
	
	/*姿态解算更新*/
	IMU_Update_Task(1);		//姿态计算，勿动	;输出==>imu->w，imu->x，imu->y，imu->z
	
	/*获取WC_Z加速度*/
	WCZ_Acc_Get_Task();		//WC_Z加速度，勿动	;输出==>wcz_acc_use
	
	/*飞行状态任务*/
	Flight_State_Task(1,CH_N);		//飞行状态，	fs.speed_set_h[X],fs.speed_set_h[Y],fs.speed_set_h[Z]
	
	/*开关状态任务*/
	Swtich_State_Task(1);		//光流模块以及激光测距模块，检测是否可用
	
	/*姿态角速度环控制*/
	Att_1level_Ctrl(1e-3f);		//输出==>mc.ct_val_rol,mc.ct_val_pit,mc.ct_val_yaw
	
	/*电机输出控制*/
	Motor_Ctrl_Task(1);		//输出==>motor[i]
		
	/*数传数据交换*/
	ANO_DT_Data_Exchange();
	
//////////////////////////////////////////////////////////////////////	
	test_rT[4]= GetSysTime_us ();
	test_rT[5] = (u32)(test_rT[4] - test_rT[3]) ;	
}

static void Loop_500Hz(void)	//2ms执行一次
{	
	/*刘德祥加Debug模式*/
	DY_Flight_Control();
	
	/*刘德祥加飞行计划*/
	DY_Flight_Plan();
}

static void Loop_200Hz(void)	//5ms执行一次
{

	
}

static void Loop_100Hz(void)	//10ms执行一次
{
			test_rT[0]= GetSysTime_us ();
//////////////////////////////////////////////////////////////////////				
	/*遥控器数据处理*/
	RC_duty_task(10);		//遥控器数据处理	;输出==>CH_N[i]
	
	/*飞行模式设置任务*/
	Flight_Mode_Set(10);		//飞行模式,3种模式（实际2中，姿态模式与定高悬停，第5通道）,一键起飞（第7通道）
	
	/*获取姿态角（ROLL PITCH YAW）*/
	calculate_RPY();		//输出==>imu_data.pit,imu_data.rol,imu_data.yaw
	
	/*姿态角度环控制*/
	Att_2level_Ctrl(10e-3f,CH_N);		//输出==>val_2[i].out
	
	/*位置速度环控制（暂无）*/
	Loc_1level_Ctrl(10,CH_N);		//输出==>loc_ctrl_1.out[X],loc_ctrl_1.out[Y]
	
	/*高度数据融合任务*/
	WCZ_Fus_Task(10);		//输出==>wcz_acc_fus,wcz_spe_fus,wcz_hei_fus
	
	/*高度速度环(内环)控制*/
	Alt_1level_Ctrl(10e-3f);		//输出==>loc_ctrl_1.out[Z],mc.ct_val_thr
	
	/*高度环(外环)控制*/
	Alt_2level_Ctrl(10e-3f);		//输出==>alt_val_2.out
	
	/*--*/	
	AnoOF_DataAnl_Task(10);		//光流模块检测	;输出==>sens_hd_check.of_ok

	/*灯光控制*/	
	LED_Task(10);
//////////////////////////////////////////////////////////////////////		
			test_rT[1]= GetSysTime_us ();
			test_rT[2] = (u32)(test_rT[1] - test_rT[0]) ;	
				
}

static void Loop_50Hz(void)	//20ms执行一次
{	
	/*罗盘数据处理任务*/
	Mag_Update_Task(20);		//AK8975校准	;输出==>mag_val[i]（校准之后）


}

static void Loop_20Hz(void)	//50ms执行一次
{	
	/*TOF激光任务*/
	Drv_Vl53_RunTask();		//VL53L0X激光测距模块	;输出==>sens_hd_check.tof_ok，tof_height_mm（高度信息）
	/*电压相关任务*/
	Power_UpdateTask(50);		//A/D转换（电池电压），勿动
}

static void Loop_2Hz(void)	//500ms执行一次
{
	/*延时存储任务*/
	Ano_Parame_Write_task(500);		//FLash存储，勿动
	printf("OF_DX2:%d\r\n",OF_DX2);
	printf("OF_DY2:%d\r\n",OF_DY2);
	printf("OF_ALT:%d\r\n",OF_ALT);

}
//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] = 
{
	{Loop_1000Hz,1000,  0, 0},
	{Loop_500Hz , 500,  0, 0},
	{Loop_200Hz , 200,  0, 0},
	{Loop_100Hz , 100,  0, 0},
	{Loop_50Hz  ,  50,  0, 0},
	{Loop_20Hz  ,  20,  0, 0},
	{Loop_2Hz   ,   2,  0, 0},
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))		//7

void Scheduler_Setup(void)
{
	uint8_t index = 0;
	//初始化任务表
	for(index=0;index < TASK_NUM;index++)
	{
		//计算每个任务的延时周期数
		sched_tasks[index].interval_ticks = TICK_PER_SECOND/sched_tasks[index].rate_hz;		//1ms,2,5,10,20,50,500
		//最短周期为1，也就是1ms
		if(sched_tasks[index].interval_ticks < 1)
		{
			sched_tasks[index].interval_ticks = 1;
		}
	}
}
//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	//循环判断所有线程，是否应该执行

	
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = SysTick_GetTick();
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			
			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();

		}	 
	}
	

}



/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
	

