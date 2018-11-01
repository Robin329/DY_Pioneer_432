#include "Ano_FlightCtrl.h"
#include "Ano_Imu.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_spl06.h"
#include "Ano_MotionCal.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Drv_led.h"
#include "Ano_RC.h"
#include "Drv_vl53l0x.h"
#include "Ano_OF.h"

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////

/*PID参数初始化*/
void All_PID_Init(void)
{

	/*姿态控制，角速度PID初始化*/
	Att_1level_PID_Init();
	
	/*姿态控制，角度PID初始化*/
	Att_2level_PID_Init();
	
	/*高度控制，高度速度PID初始化*/
	Alt_1level_PID_Init();	
	
	/*高度控制，高度PID初始化*/
	Alt_2level_PID_Init();
	
	
	/*位置速度控制PID初始化*/
	Loc_1level_PID_Init();
	
}

/*控制参数改变任务*/
void ctrl_parameter_change_task()
{

	
	if(0)
	{
		Set_Att_2level_Ki(0);
		
	}
	else
	{
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
		{

			Set_Att_1level_Ki(2);
		}
		else
		{

			Set_Att_1level_Ki(1);
		}
		
		Set_Att_2level_Ki(1);
	}
}


/*一键翻滚（暂无）*/
void one_key_roll()
{

			if(flag.flying && flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH)
			{	
				if(rolling_flag.roll_mode==0)
				{
					rolling_flag.roll_mode = 1;
					
				}
			}

}

static u16 one_key_taof_start;
/*一键起飞任务（主要功能为延迟）*/
void one_key_take_off_task(u16 dt_ms)
{
	if(one_key_taof_start != 0)
	{
		one_key_taof_start += dt_ms;
		
		
		if(one_key_taof_start > 1400 && flag.motor_preparation == 1)		//1400*10=14000ms=14s
		{
			one_key_taof_start = 0;
				if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
				{
					flag.auto_take_off_land = AUTO_TAKE_OFF;
					//解锁、起飞

					flag.taking_off = 1;
				}
			
		}
	}
	

}
/*一键起飞*/
void one_key_take_off()
{
	if(flag.unlock_en)
	{	
		one_key_taof_start = 1;
		flag.fly_ready = 1;
	}
}
/*一键降落*/
void one_key_land()
{
	flag.auto_take_off_land = AUTO_LAND;
}

//////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////
_flight_state_st fs;

s16 flying_cnt,landing_cnt;

extern s32 ref_height_get;

float stop_baro_hpf;

/*降落检测*/

static s16 ld_delay_cnt ;
void land_discriminat(s16 dT_ms)
{
//	static s16 acc_delta,acc_old;
	
//	acc_delta = imu_data.w_acc[Z]- acc_old;
//	acc_old = imu_data.w_acc[Z];
	
	/*油门归一值小于0.1并且垂直方向加速度小于阈值  或者启动自动降落*/
	if((fs.speed_set_h_norm[Z] < 0.1f && imu_data.w_acc[Z]<200) || flag.auto_take_off_land == AUTO_LAND)
	{
		if(ld_delay_cnt>0)		//200ms
		{
			ld_delay_cnt -= dT_ms;
		}
	}
	else
	{
		ld_delay_cnt = 200;
	}
	
	/*意义是：如果向上推了油门，就需要等垂直方向加速度小于200cm/s2 保持200ms才开始检测*/	
	if(ld_delay_cnt <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND) )
	{
		/*油门最终输出量小于250并且没有在手动解锁上锁过程中，持续1秒，认为着陆，然后上锁*/
		if(mc.ct_val_thr<250 && flag.fly_ready == 1 && flag.locking != 2)//ABS(wz_spe_f1.out <20 ) //还应当 与上速度条件，速度小于正20厘米每秒。
		{
			if(landing_cnt<1500)		//1500ms = 1.5s
			{
				landing_cnt += dT_ms;
			}
			else
			{

				flying_cnt = 0;
				flag.taking_off = 0;		//
				///////
					landing_cnt =0;	
					flag.fly_ready =0;		//上锁		

				flag.flying = 0;		//

			}
		}
		else
		{
			landing_cnt = 0;
		}
			
		
	}
	else
	{
		landing_cnt  = 0;
	}

}


/*飞行状态任务*/
/***************PIT、ROL控制变量初始化***************/
s16 dy_pit = 0,dy_rol = 0;

void Flight_State_Task(u8 dT_ms,s16 *CH_N)
{
	s16 thr_deadzone;
	static float max_speed_lim;
	/*设置油门摇杆量*/
	thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;		//if (flag.wifi_ch_en) != 0,(thr_deadzone) = 0;other (thr_deadzone) = 50 ，死区
	fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR],0,thr_deadzone) *0.0023f;		//-1.15~1.15	;-1.035~1.035	油门归一值
	fs.speed_set_h_norm_lpf[Z] += 0.2f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);
	
	/*推油门起飞*/
	if(flag.fly_ready)		//已解锁
	{	
		if(fs.speed_set_h_norm[Z]>0.01f && flag.motor_preparation == 1) // 0-1
		{
			flag.taking_off = 1;		//起飞
		}	
	}		
	
	if(flag.taking_off)		//起飞
	{
			
		if(flying_cnt<1000)//800ms		；1000ms = 1s
		{
			flying_cnt += dT_ms;
		}
		else
		{
			/*起飞后1秒，认为已经在飞行*/
			flag.flying = 1;  	//飞行
		}
		
		if(fs.speed_set_h_norm[Z]>0)
		{
			/*设置上升速度*/
			fs.speed_set_h[Z] = fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP;		//350cm/s
		}
		else
		{
			/*设置下降速度*/
			fs.speed_set_h[Z]  = fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW;	//300cm/s
		}
	}
	else
	{
		fs.speed_set_h[Z] = 0 ;
	}
	float speed_set_tmp[2];
	/*速度设定量，正负参考ANO坐标参考方向*/
	fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT],0,50) *0.0022f);		//-0.99~0.99
	fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL],0,50) *0.0022f);		//-0.99~0.99
		
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]);		//输出==>fs.speed_set_h_norm_lpf[X]
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);		//输出==>fs.speed_set_h_norm_lpf[Y]
	
	max_speed_lim = MAX_SPEED;		//最大水平速度 500cm/s
	
	if(switchs.of_flow_on)		//使用光流
	{
		max_speed_lim = 1.5f *wcz_hei_fus.out;
		max_speed_lim = LIMIT(max_speed_lim,50,150);		//限幅 50~150
	}	
	
	speed_set_tmp[X] = max_speed_lim *fs.speed_set_h_norm_lpf[X];
	speed_set_tmp[Y] = max_speed_lim *fs.speed_set_h_norm_lpf[Y];
	

	
	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],max_speed_lim,fs.speed_set_h_cms);		//输出==>fs.speed_set_h_cms[X],fs.speed_set_h_cms[Y]

/***************刘德祥修改——Debug模式***************/
	if(DY_Debug_Mode == 1)
	{
		fs.speed_set_h[X] = dy_pit;
		fs.speed_set_h[Y] = dy_rol;
	}
	else
	{
		fs.speed_set_h[X] = fs.speed_set_h_cms[X];
		fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];
	}
		
//#if 1
//	
//	fs.speed_set_h[X] = fs.speed_set_h_cms[X];
//	fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];	
//	
//#else
//	
//	if(CH_N[AUX2]<-200)		//-500`-200
//	{
//		fs.speed_set_h[X] = dy_pit;		//5
//		fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];	
//	}
//	else if(CH_N[AUX2]<200)		//-200~200
//	{
//		fs.speed_set_h[X] = fs.speed_set_h_cms[X];
//		fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];	
//	}
//	else		//200~500
//	{
//		fs.speed_set_h[X] = dy_pit;		//-5
//		fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];	
//	}
//	
//#endif
/******************************/
	/*调用检测着陆的函数*/
	land_discriminat(dT_ms);
	
	/*倾斜过大上锁*/
	if(rolling_flag.rolling_step == ROLL_END)
	{
		if(imu_data.z_vec[Z]<0.25f)//75度  ////////////////////////////////////////*************************** 倾斜过大上锁，慎用。
		{

			flag.fly_ready = 0;
		}

	}	
		//////////////////////////////////////////////////////////
	/*校准中，复位重力方向*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_state.G_reset = 1;
	}
	
	/*复位重力方向时，认为传感器失效*/
	if(imu_state.G_reset == 1)
	{
		flag.sensor_ok = 0;
		WCZ_Data_Reset(); //复位高度数据融合
	}
	else if(imu_state.G_reset == 0)
	{	
		if(flag.sensor_ok == 0)
		{
			flag.sensor_ok = 1;
//			ANO_DT_SendString("IMU OK!",sizeof("IMU OK!"));
		}
	}
	
	/*飞行状态复位*/
	if(flag.fly_ready == 0)		//锁定
	{
		flag.flying = 0;
		landing_cnt = 0;
		flag.taking_off = 0;
		flying_cnt = 0;
		
		//复位融合
		if(flag.taking_off == 0)
		{
//			wxyz_fusion_reset();
		}
	}
	

}


//////////////////////////////////////////////////////////////////////////////
static void LED_Switch()
{
	if( (LED_state != 0 && LED_state <= 115) )
	{
		return;
	}
	
	if(flag.flight_mode == ATT_STAB)
	{
		if(flag.fly_ready)
		{
			LED_state = 131;
		}
		else
		{
			LED_state = 121;
		}
	}
	else if(flag.flight_mode == LOC_HOLD)
	{
		if(flag.fly_ready)
		{
			LED_state = 132;
		}
		else
		{
			LED_state = 122;
		}		
	}
	else if(flag.flight_mode == RETURN_HOME)
	{
		if(flag.fly_ready)
		{
			LED_state = 133;
		}
		else
		{
			LED_state = 123;
		}	
	}

}

static u8 of_light_ok;
static u16 of_light_delay;
void Swtich_State_Task(u8 dT_ms)
{
	switchs.baro_on = 1;
	

	if(sens_hd_check.of_ok)//光流模块
	{
		if(OF_LIGHT>20 || flag.flying == 0) //光流亮度大于20或者在飞行之前，认为光流可用，判定可用延迟时间为1秒
		{
			if(of_light_delay<1000)
			{
				of_light_delay += dT_ms;
			}
			else
			{
				of_light_ok = 1;
			}
		}
		else
		{
			of_light_delay =0;
			of_light_ok = 0;
		}
		
		if(OF_ALT<1900 && flag.flight_mode == LOC_HOLD)
		{		
			if(of_light_ok)
			{
				if(switchs.of_flow_on == 0)
				{
					LED_state = 13 ;//切换指示触发1下（1闪蓝）
				}
				switchs.of_flow_on = 1;
			}
			else
			{
				if(switchs.of_flow_on )
				{
					LED_state = 23 ;//切换指示触发1下（1闪红）
				}
				switchs.of_flow_on = 0;
			}
			
			if(switchs.of_tof_on == 0)
			{
				LED_state = 14 ;//切换指示触发1下（2闪蓝）
			}
			switchs.of_tof_on = 1;
		}
		else
		{
			if(switchs.of_tof_on )
			{
				LED_state =  24 ;//切换指示触发1下（2闪红）
			}
			switchs.of_tof_on = 0;
			
			
			switchs.of_flow_on = 0;
		}			
	}
	else
	{
		switchs.of_flow_on = switchs.of_tof_on = 0;
	}
	
	if(sens_hd_check.tof_ok)//激光模块
	{
		if(tof_height_mm<1900)
		{
			if(switchs.tof_on == 0)
			{
				LED_state = 14 ;//切换指示触发1下（2闪蓝）
			}
			switchs.tof_on = 1;
		}
		else
		{
			if(switchs.tof_on )
			{
				LED_state = 24 ;//切换指示触发1下（2闪红）
			}
			switchs.tof_on = 0;
		}
	}
	else
	{
		switchs.tof_on = 0;
	}

}


static void Speed_Mode_Switch()
{
//	if( ubx_user_data.s_acc_cms > 60)// || ubx_user_data.svs_used < 6)
//	{
//		flag.speed_mode = 0;
//	}
//	else
//	{
//		flag.speed_mode = 1;
//	}

}

u8 speed_mode_old = 255;
u8 flight_mode_old = 255;

u8 DY_Debug_Mode = 0;

void Flight_Mode_Set(u8 dT_ms)
{
	LED_Switch();
	Speed_Mode_Switch();

	
	if(speed_mode_old != flag.speed_mode) //状态改变
	{
		speed_mode_old = flag.speed_mode;
		
		if(flag.speed_mode == 1)		//3种模式
		{
			LED_state = 13;
		}
		else if(flag.speed_mode == 2)
		{
			LED_state = 14;
		}
		else
		{
			LED_state = 15;
		}
		
		//xy_speed_pid_init(flag.speed_mode);////////////
	}

///////////////////////////////////////////////////////
	
	if(CH_N[AUX1]<-200)		//-500`-200
	{
		flag.flight_mode = ATT_STAB;		//姿态模式
	}
	else if(CH_N[AUX1]<200)		//-200~200
	{
		flag.flight_mode = LOC_HOLD;		//定高悬停
	}
	else		//200~500
	{
		flag.flight_mode = LOC_HOLD;		//定高悬停
	}
	
	
	if(flight_mode_old != flag.flight_mode) //状态改变
	{
		flight_mode_old = flag.flight_mode;
		


	}
	
	if(CH_N[AUX2]<-200)		//-500`-200
	{
		DY_Debug_Mode = 1;		//调试模式
	}
	else if(CH_N[AUX2]<200)		//-200~200
	{
		
	}
	else		//200~500
	{
		DY_Debug_Mode = 0;		//退出调试模式
	}
	
	if(CH_N[AUX3]<-200)		//-500`-200
	{
		one_key_take_off();		//一键起飞
	}
	else if(CH_N[AUX3]<200)		//-200~200
	{
		
	}
	else		//200~500
	{
		one_key_land();		//一键降落
	}
}

