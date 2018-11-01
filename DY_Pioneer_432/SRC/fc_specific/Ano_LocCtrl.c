#include "Ano_LocCtrl.h"
#include "Ano_Imu.h"
#include "Ano_FlightCtrl.h"
#include "Ano_OF.h"
#include "Ano_Parameter.h"


//位置速度环控制参数
_PID_arg_st loc_arg_1[2] ; 

//位置速度环控制数据
_PID_val_st loc_val_1[2] ; 

//位置速度环修正控制参数
_PID_arg_st loc_arg_1_fix[2] ; 

//位置速度环修正控制数据
_PID_val_st loc_val_1_fix[2] ; 

/*角度环PID参数初始化*/
void Loc_1level_PID_Init()
{
	loc_arg_1[X].kp = Ano_Parame.set.pid_loc_1level[KP];//0.22f  ;
	loc_arg_1[X].ki = 0.0f  ;
	loc_arg_1[X].kd_ex = 0.00f ;
	loc_arg_1[X].kd_fb = Ano_Parame.set.pid_loc_1level[KD];
	loc_arg_1[X].k_ff = 0.02f;
	
	loc_arg_1[Y] = loc_arg_1[X];
//fix	
	loc_arg_1_fix[X].kp = 0.0f  ;
	loc_arg_1_fix[X].ki = Ano_Parame.set.pid_loc_1level[KI] ;
	loc_arg_1_fix[X].kd_ex = 0.00f;
	loc_arg_1_fix[X].kd_fb = 0.00f;
	loc_arg_1_fix[X].k_ff = 0.0f;
	
	loc_arg_1_fix[Y] = loc_arg_1_fix[X];
}

_loc_ctrl_st loc_ctrl_1;
static float fb_speed_fix[2];
/*位置速度环*/
void Loc_1level_Ctrl(u16 dT_ms,s16 *CH_N)
{
	if(switchs.of_flow_on)		//光流数据有效
	{

		loc_ctrl_1.exp[X] = fs.speed_set_h[X];
		loc_ctrl_1.exp[Y] = fs.speed_set_h[Y];

		loc_ctrl_1.fb[X] = OF_DX2;
		loc_ctrl_1.fb[Y] = OF_DY2;
		
		fb_speed_fix[0] = OF_DX2FIX;
		fb_speed_fix[1] = OF_DY2FIX;
		
		for(u8 i =0;i<2;i++)
		{
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										loc_ctrl_1.fb[i] ,			//反馈值（）
										&loc_arg_1[i], //PID参数结构体
										&loc_val_1[i],	//PID数据结构体
										50,//积分误差限幅
										10 *flag.taking_off			//integration limit，积分限幅
										 )	;		//输出==>pid_val->out
			
			//fix
			PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
										loc_ctrl_1.exp[i] ,				//前馈值
										loc_ctrl_1.exp[i] ,				//期望值（设定值）
										fb_speed_fix[i] ,			//反馈值（）
										&loc_arg_1_fix[i], //PID参数结构体
										&loc_val_1_fix[i],	//PID数据结构体
										50,//积分误差限幅
										10 *flag.taking_off			//integration limit，积分限幅
										 )	;	
			
			loc_ctrl_1.out[i] = loc_val_1[i].out + loc_val_1_fix[i].out;		
		}		


	}
	else
	{
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[X] ;
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[Y] ;
	}
}

_loc_ctrl_st loc_ctrl_2;

