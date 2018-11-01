#include "DY_Flight_Log.h"
#include "Ano_AltCtrl.h"
#include "Ano_AttCtrl.h"
#include "Ano_FlightCtrl.h"
#include "Ano_Math.h"
#include "Drv_time.h"

_DY_flag dy_flag;
_DY_Flight_Plan dy_flight_plan;

void DY_Flight_Control(void)
{
	/***************控制变量初始化***************/
//	dy_height = 0;
//	dy_pit = 0;
//	dy_rol = 0;
//	dy_yaw = 0.0f;
	
	/***************飞行控制***************/
	if(dy_flag.pit_go == 1)
	{
		dy_flag.pit_go = 0;
		dy_height = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
		dy_pit += 10;
	}
	if(dy_flag.pit_back == 1)
	{
		dy_flag.pit_back = 0;
		dy_height = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
		dy_pit -= 10;
	}
	if(dy_flag.rol_left == 1)
	{
		dy_flag.rol_left = 0;
		dy_height = 0;
		dy_pit = 0;
		dy_yaw = 0.0f;
		dy_rol += 10;
	}
	if(dy_flag.rol_right == 1)
	{
		dy_flag.rol_right = 0;
		dy_height = 0;
		dy_pit = 0;
		dy_yaw = 0.0f;
		dy_rol -= 10;
	}
	if(dy_flag.yaw_ccw == 1)
	{
		dy_flag.yaw_ccw = 0;
		dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw -= 2.0f;
	}
	if(dy_flag.yaw_cw == 1)
	{
		dy_flag.yaw_cw = 0;
		dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw += 2.0f;
	}
	if(dy_flag.up == 1)
	{
		dy_flag.up = 0;
		dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
		dy_height += 10;
	}
	if(dy_flag.down == 1)
	{
		dy_flag.down = 0;
		dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
		dy_height -= 10;
	}
	if(dy_flag.stop == 1)
	{
		dy_flag.stop = 0;
		dy_height = 0;
		dy_pit = 0;
		dy_rol = 0;
		dy_yaw = 0.0f;
	}

//	dy_pit = LIMIT(dy_pit,-20,20);
//	dy_rol = LIMIT(dy_rol,-20,20);
//	dy_yaw = LIMIT(dy_yaw,-4.0f,4.0f);
//	dy_height = LIMIT(dy_height,0,20);
}

void DY_Flight_Plan(void)
{
	static u32 start_time = 0;
	if(dy_flight_plan.plan1 == 1)
	{
		start_time++;
		if(start_time == 1000)
		{
			one_key_take_off();
		}
		if(start_time == 6000)
		{
			dy_flag.pit_go = 1;
		}
		if(start_time == 8500)
		{
			dy_flag.rol_right = 1;
		}
		if(start_time == 10500)
		{
			dy_flag.stop = 1;
		}
		if(start_time == 11000)
		{
			dy_flag.up = 1;
		}
		if(start_time == 11500)
		{
			dy_flag.stop = 1;
		}
		if(start_time == 12000)
		{
			dy_flag.yaw_cw = 1;
		}
		if(start_time == 12500)
		{
			dy_flag.stop = 1;
		}
		if(start_time == 13000)
		{
			dy_flag.pit_back = 1;
		}
		if(start_time == 15500)
		{
			dy_flag.stop = 1;
		}
		if(start_time == 16000)
		{
			one_key_land();
			dy_flight_plan.plan1 = 0;
			start_time = 0;
		}
	}
	
	if(dy_flight_plan.plan2 == 1)
	{
		
	}
	
	if(dy_flight_plan.plan3 == 1)
	{
		
	}
		
}
