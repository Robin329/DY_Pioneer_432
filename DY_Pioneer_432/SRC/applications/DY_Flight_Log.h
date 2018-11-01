#ifndef _DY_FLIGHT_LOG_
#define _DY_FLIGHT_LOG_

#include "stm32f4xx.h"

typedef struct{
	u8 up;
	u8 down;
	u8 pit_go;
	u8 pit_back;
	u8 rol_left;
	u8 rol_right;
	u8 yaw_cw;
	u8 yaw_ccw;
	u8 stop;
}_DY_flag;
extern _DY_flag dy_flag;

typedef struct{
	u8 plan1;
	u8 plan2;
	u8 plan3;
}_DY_Flight_Plan;
extern _DY_Flight_Plan dy_flight_plan;

void DY_Flight_Control(void);
void DY_Flight_Plan(void);

#endif
