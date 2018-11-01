/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * ����    �������ƴ�
 * ����    ��www.anotc.com
 * �Ա�    ��anotc.taobao.com
 * ����QȺ ��190169595
 * ����    ��VL53ģ���ȡ
**********************************************************************************/
#include "drv_vl53l0x.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"
#include "Ano_FcData.h"

u8 VL53L0X_LINKOK = 0;

u16 tof_height_mm;
void Drv_Vl53_Init(void)
{
	if(VL53L0X_Init() == VL53L0X_ERROR_NONE)
	{
		VL53L0X_LINKOK = 1;
		I2C_FastMode = 1;
	}
	else
	{
		VL53L0X_LINKOK = 0;
	}
	sens_hd_check.tof_ok = VL53L0X_LINKOK;
//	printf("sens_hd_check.tof_ok:%d\r\n",sens_hd_check.tof_ok);
}
static u16 err_delay;
void Drv_Vl53_RunTask(void)
{
	if(!VL53L0X_LINKOK)		//VL53L0Xģ����
		return;
	
	//if(VL53L0X_IfDataReady())
	//{
		tof_height_mm = VL53L0X_FastRead();
	//}
	
//	printf("tof_height_mm:%d\r\n",tof_height_mm);
	
	if(tof_height_mm == 0)
	{
		err_delay++;
		if(err_delay>10)		//>500ms������ΪVL53L0Xģ�����
		{
			sens_hd_check.tof_ok = VL53L0X_LINKOK = 0;
		}
	}
	else
	{
		err_delay = 0;
	}
}


