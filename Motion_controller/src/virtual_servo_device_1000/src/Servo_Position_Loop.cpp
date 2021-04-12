/*
 * Servo_Position_Loop.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */
#include "Servo_General.h"
#include "Servo_Para.h"
#include "Servo_Position_Loop.h"
#define POS_SPEED_LIMIT 27682406
#define NEG_SPEED_LIMIT -27682406

using namespace virtual_servo_device;
static PARA_POSITION_LOOP_READ_INFO pos_alg_info[AXIS_MAX];

/*-------------------------------------------------------------
 *函数名称 :void MOLS_p_Cal_PosLoop(MO_AXIS_ENUM axis_id
						,MO_AXIS_INF_STRUCT *cal_pid_var)
 *函数输入 :
 *函数输出 :
 *函数描述 :轴框架数据初始化,速度限制值在4950
--------------------------------------------------------------*/
void virtual_servo_device::Servo_PosLoop_Init(void)
{

}
/*------------------------------------------------------------
 * 函数名称:int32_t Servo_PosLoop_SetPara(PARA_POSITION_LOOP_READ_INFO *para_cfg_ptr,
							   SERVO_AXIS_ENUM axis_id)
 * 函数输入:位置环参数指针，轴枚举
 * 函数输出:函数返回1设置成功，否则失败，参数使用默认值
 * 函数备注:设置的函数需要校验
 *---------------------------------------------------------------- */
 void virtual_servo_device::Servo_PosLoop_SetPara(SERVO_AXIS_ENUM axis_id,const PARA_POSITION_LOOP_READ_INFO *para_cfg_ptr)
{
	pos_alg_info[axis_id].kp_factor.para_value
								= para_cfg_ptr->kp_factor.para_value;

	printf("pos_kp=%d\r\n",pos_alg_info[axis_id].kp_factor.para_value);
}
/*-------------------------------------------------------------
 *函数名称 :void MOLS_p_Cal_PosLoop(MO_AXIS_ENUM axis_id
						,MO_AXIS_INF_STRUCT *cal_pid_var)
 *函数输入 :
 *函数输出 :
 *函数描述 :轴框架数据初始化,速度限制值在4950
--------------------------------------------------------------*/
void virtual_servo_device::Servo_PosLoop_p_Cal(SERVO_AXIS_ENUM axis_id
						,SERVO_AXIS_INF_STRUCT *cal_pid_var)
{
	cal_pid_var->kp_out = (cal_pid_var->ctl_err
							*pos_alg_info[axis_id].kp_factor.para_value)>>24;

	cal_pid_var->con_pid_out = cal_pid_var->kp_out;

	/*正向速度限制*/
	if(cal_pid_var->con_pid_out > POS_SPEED_LIMIT)
	{
		cal_pid_var->con_pid_out = POS_SPEED_LIMIT;
	}
	/*负向速度限制*/
	if(cal_pid_var->con_pid_out < NEG_SPEED_LIMIT)
	{
		cal_pid_var->con_pid_out = NEG_SPEED_LIMIT;
	}
}

