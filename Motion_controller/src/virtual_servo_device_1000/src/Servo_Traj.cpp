/*
 * Servo_Traj.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */
#include "Servo_Traj.h"
#include <math.h>

#define INSERT_POINT_NUM 5

using namespace virtual_servo_device;

static SERVO_TRAJ_PARA_INFO traj_target_info[AXIS_MAX];
static SERVO_TRAJ_PARA_UPDATE_INFO traj_update_info[AXIS_MAX];
static SERVO_TRAJ_TIME_COUNT traj_var_info[AXIS_MAX];
/*-----------------------------------------------------------------------
 * 函数名称: void Servo_Traj_Init(void)
 * 函数输出: none
 * 函数输入: none
 * 函数备注: 速
 * ------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Traj_Init(void)
{
	for(uint32_t i=0; i<AXIS_MAX; i++)
	{
		traj_var_info[i].internal_step = 0;
		traj_var_info[i].step_count = 0;
	}
}
/*--------------------------------------------------------------------------
 * 函数名称: void Servo_Traj_Set_MotionPara(SERVO_AXIS_ENUM axis_id,
		const SERVO_TRAJ_PARA_INFO* para_ptr,
		int64_t current_pos)
 * 函数输出: none
 * 函数输入: none
 * 函数备注: 速
 * ------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Traj_Set_ContinueMotion(SERVO_AXIS_ENUM axis_id,
									int32_t set_lsb,
									int32_t set_msb,
									int32_t vel_ff,
									int64_t current_pos)
{
	int64_t temp_lsb = set_lsb;
	int64_t temp_msb = set_msb;
	traj_target_info[axis_id].set_position = (temp_msb<<32)|temp_lsb;

	traj_target_info[axis_id].set_position = traj_target_info[axis_id].set_position<<8;
	traj_update_info[axis_id].pos_offset  = current_pos;

	traj_var_info[axis_id].internal_step = (int64_t)((traj_target_info[axis_id].set_position	\
													-current_pos)		\
												/INSERT_POINT_NUM);
	traj_var_info[axis_id].step_count = 0;
	traj_var_info[axis_id].update_flag = 1;
}
/*--------------------------------------------------------------------------
 * 函数名称: void Servo_Traj_Get_TrajPtr(SERVO_AXIS_ENUM axis_id,
		SERVO_TRAJ_PARA_UPDATE_INFO** get_traj_ptr)
 * 函数输出: none
 * 函数输入: none
 * 函数备注: 速
 * ------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Traj_Get_TrajPtr(SERVO_AXIS_ENUM axis_id,
		SERVO_TRAJ_PARA_UPDATE_INFO** get_traj_ptr)
{
	*get_traj_ptr=&traj_update_info[axis_id];
}
/*--------------------------------------------------------------------------
 * 函数名称: void Servo_Traj_p_UpdatePoint(SERVO_AXIS_ENUM axis_id)
 * 函数输出: none
 * 函数输入: none
 * 函数备注: 速
 * ------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Traj_p_UpdatePoint(SERVO_AXIS_ENUM axis_id)
{
	if(traj_var_info[axis_id].update_flag)
	{
		traj_var_info[axis_id].step_count++;

		if(traj_var_info[axis_id].step_count<=INSERT_POINT_NUM)
		{
			traj_update_info[axis_id].pos_update = (traj_var_info[axis_id].step_count
																		*traj_var_info[axis_id].internal_step)
																		 +traj_update_info[axis_id].pos_offset;
			if(traj_var_info[axis_id].step_count==INSERT_POINT_NUM)
			{
				if(traj_update_info[axis_id].pos_update!=(traj_target_info[axis_id].set_position))
				{
					traj_update_info[axis_id].pos_update = traj_target_info[axis_id].set_position;
				}
				traj_var_info[axis_id].update_flag = 0;
				traj_var_info[axis_id].step_count = 0;
				traj_update_info[axis_id].motion_flag = 0;
			}
		}
	}
}

