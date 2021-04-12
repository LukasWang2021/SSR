/*
 * Servo_Speed_Loop.c
 *
 *  Created on: 2019年7月29日
 *      Author: qichao.wang
 */
#include "Servo_General.h"
#include "Servo_Para.h"
#include "string.h"
#include "Servo_Traj.h"
#include "Servo_Speed_Loop.h"
#define KI_CAL_FACTOR 0.0002
#define ki_cal(type,x,y) ({type __x=(x); type __y=(y);(type)((__x*__y*KI_CAL_FACTOR));})

using namespace virtual_servo_device;


typedef struct
{
	int min_pos_out;
	int max_pos_out;

}SERVO_SPEED_LIMIT_STRUCT;


static PARA_SPEED_LOOP_READ_INFO speed_alg_info[AXIS_MAX];
static SERVO_SPEED_LIMIT_STRUCT sp_limit_info[AXIS_MAX];
static SERVO_TRAJ_PARA_UPDATE_INFO *traj_feed_info[AXIS_MAX];
/*------------------------------------------------------------------------------------
 * 函数名称:void Servo_SpLoop_Init(void)
 * 函数输入:
 * 函数输出:
 * 函数备注:初始化默认参数变量
 *---------------------------------------------------------------------------------- */
 void virtual_servo_device::Servo_SpLoop_Init(void)
{
	speed_alg_info[AXIS_0].kp_factor.para_value = 6000000;
	speed_alg_info[AXIS_0].kp_factor.default_value = 6000000;
	speed_alg_info[AXIS_0].kp_factor.value_up_limit = 16777216;
	speed_alg_info[AXIS_0].kp_factor.value_low_limit = 3000000;

	speed_alg_info[AXIS_1].kp_factor.para_value = 6000000;
	speed_alg_info[AXIS_1].kp_factor.default_value = 6000000;
	speed_alg_info[AXIS_1].kp_factor.value_up_limit = 16777216;
	speed_alg_info[AXIS_1].kp_factor.value_low_limit = 3000000;

	speed_alg_info[AXIS_2].kp_factor.para_value = 6000000;
	speed_alg_info[AXIS_2].kp_factor.default_value = 6000000;
	speed_alg_info[AXIS_2].kp_factor.value_up_limit = 16777216;
	speed_alg_info[AXIS_2].kp_factor.value_low_limit = 3000000;

	speed_alg_info[AXIS_3].kp_factor.para_value = 6000000;
	speed_alg_info[AXIS_3].kp_factor.default_value = 6000000;
	speed_alg_info[AXIS_3].kp_factor.value_up_limit = 16777216;
	speed_alg_info[AXIS_3].kp_factor.value_low_limit = 3000000;

	speed_alg_info[AXIS_4].kp_factor.para_value = 6000000;
	speed_alg_info[AXIS_4].kp_factor.default_value = 6000000;
	speed_alg_info[AXIS_4].kp_factor.value_up_limit = 16777216;
	speed_alg_info[AXIS_4].kp_factor.value_low_limit = 3000000;

	speed_alg_info[AXIS_5].kp_factor.para_value = 6000000;
	speed_alg_info[AXIS_5].kp_factor.default_value = 6000000;
	speed_alg_info[AXIS_5].kp_factor.value_up_limit = 16777216;
	speed_alg_info[AXIS_5].kp_factor.value_low_limit = 3000000;

	Servo_Traj_Get_TrajPtr(AXIS_0,&traj_feed_info[AXIS_0]);
	Servo_Traj_Get_TrajPtr(AXIS_1,&traj_feed_info[AXIS_1]);
	Servo_Traj_Get_TrajPtr(AXIS_2,&traj_feed_info[AXIS_2]);
	Servo_Traj_Get_TrajPtr(AXIS_3,&traj_feed_info[AXIS_3]);
	Servo_Traj_Get_TrajPtr(AXIS_4,&traj_feed_info[AXIS_4]);
	Servo_Traj_Get_TrajPtr(AXIS_5,&traj_feed_info[AXIS_5]);
	Servo_Traj_Get_TrajPtr(AXIS_6,&traj_feed_info[AXIS_6]);
	Servo_Traj_Get_TrajPtr(AXIS_7,&traj_feed_info[AXIS_7]);

	for(uint32_t i=0; i<AXIS_MAX; i++)
	{
		sp_limit_info[i].max_pos_out = 16777216*2;
		sp_limit_info[i].min_pos_out = -16777216*2;
	}

}
/*------------------------------------------------------------------------------------
 * 函数名称:void Servo_SpLoop_SetPara(SERVO_AXIS_ENUM axis_id,
								const PARA_SPEED_LOOP_LOCAL_INFO *speed_alg_ptr)
 * 函数输入:框架参数配置指针
 * 函数输出:none
 * 函数备注:设置速度环参数
 *---------------------------------------------------------------------------------- */
 void virtual_servo_device::Servo_SpLoop_SetPara(SERVO_AXIS_ENUM axis_id,
								const PARA_SPEED_LOOP_READ_INFO *speed_alg_ptr)
{
		speed_alg_info[axis_id].kp_factor.para_value
		= speed_alg_ptr->kp_factor.para_value;

		speed_alg_info[axis_id].time_res_factor.para_value
				= ki_cal(uint32_t,speed_alg_ptr->kp_factor.para_value,speed_alg_ptr->time_res_factor.para_value);

		speed_alg_info[axis_id].inertia_factor.para_value
					= speed_alg_ptr->inertia_factor.para_value;

		printf("kp=%d\r\n",speed_alg_info[axis_id].kp_factor.para_value);
		printf("ki=%d\r\n",speed_alg_info[axis_id].time_res_factor.para_value);
		printf("intertia=%d\r\n",speed_alg_info[axis_id].inertia_factor.para_value);
}
/*------------------------------------------------------------------------------------
 * 函数名称:int32_t Servo_SpLoop_GetPara(PARA_SPEED_LOOP_READ_INFO *speed_alg_ptr
							,SERVO_AXIS_ENUM axis_id
							,int32_t copy_num)
 * 函数输入:
 * 函数输出:
 * 函数备注:
 *---------------------------------------------------------------------------------- */
 int32_t virtual_servo_device::Servo_SpLoop_GetPara(PARA_SPEED_LOOP_READ_INFO *speed_alg_ptr
							,SERVO_AXIS_ENUM axis_id
							,int32_t copy_num)
{
	memcpy((void *)speed_alg_ptr,(void *)&speed_alg_info[axis_id],copy_num);

	return copy_num;
}
/*------------------------------------------------------------------------------------
 * 函数名称:void Servo_SpLoop_VelFeed(SERVO_AXIS_ENUM axis_id
								,int64_t *output)
 * 函数输入:
 * 函数输出:
 * 函数备注:
 *---------------------------------------------------------------------------------- */
 void virtual_servo_device::Servo_SpLoop_VelFeed(SERVO_AXIS_ENUM axis_id
								,int64_t *output)
{
	*output += traj_feed_info[axis_id]->vel_update;//需要添加保护
}
/*------------------------------------------------------------------------------------
 * 函数名称:void Servo_SpLoop_p_Cal(SERVO_AXIS_ENUM axis_id,
 * 				SERVO_AXIS_INF_STRUCT *axis_info_ptr)
 * 函数输入:轴枚举，轴信息变量指针
 * 函数输出:none
 * 函数备注:速度环计算函数
 *---------------------------------------------------------------------------------- */
 void virtual_servo_device::Servo_SpLoop_p_Cal(SERVO_AXIS_ENUM axis_id,SERVO_AXIS_INF_STRUCT *axis_info_ptr)
{
		axis_info_ptr->kp_out = ((axis_info_ptr->ctl_err
							  *speed_alg_info[axis_id].inertia_factor.para_value
							  *speed_alg_info[axis_id].kp_factor.para_value)>>24);

		axis_info_ptr->ki_out = ((axis_info_ptr->ctl_err
									*speed_alg_info[axis_id].inertia_factor.para_value
									*speed_alg_info[axis_id].time_res_factor.para_value)>>24);

		axis_info_ptr->ki_sum += axis_info_ptr->ki_out;

		if((axis_info_ptr->ki_sum) >sp_limit_info[axis_id].max_pos_out)
		{
			axis_info_ptr->ki_sum = sp_limit_info[axis_id].max_pos_out;
		}

		if((axis_info_ptr->ki_sum) < sp_limit_info[axis_id].min_pos_out)
		{
			axis_info_ptr->ki_sum = sp_limit_info[axis_id].min_pos_out;
		}

		axis_info_ptr->con_pid_out = (axis_info_ptr->kp_out + axis_info_ptr->ki_sum);

		/*速度环输出限幅*/
		if((axis_info_ptr->con_pid_out) > sp_limit_info[axis_id].max_pos_out)
		{
			axis_info_ptr->con_pid_out = sp_limit_info[axis_id].max_pos_out;
		}

		if((axis_info_ptr->con_pid_out) < sp_limit_info[axis_id].min_pos_out)
		{
			axis_info_ptr->con_pid_out = sp_limit_info[axis_id].min_pos_out;
		}
}

