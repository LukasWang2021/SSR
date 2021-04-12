/*
 * Servo_Traj.h
 *
 *  Created on: 2019年7月15日
 *      Author: qichao.wang
 */

#ifndef SERVO_TRAJ_H_
#define SERVO_TRAJ_H_

#include "Servo_General.h"

namespace virtual_servo_device{


typedef struct
{
	int64_t set_position;
	int64_t vel_limit;
	int64_t acc_limit;
	int64_t dec_limit;
	int64_t jerk;

}SERVO_TRAJ_PARA_INFO;

typedef struct
{
	double set_position;
	double vel_max;
	double acc_max;
	double dec_max;
	double jerk_max;

}SERVO_TRAJ_PARA_CONV_FINFO;

typedef struct
{
	int64_t pos_update;
	int64_t pos_offset;
	int64_t vel_update;
	int64_t acc_update;
	int32_t motion_flag;

}SERVO_TRAJ_PARA_UPDATE_INFO;

typedef struct
{
	int64_t internal_step;
	int64_t step_count;
	int64_t update_flag;

}SERVO_TRAJ_TIME_COUNT;

typedef struct
{
	double va;
	double sa;
	double sv;
	int32_t p_factor;
	int32_t q_factor;

}SERVO_ANA_ACC_LINE_STYLE_STRUCT;

typedef struct
{
	double ta_time;
	double tj_time;
	double tv_time;

}SERVO_LINE_TIME_STRUCT;

void Servo_Traj_Init(void);

void Servo_Traj_Get_TrajPtr(SERVO_AXIS_ENUM axis_id,
		SERVO_TRAJ_PARA_UPDATE_INFO** get_traj_ptr);

void Servo_Traj_p_UpdatePoint(SERVO_AXIS_ENUM axis_id);

void Servo_Traj_Set_ContinueMotion(SERVO_AXIS_ENUM axis_id,
									int32_t set_lsb,
									int32_t set_msb,
									int32_t vel_ff,
									int64_t current_pos);
}

#endif /* SERVO_TRAJ_H_ */

