/*
 * Servo_Current_Loop.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */

#include "Servo_Current_Loop.h"
#include "Servo_Para.h"
#include "Servo_General.h"
#include "Servo_Fpga.h"

using namespace virtual_servo_device;

static PARA_CURRENT_LOOP_READ_INFO current_alg_info[AXIS_MAX];

void virtual_servo_device::Servo_CurLoop_SetPara(SERVO_AXIS_ENUM axis_id,
								  const PARA_CURRENT_LOOP_READ_INFO *para_ptr)
{
	current_alg_info[axis_id].kp_factor.para_value = 6000000;//para_ptr->kp_factor.para_value;
	current_alg_info[axis_id].ki_factor.para_value = 1222290;//para_ptr->ki_factor.para_value;

	Servo_Fpga_Write(axis_id,CUR_KP_RAM,current_alg_info[axis_id].kp_factor.para_value);
	Servo_Fpga_Write(axis_id,CUR_KI_RAM,current_alg_info[axis_id].ki_factor.para_value);
}

