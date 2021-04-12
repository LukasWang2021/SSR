/*
 * Servo_Position_Loop.h
 *
 *  Created on: 2019年7月15日
 *      Author: qichao.wang
 */

#ifndef SERVO_POSITION_LOOP_H_
#define SERVO_POSITION_LOOP_H_

#include "Servo_Para.h"

namespace virtual_servo_device{

typedef struct
{
	int64_t filter_out;
	int64_t filter_var;

}SERVO_POSITION_FILTER_ALG;

void Servo_PosLoop_Init(void);

void Servo_PosLoop_SetPara(SERVO_AXIS_ENUM axis_id,const PARA_POSITION_LOOP_READ_INFO *para_cfg_ptr);

int32_t Servo_PosLoop_GetPara(PARA_POSITION_LOOP_READ_INFO *para_cfg_ptr,
							   SERVO_AXIS_ENUM axis_id,
							   int32_t copy_num);

void Servo_PosLoop_p_Cal(SERVO_AXIS_ENUM axis_id,SERVO_AXIS_INF_STRUCT *axis_var_info);

}

#endif /* SERVO_POSITION_LOOP_H_ */

