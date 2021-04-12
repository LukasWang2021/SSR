/*
 * Servo_Speed_Loop.h
 *
 *  Created on: 2019年7月15日
 *      Author: qichao.wang
 */

#ifndef SERVO_SPEED_LOOP_H_
#define SERVO_SPEED_LOOP_H_

#include "Servo_General.h"
#include "Servo_Para.h"

#define SERVO_SPEED_LOOP_BASE 0x60

#define KI_CAL_FACTOR 0.0002

#define ki_cal(type,x,y) ({type __x=(x); type __y=(y);(type)((__x*__y*KI_CAL_FACTOR));})

namespace virtual_servo_device{

typedef struct
{
	int64_t filter_out;
	int64_t filter_var;

}SERVO_SPEED_FILTER_ALG;

void Servo_SpLoop_Init(void);

void Servo_SpLoop_SetPara(SERVO_AXIS_ENUM axis_id,const PARA_SPEED_LOOP_READ_INFO *speed_alg_ptr);

int32_t Servo_SpLoop_GetPara(PARA_SPEED_LOOP_READ_INFO *speed_alg_ptr,SERVO_AXIS_ENUM axis_id
									,int32_t copy_num);

void Servo_SpLoop_VelFeed(SERVO_AXIS_ENUM axis_id
								,int64_t *output);

void Servo_SpLoop_p_Cal(SERVO_AXIS_ENUM axis_id,SERVO_AXIS_INF_STRUCT *axis_info_ptr);

}

#endif /* SERVO_SPEED_LOOP_H_ */

