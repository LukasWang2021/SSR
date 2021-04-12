/*
 * Servo_QuichStop.h
 *
 *  Created on: 2019年7月29日
 *      Author: qichao.wang
 */

#ifndef SERVO_QUICHSTOP_H_
#define SERVO_QUICHSTOP_H_

#include "Servo_General.h"
#include <stdint.h>

/*定标完成*/
#define DEC_TIME 0.5
#define DEC_COUNT 1000

namespace virtual_servo_device{

typedef enum
{
	SG_IDLE,
	SG_DEC

}SERVO_QUICK_STOP_STATE_ENUM;

typedef enum
{
	STOP_QS_STATE,
	POS_QS_STATE,
	NEG_QS_STATE

}SERVO_QUICK_STOP_DIRECTION_ENUM;

typedef struct
{
	int32_t gs_state;
	int32_t update_vel;
	int32_t init_vel;
	int32_t comepe_flag;
	int32_t dec_count;
	int32_t dec_acc_step;
	int64_t update_pos;

}SERVO_QUICK_STOP_UPDATE_STRUCT;

typedef struct
{
	double set_curr_vel;
	double set_dec_acc;

}SERVO_QUICK_STOP_VAR_STRUCT;

void Servo_QuickStop_Init(void);

void Servo_QuickStop_Get_QsPtr(SERVO_QUICK_STOP_UPDATE_STRUCT **speed_data
									,SERVO_AXIS_ENUM axis_id);

void Servo_QuickStop_Update(SERVO_AXIS_ENUM axis_id);

void Servo_QuickStop_Cal_DecTime(SERVO_AXIS_ENUM axis_id,int curr_speed,int64_t position);
}

#endif /* SERVO_QUICHSTOP_H_ */

