/*
 * Servo_Speed.h
 *
 *  Created on: 2019年7月25日
 *      Author: qichao.wang
 */

#ifndef SERVO_SPEED_H_
#define SERVO_SPEED_H_

#include "Servo_Fpga.h"
namespace virtual_servo_device{
typedef struct
{
	int32_t speed_back;

}SERVO_SPEED_OUTPUT_STRUCT;

void Servo_Speed_Init(void);

void Servo_Speed_Get_SpeedPtr(SERVO_SPEED_OUTPUT_STRUCT **get_ptr,
									SERVO_AXIS_ENUM axis_id);

void Servo_Speed_p_CalSpeed(SERVO_AXIS_ENUM axis_id);
}
#endif /* SERVO_SPEED_H_ */

