/*
 * Servo_Speed.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */
#include "Servo_Speed.h"


using namespace virtual_servo_device;

static SERVO_SPEED_OUTPUT_STRUCT speed_output[AXIS_MAX];

static SERVO_FPGA_VEL_BACK_STRUCT *source_speed[AXIS_MAX];
/*-----------------------------------------------------------------------
 * 函数名称: void Servo_Speed_Init(void)
 * 函数输出: none
 * 函数输入: none
 * 函数备注: 速度框架初始化，获取
 * --------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Speed_Init(void)
{
	Servo_Fpga_Get_VelPtr(&source_speed[AXIS_0],AXIS_0);
	Servo_Fpga_Get_VelPtr(&source_speed[AXIS_1],AXIS_1);
	Servo_Fpga_Get_VelPtr(&source_speed[AXIS_2],AXIS_2);
	Servo_Fpga_Get_VelPtr(&source_speed[AXIS_3],AXIS_3);
	Servo_Fpga_Get_VelPtr(&source_speed[AXIS_4],AXIS_4);
	Servo_Fpga_Get_VelPtr(&source_speed[AXIS_5],AXIS_5);
	Servo_Fpga_Get_VelPtr(&source_speed[AXIS_6],AXIS_6);
	Servo_Fpga_Get_VelPtr(&source_speed[AXIS_7],AXIS_7);
}
/*--------------------------------------------------------------------------
 * 函数名称: void Servo_Speed_Get_SpeedPtr(SERVO_SPEED_OUTPUT_STRUCT **get_ptr,
									SERVO_AXIS_ENUM axis_id)
 * 函数输出: none
 * 函数输入: none
 * 函数备注: 速
 * ------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Speed_Get_SpeedPtr(SERVO_SPEED_OUTPUT_STRUCT **get_ptr,
									SERVO_AXIS_ENUM axis_id)
{
	*get_ptr = &speed_output[axis_id];
}
/*--------------------------------------------------------------------------
 * 函数名称: void Servo_Speed_p_CalSpeed(SERVO_AXIS_ENUM axis_id)
 * 函数输出: none
 * 函数输入: none
 * 函数备注: 从FPGA拿过来的数据进行滤波处理
 * ------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Speed_p_CalSpeed(SERVO_AXIS_ENUM axis_id)
{
	speed_output[axis_id].speed_back=
							source_speed[axis_id]->vel_act;//需要添加低通滤波
}

