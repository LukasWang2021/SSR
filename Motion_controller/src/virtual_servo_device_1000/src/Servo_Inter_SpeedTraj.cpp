/*
 * Servo_Inter_SpeedTraj.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */

#include "Servo_Inter_SpeedTraj.h"
#include "Servo_General.h"
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <math.h>
#define TIME_DISPERATE 0.0002
#define VELOCITY_FACTOR 67.108864

using namespace virtual_servo_device;

static SERVO_SPEED_MODE_GIVEN_INFO vel_info[AXIS_MAX];
static SERVO_SPEED_MODE_UPDATE_INFO vel_update[AXIS_MAX];
static SERVO_SPEED_TIME_INFO time_info[AXIS_MAX];

static int32_t vel_state[AXIS_MAX] = {VELOCITY_IDLE};
static int32_t acc_shape[AXIS_MAX] = {J_LINE};
static uint32_t time_count[AXIS_MAX] = {0};

static void Servo_SpTraj_CalTime(SERVO_AXIS_ENUM axis_id);
 /*-------------------------------------------------------------
  *函数名称 :void Servo_SpTraj_Init(void)
  *函数输入 :无
  *函数输出 :无
  *函数描述 :框架数据初始化
 --------------------------------------------------------------*/
 void virtual_servo_device::Servo_SpTraj_Init(void)
{
}
/*-------------------------------------------------------------
 *函数名称 :void Servo_SpTraj_Get_SpeedPtr(SERVO_SPEED_MODE_UPDATE_INFO **update_vel
									,SERVO_AXIS_ENUM axis_id)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_SpTraj_Get_SpeedPtr(SERVO_SPEED_MODE_UPDATE_INFO **update_vel
									,SERVO_AXIS_ENUM axis_id)
{
	*update_vel = &vel_update[axis_id];
}
#define COMPARE_LIMIT 0.000001
/*-------------------------------------------------------------
 *函数名称 :void MOGR_p_Write_FPGA(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void Servo_SpTraj_CalTime(SERVO_AXIS_ENUM axis_id)
{
	double temp_jeark_time = 0.;
	double temp_acc_time = 0.;

	double temp_vel_jeark = (double)(vel_info[axis_id].vel_jeark);
	double temp_pos_acc = (double)(vel_info[axis_id].pos_acc);
	double temp_velocity = (double)(vel_info[axis_id].target_velocity);

	double temp_data = 0.;
	double temp_vel = 0.;

	temp_data = temp_pos_acc/temp_vel_jeark;
	temp_vel = temp_vel_jeark*temp_data*temp_data;

	if((temp_velocity-temp_vel)>COMPARE_LIMIT)
	{
		temp_jeark_time = temp_data*5000;
		time_info[axis_id].jeark_time = (uint32_t)(temp_jeark_time);
		temp_acc_time = 5000*(temp_velocity-temp_vel)/temp_pos_acc;
		time_info[axis_id].acc_time = (uint32_t)(temp_acc_time);
		acc_shape[axis_id] = JA_LINE;
	}else{
		temp_vel = sqrt(temp_velocity/temp_vel_jeark);
		temp_jeark_time = temp_vel*5000;
		time_info[axis_id].jeark_time = (uint32_t)(temp_jeark_time);
		time_info[axis_id].acc_time = 0;
		acc_shape[axis_id] = J_LINE;
	}

	vel_state[axis_id] = VELOCITY_JEARK_S1;
}
/*-------------------------------------------------------------
 *函数名称 :void Servo_SpTraj_SetPara(SERVO_AXIS_ENUM axis_id,
		const SERVO_SPEED_MODE_GIVEN_INFO *vel_ptr)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_SpTraj_SetPara(SERVO_AXIS_ENUM axis_id,
		const int32_t *vel_ptr)
{
	assert(vel_ptr!=NULL);
	vel_info[axis_id].target_velocity = *(vel_ptr+1);
	vel_info[axis_id].pos_acc = *(vel_ptr+2);
	vel_info[axis_id].dec_acc = *(vel_ptr+3);
	vel_info[axis_id].vel_jeark = *(vel_ptr+4);
	vel_info[axis_id].direction = *(vel_ptr+5);

	if(vel_info[axis_id].vel_jeark<500)
	{
		vel_info[axis_id].vel_jeark = 500;//1/0.0002,避免加速度过小，规划不出轨迹
	}

	Servo_SpTraj_CalTime(axis_id);
	vel_update[axis_id].update_flag = 1;
}
/*-------------------------------------------------------------
 *函数名称 :void Servo_SpTraj_Update(SERVO_AXIS_ENUM axis_id)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_SpTraj_p_Update(SERVO_AXIS_ENUM axis_id)
{
    static int32_t temp_vel_update[AXIS_MAX] = {0};
    static int32_t temp_acc_update[AXIS_MAX] = {0};
	double temp_value = 0.;
    double temp_acc = 0.;
    double temp_acc_jerk2 = 0.;
	if(vel_update[axis_id].update_flag)
	{
		switch(vel_state[axis_id])
		{
			case(VELOCITY_IDLE):
			{
				break;
			}
			case(VELOCITY_JEARK_S1):
			{
				time_count[axis_id]++;
				if(acc_shape[axis_id]==J_LINE)
				{
					if(time_count[axis_id]<=time_info[axis_id].jeark_time)
					{
                        temp_acc = time_count[axis_id]*vel_info[axis_id].vel_jeark*TIME_DISPERATE;

                        temp_acc = (vel_info[axis_id].direction==1)?(temp_acc):(0-temp_acc);

						temp_value = (0.5*temp_acc
								*time_count[axis_id]*VELOCITY_FACTOR);

						vel_update[axis_id].velocity_update = (int64_t)temp_value;

					}else{
							time_count[axis_id] = 0;
                            temp_vel_update[axis_id] = vel_update[axis_id].velocity_update;
                            temp_acc_update[axis_id] = (vel_info[axis_id].direction==1)?(vel_info[axis_id].pos_acc):(0-vel_info[axis_id].pos_acc);
                            printf("acc_data_J:%d\r\n",temp_acc_update[axis_id]);
                            printf("vel_update_J:%d\r\n",temp_vel_update[axis_id]);
							vel_state[axis_id] = VELOCITY_JEARK_S2;
					}
			}else if(acc_shape[axis_id]==JA_LINE){

				if(time_count[axis_id]<=time_info[axis_id].jeark_time)
				{
                    temp_acc = time_count[axis_id]*vel_info[axis_id].vel_jeark*TIME_DISPERATE;

                    temp_acc = (vel_info[axis_id].direction==1)?(temp_acc):(0-temp_acc);

					temp_value = (0.5*temp_acc*time_count[axis_id]*VELOCITY_FACTOR);

					vel_update[axis_id].velocity_update = (int64_t)temp_value;
				}else{
					time_count[axis_id] = 0;
                    temp_vel_update[axis_id] = vel_update[axis_id].velocity_update;
                    temp_acc_update[axis_id] = (vel_info[axis_id].direction==1)?(vel_info[axis_id].pos_acc):(0-vel_info[axis_id].pos_acc);
                    printf("acc_data_JA:%d\r\n",temp_acc_update[axis_id]);
                    printf("vel_update_J:%d\r\n",temp_vel_update[axis_id]);
					vel_state[axis_id] = VELOCITY_ACC;
				}

			}else{

			}
			break;
		}
		case(VELOCITY_ACC):
		{
			time_count[axis_id]++;
			if(time_count[axis_id]<=time_info[axis_id].acc_time)
			{
				temp_value = (time_count[axis_id]*temp_acc_update[axis_id]*VELOCITY_FACTOR)+temp_vel_update[axis_id];

				vel_update[axis_id].velocity_update = (int32_t)(temp_value);
			}else{
				time_count[axis_id] = 0;
                temp_vel_update[axis_id] = vel_update[axis_id].velocity_update;
                printf("vel_update:%d\r\n",temp_vel_update[axis_id]);
				vel_state[axis_id] = VELOCITY_JEARK_S2;
			}

			break;
		}
        case(VELOCITY_JEARK_S2):
        {
            time_count[axis_id]++;

            if(time_count[axis_id]<=time_info[axis_id].jeark_time)
            {
            temp_acc_jerk2 = 0.5*vel_info[axis_id].vel_jeark*time_count[axis_id]*TIME_DISPERATE;

            temp_acc_jerk2 = (vel_info[axis_id].direction==1)?(0-temp_acc_jerk2):(temp_acc_jerk2);

            temp_acc = temp_acc_update[axis_id] + temp_acc_jerk2;

			temp_value = (temp_acc*time_count[axis_id]*VELOCITY_FACTOR)+temp_vel_update[axis_id];

			vel_update[axis_id].velocity_update = (int64_t)temp_value;

            }else{
            time_count[axis_id] = 0;
            printf("JERK_S2:%d\r\n",temp_vel_update[axis_id]);
            vel_state[axis_id] = VELOCITY_CONSTANT;
            }
            break;
        }
		case(VELOCITY_CONSTANT):
		{
			time_count[axis_id] = 0;
            temp_vel_update[axis_id] = 0;
			vel_update[axis_id].update_flag = 0;
			break;
		}
        default:break;
	}
	printf("vel_update:%d\r\n",vel_update[axis_id].velocity_update);
   }
}
/*-------------------------------------------------------------
 *函数名称 :void Servo_SpTraj_Update(SERVO_AXIS_ENUM axis_id):
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_SpTraj_RestVar(SERVO_AXIS_ENUM axis_id)
{
	time_info[axis_id].acc_time = 0;
	time_info[axis_id].jeark_time = 0;

	vel_update[axis_id].update_flag = 0;
	vel_update[axis_id].velocity_update = 0;

}

