/*
 * Servo_Axis.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */

#include "Servo_Axis.h"
#include "Servo_Motor.h"
#include "Servo_Current_Loop.h"
#include "Servo_Driver.h"
#include "Servo_CheckErr.h"
#include "Servo_Dignose.h"
#include "common/comm_reg_0.h"
#include "common/core_comm_servo_datatype.h"
#include "log_manager_producer_bare.h"
#include "Servo_Sm.h"
#include <assert.h>
#include <string.h>
#include <iostream>

using namespace std;
using namespace virtual_servo_device;

static uint32_t axis_state[AXIS_MAX];
static uint32_t mode_state[AXIS_MAX];

static SERVO_AXIS_INF_STRUCT pos_loop_info[AXIS_MAX];
static SERVO_AXIS_INF_STRUCT sp_loop_info[AXIS_MAX];

static SERVO_ENCODE_OUTPUT_STRUCT *position_back_info[AXIS_MAX];
static SERVO_SPEED_OUTPUT_STRUCT *speed_back_info[AXIS_MAX];

static SERVO_TRAJ_PARA_UPDATE_INFO *motion_traj[AXIS_MAX];

static SERVO_SPEED_MODE_UPDATE_INFO *speed_inter_traj[AXIS_MAX];
static SERVO_POS_TRAJ_CONV_UPDATE_INFO *pos_inter_traj[AXIS_MAX];
static SERVO_TORQUE_TRAJ_UPDATE_STRUCT *torque_inter_traj[AXIS_MAX];

static PARA_AXIS_READ_INFO *axis_para_inter_info[AXIS_MAX];
static PARA_AXIS_LOCAL_INFO *axis_para_local_info[AXIS_MAX];
static SERVO_FPGA_IQID_STRUCT *axis_current_dq[AXIS_MAX];

static VirtualServo1000_t* g_local_servo1000;

/*----------------------------框架静态函数---------------------------*/
static void Servo_Axis_SetPara(SERVO_AXIS_ENUM axis_id);
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_Init(void)
 * 函数输入:none
 * 函数输出:none
 * 函数描述:初始化框架静态数据，获取其他框架指针数据
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_Init(void)
{
	Servo_Para_Get_ReadPtr(AXIS_0,&axis_para_inter_info[AXIS_0]);
	Servo_Para_Get_ReadPtr(AXIS_1,&axis_para_inter_info[AXIS_1]);
	Servo_Para_Get_ReadPtr(AXIS_2,&axis_para_inter_info[AXIS_2]);
	Servo_Para_Get_ReadPtr(AXIS_3,&axis_para_inter_info[AXIS_3]);
	Servo_Para_Get_ReadPtr(AXIS_4,&axis_para_inter_info[AXIS_4]);
	Servo_Para_Get_ReadPtr(AXIS_5,&axis_para_inter_info[AXIS_5]);
	Servo_Para_Get_ReadPtr(AXIS_6,&axis_para_inter_info[AXIS_6]);
	Servo_Para_Get_ReadPtr(AXIS_7,&axis_para_inter_info[AXIS_7]);

	Servo_Para_Get_LocalPtr(AXIS_0,&axis_para_local_info[AXIS_0]);
	Servo_Para_Get_LocalPtr(AXIS_1,&axis_para_local_info[AXIS_1]);
	Servo_Para_Get_LocalPtr(AXIS_2,&axis_para_local_info[AXIS_2]);
	Servo_Para_Get_LocalPtr(AXIS_3,&axis_para_local_info[AXIS_3]);
	Servo_Para_Get_LocalPtr(AXIS_4,&axis_para_local_info[AXIS_4]);
	Servo_Para_Get_LocalPtr(AXIS_5,&axis_para_local_info[AXIS_5]);
	Servo_Para_Get_LocalPtr(AXIS_6,&axis_para_local_info[AXIS_6]);
	Servo_Para_Get_LocalPtr(AXIS_7,&axis_para_local_info[AXIS_7]);

	Servo_Encode_Get_EncodePtr(&position_back_info[AXIS_0],AXIS_0);
	Servo_Encode_Get_EncodePtr(&position_back_info[AXIS_1],AXIS_1);
	Servo_Encode_Get_EncodePtr(&position_back_info[AXIS_2],AXIS_2);
	Servo_Encode_Get_EncodePtr(&position_back_info[AXIS_3],AXIS_3);
	Servo_Encode_Get_EncodePtr(&position_back_info[AXIS_4],AXIS_4);
	Servo_Encode_Get_EncodePtr(&position_back_info[AXIS_5],AXIS_5);
	Servo_Encode_Get_EncodePtr(&position_back_info[AXIS_6],AXIS_6);
	Servo_Encode_Get_EncodePtr(&position_back_info[AXIS_7],AXIS_7);

	Servo_Speed_Get_SpeedPtr(&speed_back_info[AXIS_0],AXIS_0);
	Servo_Speed_Get_SpeedPtr(&speed_back_info[AXIS_1],AXIS_1);
	Servo_Speed_Get_SpeedPtr(&speed_back_info[AXIS_2],AXIS_2);
	Servo_Speed_Get_SpeedPtr(&speed_back_info[AXIS_3],AXIS_3);
	Servo_Speed_Get_SpeedPtr(&speed_back_info[AXIS_4],AXIS_4);
	Servo_Speed_Get_SpeedPtr(&speed_back_info[AXIS_5],AXIS_5);
	Servo_Speed_Get_SpeedPtr(&speed_back_info[AXIS_6],AXIS_6);
	Servo_Speed_Get_SpeedPtr(&speed_back_info[AXIS_7],AXIS_7);

	Servo_TorTrj_Get_TrjPtr(&torque_inter_traj[AXIS_0],AXIS_0);
	Servo_TorTrj_Get_TrjPtr(&torque_inter_traj[AXIS_1],AXIS_1);
	Servo_TorTrj_Get_TrjPtr(&torque_inter_traj[AXIS_2],AXIS_2);
	Servo_TorTrj_Get_TrjPtr(&torque_inter_traj[AXIS_3],AXIS_3);
	Servo_TorTrj_Get_TrjPtr(&torque_inter_traj[AXIS_4],AXIS_4);
	Servo_TorTrj_Get_TrjPtr(&torque_inter_traj[AXIS_5],AXIS_5);
	Servo_TorTrj_Get_TrjPtr(&torque_inter_traj[AXIS_6],AXIS_6);
	Servo_TorTrj_Get_TrjPtr(&torque_inter_traj[AXIS_7],AXIS_7);

	Servo_Traj_Get_TrajPtr(AXIS_0,&motion_traj[AXIS_0]);
	Servo_Traj_Get_TrajPtr(AXIS_1,&motion_traj[AXIS_1]);
	Servo_Traj_Get_TrajPtr(AXIS_2,&motion_traj[AXIS_2]);
	Servo_Traj_Get_TrajPtr(AXIS_3,&motion_traj[AXIS_3]);
	Servo_Traj_Get_TrajPtr(AXIS_4,&motion_traj[AXIS_4]);
	Servo_Traj_Get_TrajPtr(AXIS_5,&motion_traj[AXIS_5]);
	Servo_Traj_Get_TrajPtr(AXIS_6,&motion_traj[AXIS_6]);
	Servo_Traj_Get_TrajPtr(AXIS_7,&motion_traj[AXIS_7]);

	Servo_Fpga_Get_IqdPtr(&axis_current_dq[AXIS_0],AXIS_0);
	Servo_Fpga_Get_IqdPtr(&axis_current_dq[AXIS_1],AXIS_1);
	Servo_Fpga_Get_IqdPtr(&axis_current_dq[AXIS_2],AXIS_2);
	Servo_Fpga_Get_IqdPtr(&axis_current_dq[AXIS_3],AXIS_3);
	Servo_Fpga_Get_IqdPtr(&axis_current_dq[AXIS_4],AXIS_4);
	Servo_Fpga_Get_IqdPtr(&axis_current_dq[AXIS_5],AXIS_5);
	Servo_Fpga_Get_IqdPtr(&axis_current_dq[AXIS_6],AXIS_6);
	Servo_Fpga_Get_IqdPtr(&axis_current_dq[AXIS_7],AXIS_7);

	Servo_SpTraj_Get_SpeedPtr(&speed_inter_traj[AXIS_0],AXIS_0);
	Servo_SpTraj_Get_SpeedPtr(&speed_inter_traj[AXIS_1],AXIS_1);
	Servo_SpTraj_Get_SpeedPtr(&speed_inter_traj[AXIS_2],AXIS_2);
	Servo_SpTraj_Get_SpeedPtr(&speed_inter_traj[AXIS_3],AXIS_3);
	Servo_SpTraj_Get_SpeedPtr(&speed_inter_traj[AXIS_4],AXIS_4);
	Servo_SpTraj_Get_SpeedPtr(&speed_inter_traj[AXIS_5],AXIS_5);
	Servo_SpTraj_Get_SpeedPtr(&speed_inter_traj[AXIS_6],AXIS_6);
	Servo_SpTraj_Get_SpeedPtr(&speed_inter_traj[AXIS_7],AXIS_7);

	Servo_PosTraj_Get_TrajUpdate_Ptr(AXIS_0,&pos_inter_traj[AXIS_0]);
	Servo_PosTraj_Get_TrajUpdate_Ptr(AXIS_1,&pos_inter_traj[AXIS_1]);
	Servo_PosTraj_Get_TrajUpdate_Ptr(AXIS_2,&pos_inter_traj[AXIS_2]);
	Servo_PosTraj_Get_TrajUpdate_Ptr(AXIS_3,&pos_inter_traj[AXIS_3]);
	Servo_PosTraj_Get_TrajUpdate_Ptr(AXIS_4,&pos_inter_traj[AXIS_4]);
	Servo_PosTraj_Get_TrajUpdate_Ptr(AXIS_5,&pos_inter_traj[AXIS_5]);
	Servo_PosTraj_Get_TrajUpdate_Ptr(AXIS_6,&pos_inter_traj[AXIS_6]);
	Servo_PosTraj_Get_TrajUpdate_Ptr(AXIS_7,&pos_inter_traj[AXIS_7]);

	Servo_Axis_SetPara(AXIS_0);
	Servo_Axis_SetPara(AXIS_1);
	Servo_Axis_SetPara(AXIS_2);
	Servo_Axis_SetPara(AXIS_3);
	Servo_Axis_SetPara(AXIS_4);
	Servo_Axis_SetPara(AXIS_5);
	Servo_Axis_SetPara(AXIS_6);
	Servo_Axis_SetPara(AXIS_7);

	servo_cmd_get_gservo_ptr(&g_local_servo1000);

	for(int i=0; i<AXIS_MAX; i++)
	{
			/*位置环信息表初始化*/
			pos_loop_info[i].con_pid_out = 0;
			pos_loop_info[i].ctl_err = 0;
			pos_loop_info[i].kd_out = 0;
			pos_loop_info[i].ki_out = 0;
			pos_loop_info[i].ki_sum = 0;
			pos_loop_info[i].kp_out = 0;
			pos_loop_info[i].pre_dout = 0;
			/*速度环信息表初始化*/
			sp_loop_info[i].con_pid_out = 0;
			sp_loop_info[i].ctl_err = 0;
			sp_loop_info[i].kd_out = 0;
			sp_loop_info[i].ki_out = 0;
			sp_loop_info[i].ki_sum = 0;
			sp_loop_info[i].kp_out = 0;
			sp_loop_info[i].pre_dout = 0;

			mode_state[i] = NORMAL_MODE;
			axis_state[i] = SERVO_SM_START;
			/*注册变量*/
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&motion_traj[static_cast<SERVO_AXIS_ENUM>(i)]->pos_update),POS_DEM_VAL);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&position_back_info[static_cast<SERVO_AXIS_ENUM>(i)]->position_back),POS_ACT_VAL);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&pos_loop_info[static_cast<SERVO_AXIS_ENUM>(i)].ctl_err),POS_FOLL_ERR);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&pos_loop_info[static_cast<SERVO_AXIS_ENUM>(i)].kp_out),POS_KP_OUT);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&motion_traj[static_cast<SERVO_AXIS_ENUM>(i)]->vel_update),VEL_FEED_VAL);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&speed_inter_traj[static_cast<SERVO_AXIS_ENUM>(i)]->velocity_update),VEL_DEM_VAL);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&speed_back_info[static_cast<SERVO_AXIS_ENUM>(i)]->speed_back),VEL_ACT_VAL);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&sp_loop_info[static_cast<SERVO_AXIS_ENUM>(i)].ctl_err),VEL_FOLL_ERR);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&sp_loop_info[static_cast<SERVO_AXIS_ENUM>(i)].kp_out),VEL_KP_OUT);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&sp_loop_info[static_cast<SERVO_AXIS_ENUM>(i)].ki_sum),VEL_KI_OUT);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&motion_traj[static_cast<SERVO_AXIS_ENUM>(i)]->acc_update),TOR_FEED_VAL);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&sp_loop_info[static_cast<SERVO_AXIS_ENUM>(i)].con_pid_out),IQ_GIVEN_VAL);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&pos_inter_traj[static_cast<SERVO_AXIS_ENUM>(i)]->pos_update),IQ_BACK_VAL);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&speed_inter_traj[static_cast<SERVO_AXIS_ENUM>(i)]->velocity_update),PHA_CURRENT_U);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(int32_t*)(&pos_inter_traj[static_cast<SERVO_AXIS_ENUM>(i)]->vel_update),PHA_CURRENT_V);
			Servo_Dignose_Register_Var(static_cast<SERVO_AXIS_ENUM>(i),(&position_back_info[static_cast<SERVO_AXIS_ENUM>(i)]->encode_source),ENCODE_SOURCE_VAL);
	}

}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_Set_TargetPoint(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:设定轨迹目标点
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_Set_TargetPoint(SERVO_AXIS_ENUM axis_id)
{
	motion_traj[axis_id]->pos_update =
			position_back_info[axis_id]->position_back;
	pos_inter_traj[axis_id]->pos_offset =
			position_back_info[axis_id]->position_back;
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_p_Cal_PosErr(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:初始化框架静态数据，获取其他框架指针数据
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_p_Cal_PosErr(SERVO_AXIS_ENUM axis_id)
{
	if(axis_state[axis_id] == SERVO_SM_OPERATION_ENABLED)
		{
			if(*g_local_servo1000->servo[axis_id].servo_mode == SERVO_OP_MODE_INTERPOLATED_POSITION_MODE)
			{
				pos_loop_info[axis_id].ctl_err =  (int64_t)(motion_traj[axis_id]->pos_update
														- position_back_info[axis_id]->position_back);

			}
			/*---------点动模式-----------------*/
			if(*g_local_servo1000->servo[axis_id].servo_mode == SERVO_OP_MODE_PROFILE_POSITION_MODE)
			{
				pos_loop_info[axis_id].ctl_err = (int64_t)(pos_inter_traj[axis_id]->pos_update
													+ pos_inter_traj[axis_id]->pos_offset
													- position_back_info[axis_id]->position_back);
			}
		}
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_p_Cal_PosErr(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:初始化框架静态数据，获取其他框架指针数据
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_p_Cal_PosLoop(SERVO_AXIS_ENUM axis_id)
{
	switch(axis_state[axis_id])
		{
			case(SERVO_SM_START):
			case(SERVO_SM_NOT_READY_TO_SWITCH_ON):
			case(SERVO_SM_SWITCH_ON_DISABLED):
			case(SERVO_SM_READY_TO_SWITCH_ON):
			case(SERVO_SM_SWITCHED_ON):
			case(SERVO_SM_QUICK_STOP_ACTIVE):
			case(SERVO_SM_FAULT_REACTION_ACTIVE):
			case(SERVO_SM_FAULT):
			{
				pos_loop_info[axis_id].ctl_err = 0;
				pos_loop_info[axis_id].con_pid_out = 0;
				pos_loop_info[axis_id].kp_out = 0;
				pos_loop_info[axis_id].ki_sum = 0;
				pos_loop_info[axis_id].ki_out = 0;
				pos_loop_info[axis_id].kd_out = 0;
				pos_loop_info[axis_id].pre_dout = 0;

				break;
			}
			case(SERVO_SM_OPERATION_ENABLED):
			{
				/*位置环计算*/
				if((*g_local_servo1000->servo[axis_id].servo_mode == SERVO_OP_MODE_INTERPOLATED_POSITION_MODE)
						||(*g_local_servo1000->servo[axis_id].servo_mode == SERVO_OP_MODE_PROFILE_POSITION_MODE))
				{
					Servo_PosLoop_p_Cal(axis_id
										,&pos_loop_info[axis_id]);
				}
				else
				{
					pos_loop_info[axis_id].ctl_err = 0;
					pos_loop_info[axis_id].con_pid_out = 0;
					pos_loop_info[axis_id].kp_out = 0;
					pos_loop_info[axis_id].ki_sum = 0;
					pos_loop_info[axis_id].ki_out = 0;
					pos_loop_info[axis_id].kd_out = 0;
					pos_loop_info[axis_id].pre_dout = 0;
				}
				break;
			}
			default:break;
		}
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_Cal_SpErr(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:计算速度速度给定和速度反馈的差值
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_p_Cal_SpErr(SERVO_AXIS_ENUM axis_id)
{
	/*位置模式，速度环计算*/
	if((*g_local_servo1000->servo[axis_id].servo_mode == SERVO_OP_MODE_INTERPOLATED_POSITION_MODE)
			||(*g_local_servo1000->servo[axis_id].servo_mode == SERVO_OP_MODE_PROFILE_POSITION_MODE))
	{
		if(axis_state[axis_id] == SERVO_SM_OPERATION_ENABLED)
		{
			sp_loop_info[axis_id].ctl_err
						= (int64_t)(pos_loop_info[axis_id].con_pid_out - speed_back_info[axis_id]->speed_back);
		}
	}
	/*如果选用速度模式，则覆盖位置环输出参考*/
	if(*g_local_servo1000->servo[axis_id].servo_mode == SERVO_OP_MODE_PROFILE_VELOCITY_MODE)
	{
		/*速度模式*/
		if(axis_state[axis_id] == SERVO_SM_OPERATION_ENABLED)
		{
			sp_loop_info[axis_id].ctl_err = (int64_t)(speed_inter_traj[axis_id]->velocity_update
									- speed_back_info[axis_id]->speed_back);
		}
	}

}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_s_SetCurrent(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:速度环路计算，最终输出iq电流值
 * ----------------------------------------------------------------*/
void Servo_Axis_s_SetCurrent(SERVO_AXIS_ENUM axis_id)
{
	axis_current_dq[axis_id]->iq_out
					= (int32_t)sp_loop_info[axis_id].con_pid_out;
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_p_Cal_SpLoop(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:速度环路计算，最终输出iq电流值
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_p_Cal_SpeedLoop(SERVO_AXIS_ENUM axis_id)
{
	switch(axis_state[axis_id])
	{
			case(SERVO_SM_START):
			case(SERVO_SM_NOT_READY_TO_SWITCH_ON):
			case(SERVO_SM_SWITCH_ON_DISABLED):
			case(SERVO_SM_READY_TO_SWITCH_ON):
			case(SERVO_SM_SWITCHED_ON):
			case(SERVO_SM_QUICK_STOP_ACTIVE):
			case(SERVO_SM_FAULT_REACTION_ACTIVE):
			case(SERVO_SM_FAULT):
			{
				sp_loop_info[axis_id].con_pid_out = 0;
				sp_loop_info[axis_id].ctl_err = 0;
				sp_loop_info[axis_id].ki_out = 0;
				sp_loop_info[axis_id].ki_sum = 0;
				sp_loop_info[axis_id].kp_out = 0;
				sp_loop_info[axis_id].kd_out = 0;
				sp_loop_info[axis_id].pre_dout = 0;

				break;
			}
			case(SERVO_SM_OPERATION_ENABLED):
			{
				Servo_SpTraj_p_Update(axis_id);

				Servo_PosTraj_UpateTraj(axis_id);

				Servo_SpLoop_p_Cal(axis_id,&sp_loop_info[axis_id]);

				break;
			}
			default:break;
	}

	Servo_Axis_s_SetCurrent(axis_id);
	Servo_Write_p_FPGA(axis_id);

}
/*-----------------------------------------------------------------
 * 函数名称: void Servo_Axis_SetPara(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:分发设置各个轴的框架数据
 * ----------------------------------------------------------------*/
 void Servo_Axis_SetPara(SERVO_AXIS_ENUM axis_id)
{
	Servo_SpLoop_SetPara(axis_id,&axis_para_inter_info[axis_id]->axis_alg_info.sp_alg_loop);
	Servo_PosLoop_SetPara(axis_id,&axis_para_inter_info[axis_id]->axis_alg_info.pos_alg_loop);
	Servo_Motor_SetPara(axis_id,&axis_para_inter_info[axis_id]->motor_info);
	Servo_TorTrj_SetPara(axis_id,&axis_para_inter_info[axis_id]->torque_traj_info);
	Servo_Encode_SetPara(axis_id,&axis_para_inter_info[axis_id]->encode_info);
	Servo_Driver_SetPara(axis_id,&axis_para_inter_info[axis_id]->driver_info);
	Servo_CurLoop_SetPara(axis_id,&axis_para_inter_info[axis_id]->axis_alg_info.current_alg_loop);
//	Servo_CheckErr_SetPara(axis_id,&axis_para_inter_info[axis_id]->heck_err_info);
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_p_Cal_SpLoop(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:速度环路计算，最终输出iq电流值
 * ----------------------------------------------------------------*/
 void virtual_servo_device::servo_config_CoreComm(void)
{
    do
    {
        if(!g_local_servo1000->config.core_comm_config_ready)
        {
        if(g_local_servo1000->config.core_comm.isMasterBooted())
        {
            std::cout<<"master is booted"<<std::endl;
            if(g_local_servo1000->config.core_comm.bootAsSlave() == SUCCESS)
            {
                g_local_servo1000->config.from_block_ptr = g_local_servo1000->config.core_comm.getFromCommBlockDataPtrList(g_local_servo1000->config.from_block_number);
                g_local_servo1000->config.to_block_ptr = g_local_servo1000->config.core_comm.getToCommBlockDataPtrList(g_local_servo1000->config.to_block_number);
                std::cout<<"slave boot success"<<std::endl;
            }
            else
            {
                std::cout<<"slave boot failed"<<std::endl;
                goto CORE_COMM_CONFIG_FAILED;
            }

            // create servo_cpu_comm channel by configuration
            g_local_servo1000->cpu.servo_cpu_comm_ptr = createServoCpuCommByServo(g_local_servo1000->cpu.servo_id);
            if(g_local_servo1000->cpu.servo_cpu_comm_ptr == NULL)
            {
                std::cout<<"servo cpu create comm_ptr failed"<<std::endl;
                goto CORE_COMM_CONFIG_FAILED;


            }
            else
            {
                std::cout<<"servo cpu create comm_ptr success"<<std::endl;
            }
            if(!initServoCpuCommByServo(g_local_servo1000->cpu.servo_cpu_comm_ptr, 
                                        g_local_servo1000->config.from_block_ptr, 
                                        g_local_servo1000->config.from_block_number, 
                                        g_local_servo1000->config.to_block_ptr, 
                                        g_local_servo1000->config.to_block_number))
            {
                std::cout<<"servo cpu init communication channel failed"<<std::endl;
                goto CORE_COMM_CONFIG_FAILED;
            }
            else
            {           
                std::cout<<"servo cpu init communication channel success"<<std::endl;
            }

            // create servo_comm channel by configuration
            int32_t controller_id;
            for(size_t i = 0; i < 8; ++i)
            {
                if(getServoCommControllerIdByServoIndex(i, 
                                                        g_local_servo1000->config.from_block_ptr, 
                                                        g_local_servo1000->config.from_block_number, 
                                                        controller_id))
                {
                    g_local_servo1000->servo[i].servo_comm_ptr = createServoCommByServo(controller_id, g_local_servo1000->cpu.servo_id, i);
                    if(g_local_servo1000->servo[i].servo_comm_ptr == NULL)
                    {
                        std::cout<<"servo "<<i<<" create comm_ptr failed"<<std::endl;
                        goto CORE_COMM_CONFIG_FAILED;
                    }
                    else
                    {
                        initCommSm(&g_local_servo1000->servo[i].core_comm_sm, g_local_servo1000->servo[i].servo_comm_ptr);
                        std::cout<<"servo "<<i<<" create comm_ptr success"<<std::endl;
                    }
                }        
            }
            
            // init non pdo communication channel, set comm_state to INIT 
            for(size_t i = 0; i < 8; ++i)
            {
                if(g_local_servo1000->servo[i].servo_comm_ptr != NULL) // imply the servo is configured
                {
                    if(!initServoCommInit2PreOpByServo(g_local_servo1000->servo[i].servo_comm_ptr, 
                                                        g_local_servo1000->config.from_block_ptr, 
                                                        g_local_servo1000->config.from_block_number, 
                                                        g_local_servo1000->config.to_block_ptr, 
                                                        g_local_servo1000->config.to_block_number))
                    {
                        std::cout<<"servo "<<i<<" init2preop failed"<<std::endl;
                        goto CORE_COMM_CONFIG_FAILED;
                    }
                    else
                    {
                        g_local_servo1000->servo[i].core_comm_sm.expect_state = (int32_t)CORE_COMM_STATE_PREOP;
			            g_local_servo1000->config.core_comm_config_ready = true;
                        std::cout<<"servo "<<i<<" trans to preop"<<std::endl;
                    }
                }
            }
	    }
        }
    }while(!g_local_servo1000->config.core_comm_config_ready);

    return;

CORE_COMM_CONFIG_FAILED:
	return;
}
/*-----------------------------------------------------------------
 * 函数名称:void processCtrlAppData4000(ServoComm_t* comm_ptr)
 * 函数输入:none
 * 函数输出:none
 * 函数描述:控制指令字
 * ----------------------------------------------------------------*/
static void Servo_Axis_CtrlAppData4000(ServoComm_t* comm_ptr)
{
	static int64_t last_point[AXIS_MAX] = {0};
	int64_t temp_lsb = 0;
	int64_t temp_msb = 0;

	if(!motion_traj[comm_ptr->servo_index]->motion_flag)
	{
		if((g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_sync == 0)
			&& g_local_servo1000->cpu.ctrl_pdo_sync_reg[comm_ptr->servo_index])    // start to pull signal emit by controller
		{
			g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_sync = 1;
			if(getOccupiedElementNumber(comm_ptr->ctrl_pdo_ptr) >= 1)
			{
				pullCircleBufferType1(comm_ptr->ctrl_pdo_ptr, (uint8_t*)&g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode, 1);

				motion_traj[comm_ptr->servo_index]->motion_flag = 1;
				motion_traj[comm_ptr->servo_index]->pos_offset =
							position_back_info[comm_ptr->servo_index]->position_back;

				Servo_Traj_Set_ContinueMotion(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index),g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_lsb,
						g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_msb,
						g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.feedforward_velocity,
						last_point[comm_ptr->servo_index]);

				temp_lsb = (int64_t)g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_lsb;
				temp_msb = (int64_t)g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_msb;

				last_point[comm_ptr->servo_index] = temp_lsb|(temp_msb<<32);
			}
		}
		else if(g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_sync == 1)   // continuous pulling ctrl pdo
		{
			if(getOccupiedElementNumber(comm_ptr->ctrl_pdo_ptr) >= 1)
		
	{
				pullCircleBufferType1(comm_ptr->ctrl_pdo_ptr, (uint8_t*)&g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode, 1);

				motion_traj[comm_ptr->servo_index]->motion_flag = 1;

				Servo_Traj_Set_ContinueMotion(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index),g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_lsb,
										g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_msb,
										g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.feedforward_velocity,
										last_point[comm_ptr->servo_index]);

				temp_lsb = (int64_t)g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_lsb;
				temp_msb = (int64_t)g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_msb;
				last_point[comm_ptr->servo_index] = temp_lsb|(temp_msb<<32);


			}
			else    // all buffered ctrl pdos have been pulled out
			{
				setServoCpuCommCtrlPdoSync(g_local_servo1000->cpu.servo_cpu_comm_ptr,
						axis_para_local_info[comm_ptr->servo_index]->sync_reg
						,0); // reset the bit
				printf("axis_id=%d,motion completely\r\n",comm_ptr->servo_index);
				g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_sync = 0;
				g_local_servo1000->cpu.ctrl_pdo_sync_reg[comm_ptr->servo_index] = 0;
			}
		}
		else{}  // wait pull signal
	}
}
/*-----------------------------------------------------------------
 * 函数名称:void processFdbAppData3000(ServoComm_t* comm_ptr)
 * 函数输入:none
 * 函数输出:none
 * 函数描述:返回数据
 * ----------------------------------------------------------------*/
static void Servo_Axis_FdbAppData3000(ServoComm_t* comm_ptr)
{
	g_local_servo1000->servo[comm_ptr->servo_index].fdb_pdo_position_mode.time_stamp
											= g_local_servo1000->time_stamp;
	g_local_servo1000->servo[comm_ptr->servo_index].fdb_pdo_position_mode.state_word.all
											= g_local_servo1000->servo[comm_ptr->servo_index].servo_sm.state_word.all;
	//g_local_servo1000->servo[comm_ptr->servo_index].fdb_pdo_position_mode.fdb_position_lsb
	//										= g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_lsb;
	//g_local_servo1000->servo[comm_ptr->servo_index].fdb_pdo_position_mode.fdb_position_msb
	//										= g_local_servo1000->servo[comm_ptr->servo_index].ctrl_pdo_position_mode.cmd_position_msb;;
	g_local_servo1000->servo[comm_ptr->servo_index].fdb_pdo_position_mode.fdb_velocity
											= g_local_servo1000->servo[comm_ptr->servo_index].actual_velocity;
	g_local_servo1000->servo[comm_ptr->servo_index].fdb_pdo_position_mode.actual_op_mode = *g_local_servo1000->servo[comm_ptr->servo_index].servo_mode;

    pushCircleBufferType2(comm_ptr->fdb_pdo_ptr, (uint8_t*)&g_local_servo1000->servo[comm_ptr->servo_index].fdb_pdo_position_mode);
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_p_Cal_SpLoop(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:速度环路计算，最终输出iq电流值
 * ----------------------------------------------------------------*/
static void Servo_Axis_CtrlPdo(ServoComm_t* comm_ptr)
{
    switch(*g_local_servo1000->servo[comm_ptr->servo_index].servo_mode)   // op_mode
    {
        case SERVO_OP_MODE_INTERPOLATED_POSITION_MODE: Servo_Axis_CtrlAppData4000(comm_ptr); break;
        default: ;
    }
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_p_Cal_SpLoop(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:速度环路计算，最终输出iq电流值
 * ----------------------------------------------------------------*/
static void Servo_Axis_FdbPdo(ServoComm_t* comm_ptr)
{
    switch(*g_local_servo1000->servo[comm_ptr->servo_index].servo_mode)   // op_mode
    {
    	case SERVO_OP_MODE_VELOCITY_MODE:
    	case SERVO_OP_MODE_PROFILE_VELOCITY_MODE:
    	case SERVO_OP_MODE_PROFILE_POSITION_MODE:
        case SERVO_OP_MODE_INTERPOLATED_POSITION_MODE:  Servo_Axis_FdbAppData3000(comm_ptr); break;
        default: ;
    }
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_p_Cal_SpLoop(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:速度环路计算，最终输出iq电流值
 * ----------------------------------------------------------------*/
 void virtual_servo_device::servo_axis_p_machine_state(SERVO_AXIS_ENUM axis_id)
{
	static uint32_t current_state[AXIS_MAX] = {0};
	static uint32_t pre_state[AXIS_MAX] = {0};

	uint32_t temp_high = 0;

    g_local_servo1000->servo[axis_id].servo_sm.run((void*)&g_local_servo1000->servo[axis_id].servo_sm);
    g_local_servo1000->servo[axis_id].servo_sm.ctrl_word_back.all
                                = g_local_servo1000->servo[axis_id].servo_sm.ctrl_word.all;

    g_local_servo1000->cpu.ctrl_pdo_sync_reg[axis_id] = getServoCpuCommCtrlPdoSync(g_local_servo1000->cpu.servo_cpu_comm_ptr
    		,axis_para_local_info[axis_id]->sync_reg);
    g_local_servo1000->cpu.sampling_sync_reg_back = g_local_servo1000->cpu.sampling_sync_reg;
    g_local_servo1000->cpu.sampling_sync_reg = getServoCpuCommSamplingSync(g_local_servo1000->cpu.servo_cpu_comm_ptr);

    current_state[axis_id] = g_local_servo1000->servo[axis_id].servo_sm.state_word.all&0x7F;

    temp_high = current_state[axis_id]^pre_state[axis_id];

    pre_state[axis_id] = current_state[axis_id];

    if(temp_high)
    {
    	switch((g_local_servo1000->servo[axis_id].servo_sm.state_word.all&0x7f))
    	{
    	    case(0x40):
    		{
    	    	Servo_Axis_Set_AxisState(axis_id,SERVO_SM_SWITCH_ON_DISABLED);
    	    	break;
    		}
    	    case(0x20):
    		{
    	    	Servo_Axis_Set_AxisState(axis_id,SERVO_SM_READY_TO_SWITCH_ON);
    	    	break;
    		}
    	    case(0x27):
    		{
    	    	Servo_Axis_Set_AxisState(axis_id,SERVO_SM_OPERATION_ENABLED);
    	    	break;
    		}
    	    case(0x23):
    		{
    	    	Servo_Axis_Set_AxisState(axis_id,SERVO_SM_SWITCHED_ON);
    	    	break;
    		}
    	    case(0x7):
    		{
    	    	Servo_Axis_Set_AxisState(axis_id,SERVO_SM_QUICK_STOP_ACTIVE);
    	    	break;
    		}
    	    case(0x0F):
    		{
    	    	Servo_Axis_Set_AxisState(axis_id,SERVO_SM_FAULT_REACTION_ACTIVE);
    	    	break;
    		}
    	    case(0x8):
    		{
    	    	Servo_Axis_Set_AxisState(axis_id,SERVO_SM_FAULT);
    	    	break;
    		}
    	    default:break;
    	}
    }


}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_p_Cal_SpLoop(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:速度环路计算，最终输出iq电流值
 * ----------------------------------------------------------------*/
 void virtual_servo_device::servo_axis_p_comm_state(SERVO_AXIS_ENUM axis_id)
{

    if(g_local_servo1000->config.core_comm_config_ready)
    {
        g_local_servo1000->servo[axis_id].core_comm_sm.run(&g_local_servo1000->servo[axis_id].core_comm_sm);
        CoreCommState_e comm_state = getServoCommState(g_local_servo1000->servo[axis_id].servo_comm_ptr);

        switch(comm_state)
        {
            case CORE_COMM_STATE_OP:
            {
            	Servo_Axis_CtrlPdo(g_local_servo1000->servo[axis_id].servo_comm_ptr);
            }
            case CORE_COMM_STATE_SAFEOP:
            {
            	Servo_Axis_FdbPdo(g_local_servo1000->servo[axis_id].servo_comm_ptr);
            }
            case CORE_COMM_STATE_PREOP:
            {// SZYFIX: change the name of processCoreProcessCall
            	process_core_process_call(g_local_servo1000->servo[axis_id].servo_comm_ptr); break;
            }
            default:break;
        }
    }else{

    }
}

#define DEALY_COUNT 500000
/*-----------------------------------------------------------------
 * 函数名称:void servo_axis_b_down_para(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:对应轴下发参数
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_b_DownPara(SERVO_AXIS_ENUM axis_id)
{
    int32_t get_data_length = 0;

    static int32_t down_delay_count[AXIS_MAX] = {0};
    static int32_t down_para_state[AXIS_MAX] = {0};

    if(g_local_servo1000->servo[axis_id].download_param_running)
    {
    	switch(down_para_state[axis_id])
		{
    		case(0):
			{
    			 getBufferType1(g_local_servo1000->servo[axis_id].servo_comm_ptr->download_param_buffer_ptr,
    			                      (uint8_t*)axis_para_local_info[axis_id],
    			                      &get_data_length);
    			 Servo_Para_SetPara(axis_id,0,(get_data_length>>2));

    			 down_para_state[axis_id] = 1;
    			break;
			}
    		case(1):
			{
    			  if(down_delay_count[axis_id]<DEALY_COUNT)
    			  {
    			      down_delay_count[axis_id]++;
    			  }else{
    			      sendCoreProcessCallAsyncAck(g_local_servo1000->servo[axis_id].servo_comm_ptr->service_ptr);
    			      g_local_servo1000->servo[axis_id].download_param_running = false;
    			      down_delay_count[axis_id]=0;
    			      down_para_state[axis_id] = 0;
    			  }
    			break;
			}
    		default:break;
		}
    }
}
/*-----------------------------------------------------------------
 * 函数名称:void servo_axis_b_upload_para(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:对应上传参数
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_b_UploadPara(SERVO_AXIS_ENUM axis_id)
{
	static int32_t delay_upload_count[AXIS_MAX] = {0};
	static int32_t up_para_state[AXIS_MAX] = {0};

    if(g_local_servo1000->servo[axis_id].upload_param_running)
    {
    	switch(up_para_state[axis_id])
    	{
			case(0):
			{
				 setBufferType1(g_local_servo1000->servo[axis_id].servo_comm_ptr->upload_param_buffer_ptr,
															(uint8_t*)axis_para_inter_info[axis_id],
																   sizeof(BufferAppData2002_t));
				 up_para_state[axis_id] = 1;
				break;
			}
			case(1):
			{
			     if(delay_upload_count[axis_id]<DEALY_COUNT)
			     {
			        	delay_upload_count[axis_id]++;
			     }else{
			        	 sendCoreProcessCallAsyncAck(g_local_servo1000->servo[axis_id].servo_comm_ptr->service_ptr);
			        	 g_local_servo1000->servo[axis_id].upload_param_running = false;
			        	 delay_upload_count[axis_id] = 0;
			        	 up_para_state[axis_id] = 0;
			      }
			     break;
			}
			default:break;
    	}
    }
}
/*-----------------------------------------------------------------
 * 函数名称:void servo_axis_test_dignose(void)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:测试函数
 * ----------------------------------------------------------------*/
 void virtual_servo_device::servo_axis_test_dignose(void)
{
	g_local_servo1000->time_stamp++;
}
/*-----------------------------------------------------------------
 * 函数名称:void print_axis_state(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:测试函数
 * ----------------------------------------------------------------*/
 void virtual_servo_device::print_axis_state(SERVO_AXIS_ENUM axis_id)
{
	printf("axis_id=%d,axis_state=%d\n",axis_id,axis_state[axis_id]);
	for(uint32_t i=0; i<5000000;i++);
}
/*-----------------------------------------------------------------
 * 函数名称:void servo_axis_b_upload_para(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:对应上传参数
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_Set_AxisState(SERVO_AXIS_ENUM axis_id
		,ServoSm_e servo_state)
{
	assert(axis_id<AXIS_MAX);
	axis_state[axis_id] = servo_state;
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_Set_AxisErr(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:对应上传参数
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_Set_AxisErr(SERVO_AXIS_ENUM axis_id)
{
	setStateFaultReactionLine(&g_local_servo1000->servo[axis_id].servo_sm);
	Servo_Axis_Set_AxisState(axis_id,SERVO_SM_FAULT_REACTION_ACTIVE);
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_CheckErr(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数描述:轴错误检测
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_CheckErr(SERVO_AXIS_ENUM axis_id)
{
	Servo_IQ_Overload_Check(axis_id);
	Servo_Encode_Err_Check(axis_id);
	Servo_PhaseErr_Check(axis_id);
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_DriverErr(void)
 * 函数输入:none
 * 函数输出:none
 * 函数描述:驱动板错误检测，包含电压，IPM故障信号等
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_DriverErr(void)
{
	if((axis_state[AXIS_0]==SERVO_SM_OPERATION_ENABLED)
			||axis_state[AXIS_1]==SERVO_SM_OPERATION_ENABLED
			||axis_state[AXIS_2]==SERVO_SM_OPERATION_ENABLED
			||axis_state[AXIS_3]==SERVO_SM_OPERATION_ENABLED
			||axis_state[AXIS_4]==SERVO_SM_OPERATION_ENABLED
			||axis_state[AXIS_5]==SERVO_SM_OPERATION_ENABLED
			||axis_state[AXIS_6]==SERVO_SM_OPERATION_ENABLED
			||axis_state[AXIS_7]==SERVO_SM_OPERATION_ENABLED)
	{
		Servo_DCBUS_Vol_Check();
	}

	Servo_Ipm_OverCur_Check();
	Servo_Temp_Check();
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_DriverErr(void)
 * 函数输入:none
 * 函数输出:none
 * 函数描述:驱动板错误检测，包含电压，IPM故障信号等
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_UpdateContrcmd(SERVO_AXIS_ENUM axis_id)
{
	axis_para_local_info[axis_id]->monitor_info.operation_mode_act =
			axis_para_local_info[axis_id]->ref_cmd_info.expect_mode;

	*g_local_servo1000->servo[axis_id].servo_mode =
			axis_para_local_info[axis_id]->ref_cmd_info.expect_mode;

    /*指令值更新，前馈值更新*/
}
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_UpdateMonitor(SERVO_AXIS_ENUM axis_id)
 * 函数输入:none
 * 函数输出:none
 * 函数描述:更新观测量到表单
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_UpdateMonitor(SERVO_AXIS_ENUM axis_id)
{
	/*监控变量更新*/
	axis_para_local_info[axis_id]->monitor_info.current_torque
								= sp_loop_info[axis_id].con_pid_out;
}

#define WINDOW_ERR 233026
#define SETTING_TIME 10000
/*-----------------------------------------------------------------
 * 函数名称:void Servo_Axis_Check_ArrivePos(SERVO_AXIS_ENUM axis_id)
 * 函数输入:none
 * 函数输出:none
 * 函数描述:更新观测量到表单
 * ----------------------------------------------------------------*/
 void virtual_servo_device::Servo_Axis_Check_ArrivePos(SERVO_AXIS_ENUM axis_id)
{
	uint64_t temp_data = 0;
	static uint32_t setting_count[AXIS_MAX] = {0};

	switch(*g_local_servo1000->servo[axis_id].servo_mode)
	{
		case(SERVO_OP_MODE_INTERPOLATED_POSITION_MODE):
		case(SERVO_OP_MODE_PROFILE_POSITION_MODE):
		{
			temp_data = 0;//(pos_loop_info[axis_id].ctl_err>0)?(pos_loop_info[axis_id].ctl_err):(0-pos_loop_info[axis_id].ctl_err);

			if(temp_data<WINDOW_ERR)
			{
				if(setting_count[axis_id]<SETTING_TIME)
				{
					setting_count[axis_id]++;
				}else{
					setting_count[axis_id] = 0;
					g_local_servo1000->servo[axis_id].servo_sm.state_word.bit.target_reached = 1;
				}
			}else{
				g_local_servo1000->servo[axis_id].servo_sm.state_word.bit.target_reached = 0;
				setting_count[axis_id] = 0;
			}
			break;
		}
		case(SERVO_OP_MODE_PROFILE_VELOCITY_MODE):
		case(SERVO_OP_MODE_VELOCITY_MODE):
		{
			temp_data = 0;//(sp_loop_info[axis_id].ctl_err>0)?(sp_loop_info[axis_id].ctl_err):(0-sp_loop_info[axis_id].ctl_err);

			if(temp_data<WINDOW_ERR)
			{
				if(setting_count[axis_id]<SETTING_TIME)
				{
						setting_count[axis_id]++;
				}else{
						setting_count[axis_id] = 0;
						g_local_servo1000->servo[axis_id].servo_sm.state_word.bit.target_reached = 1;
				}
			}else{
				g_local_servo1000->servo[axis_id].servo_sm.state_word.bit.target_reached = 0;
				setting_count[axis_id] = 0;
			}
			break;
		}
		default:break;
	}


}

