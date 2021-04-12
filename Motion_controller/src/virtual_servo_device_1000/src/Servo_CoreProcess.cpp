/*
 * Servo_CoreProcess.h
 *
 *  Created on: 2019年7月19日
 *      Author: qichao.wang
 */
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <iostream>
#include "Servo_CoreProcess.h"
#include "Servo_General.h"
#include "Servo_Para.h"
#include "Servo_Motor.h"
#include "Servo_Axis.h"
#include "error_process.h"
#include "Servo_Safety.h"
#include "Servo_Inter_SpeedTraj.h"
#include "error_process.h"
using namespace std;
using namespace virtual_servo_device;

static VirtualServo1000_t g_servo1000;

static void (*cmd_ana[SERVO_CMD_MAX])(ServoComm_t* comm_ptr,
                                         CoreProcessCallAppData1000_t* req_data_ptr, CoreProcessCallAppData1000_t* res_data_ptr);

static void handleTransCommState(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr
                                 , CoreProcessCallAppData1000_t* res_data_ptr)
{
    bool result = true;
    CoreCommState_e current_comm_state = getServoCommState(comm_ptr);
    if(current_comm_state == CORE_COMM_STATE_PREOP && req_data_ptr->param1 == (int32_t)CORE_COMM_STATE_SAFEOP)
    {
        switch(*g_servo1000.servo[comm_ptr->servo_index].servo_mode)   // op_mode is relate to pdo
        {
            case SERVO_OP_MODE_INTERPOLATED_POSITION_MODE:
            {
                if(!initServoCommPreOp2SafeOpByServo(comm_ptr, g_servo1000.config.from_block_ptr, g_servo1000.config.from_block_number, 3000))
                {
                    result = false;
                }
                break;
            }
            default:result = false;
        }

        if(result)
        {
            g_servo1000.servo[comm_ptr->servo_index].core_comm_sm.expect_state = (int32_t)CORE_COMM_STATE_SAFEOP;
        }
    }
    else if(current_comm_state == CORE_COMM_STATE_SAFEOP
            && req_data_ptr->param1 == (int32_t)CORE_COMM_STATE_OP)
    {
        switch(*g_servo1000.servo[comm_ptr->servo_index].servo_mode)   // op_mode is relate to pdo
        {
            case SERVO_OP_MODE_INTERPOLATED_POSITION_MODE:
            {
                if(!initServoCommSafeOp2OpByServo(comm_ptr, g_servo1000.config.to_block_ptr
                                                  , g_servo1000.config.to_block_number, 4000))
                {
                    result = false;
                }
                break;
            }
            default:result = false;
        }
        if(result)
        {
            g_servo1000.servo[comm_ptr->servo_index].core_comm_sm.expect_state =   (int32_t)CORE_COMM_STATE_OP;
        }
    }
    else if(current_comm_state == req_data_ptr->param1)
    {
        result = true;
    }
    else
    {
        result = false;
        res_data_ptr->cmd = SERVO_CMD_TRANS_COMM_STATE;
        res_data_ptr->param1 = -1;
    }

    res_data_ptr->cmd = SERVO_CMD_TRANS_COMM_STATE;
    res_data_ptr->param1 = result ? 0 : -1;
}

static void handleShutDown(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr
                           , CoreProcessCallAppData1000_t* res_data_ptr)
{
    setCtrlShutDown(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleShutDown%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);

    Servo_Safety_b_Write_Cmd_Ana(RESET_SAFETY);
    //WRITE_HPS(SAFETY_RESET_ADD,1);//reset
    //WRITE_HPS(SAFE_BASE_ADD+SAFE_WRITE_OFFSET,0);//清除错误

    res_data_ptr->cmd = SERVO_CMD_SHUT_DOWN;
    res_data_ptr->param1 = 0;
}

static void handleSwitchOn(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr,
                           CoreProcessCallAppData1000_t* res_data_ptr)
{
    setCtrlSwitchOn(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleSwitchOn%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);

    Servo_Safety_b_Write_Cmd_Ana(EXCITATION);
    res_data_ptr->cmd = SERVO_CMD_SWITCH_ON;
    res_data_ptr->param1 = 0;

}

static void handleDisableVoltage(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr
                                 , CoreProcessCallAppData1000_t* res_data_ptr)
{
    setCtrlDisableVoltage(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleDisableVoltage%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);
    Servo_SpTraj_RestVar(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index));
    res_data_ptr->cmd = SERVO_CMD_DISABLE_VOLTAGE;
    res_data_ptr->param1 = 0;
}

static void handleEnableOperation(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr,
                                  CoreProcessCallAppData1000_t* res_data_ptr)
{
    setCtrlEnableOperation(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleEnableOperation,%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);
    Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index),MOTOR_ENABLE);
    res_data_ptr->cmd = SERVO_CMD_ENABLE_OPERATION;
    res_data_ptr->param1 = 0;
}

static void handleSwitchOnAndEnableOperation(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr,
                                             CoreProcessCallAppData1000_t* res_data_ptr)
{
    setCtrlSwitchOnAndEnableOperation(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleSwitchOnAndEnableOperation,%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);
    Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index),MOTOR_ENABLE);
    res_data_ptr->cmd = SERVO_CMD_SWITCH_ON_AND_ENABLE_OPERATION;
    res_data_ptr->param1 = 0;
}

static void handleDisableOperation(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr
                                   , CoreProcessCallAppData1000_t* res_data_ptr)
{
    setCtrlDisableOperation(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleDisableOperation,%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);
    Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index),MOTOR_DISABLE);
    Servo_Safety_b_Write_Cmd_Ana(EXCITATION);
    res_data_ptr->cmd = SERVO_CMD_DISABLE_OPERATION;
    res_data_ptr->param1 = 0;
}

static void handleQuickStop(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr
                            , CoreProcessCallAppData1000_t* res_data_ptr)
{
    setCtrlQuickStop(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleQuickStop,%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);
    res_data_ptr->cmd = SERVO_CMD_QUICK_STOP;
    res_data_ptr->param1 = 0;
}

static void handleFaultReset(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr,
                             CoreProcessCallAppData1000_t* res_data_ptr)
{
    setCtrlFaultReset(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleFaultReset,%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);
    Servo_Axis_Set_TargetPoint(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index));
    error_process_reset_errcode(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index));
    res_data_ptr->cmd = SERVO_CMD_FAULT_RESET;
    res_data_ptr->param1 = 0;
}

static void handleResetFaultReset(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr,
                                  CoreProcessCallAppData1000_t* res_data_ptr)
{
    resetCtrlFaultReset(&g_servo1000.servo[comm_ptr->servo_index].servo_sm.ctrl_word);
    printf("handleResetFaultReset%x\r\n",g_servo1000.servo[comm_ptr->servo_index].servo_sm.state_word.all);
    Servo_Axis_Set_TargetPoint(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index));
    res_data_ptr->cmd = SERVO_CMD_RESET_FAULT_RESET;
    res_data_ptr->param1 = 0;
}

static void handleReadParameter(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr,
                                CoreProcessCallAppData1000_t* res_data_ptr)
{
   // res_data_ptr->param2 = *g_servo1000.servo[comm_ptr->servo_index].servo_mode;
	res_data_ptr->param1 = 0;
    Servo_Para_Manager_ReadPara(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index),&res_data_ptr->param2,req_data_ptr->param1);
    printf("resp_data:%x\r\n",res_data_ptr->param2);
    res_data_ptr->cmd = SERVO_CMD_READ_PARAMETER;
}

static void handleWriteParameter(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr,
                                 CoreProcessCallAppData1000_t* res_data_ptr)
{
    Servo_Para_Manager_WritePara(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index)
    		,req_data_ptr->param2,req_data_ptr->param1);
    res_data_ptr->cmd = SERVO_CMD_WRITE_PARAMETER;
    res_data_ptr->param1 = 0;
}

static void handleUploadParameters(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr
                            , CoreProcessCallAppData1000_t* res_data_ptr)
{
    if(!g_servo1000.servo[comm_ptr->servo_index].upload_param_running)
    {
        g_servo1000.servo[comm_ptr->servo_index].upload_param_running = true;

        res_data_ptr->cmd = SERVO_CMD_UPLOAD_PARAMETERS;
        res_data_ptr->param1 = 0;
    }
    else
    {
        res_data_ptr->cmd = SERVO_CMD_UPLOAD_PARAMETERS;
        res_data_ptr->param1 = -1;
    }
}

static void handleDownloadParameters(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr, CoreProcessCallAppData1000_t* res_data_ptr)
{
    if(!g_servo1000.servo[comm_ptr->servo_index].download_param_running)
    {
        g_servo1000.servo[comm_ptr->servo_index].download_param_running = true;
        res_data_ptr->cmd = SERVO_CMD_DOWNLOAD_PARAMETERS;
        res_data_ptr->param1 = 0;
    }
    else
    {
        res_data_ptr->cmd = SERVO_CMD_DOWNLOAD_PARAMETERS;
        res_data_ptr->param1 = -1;
    }
}

static void handleMoveVelocity(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr, CoreProcessCallAppData1000_t* res_data_ptr)
{
    if(*g_servo1000.servo[comm_ptr->servo_index].servo_mode== SERVO_OP_MODE_PROFILE_VELOCITY_MODE)
    {
    	Servo_SpTraj_SetPara(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index),(int32_t*)req_data_ptr);
        res_data_ptr->cmd = SERVO_CMD_MOVE_VELOCITY;
        res_data_ptr->param1 = 0;
    }
    else
    {
        res_data_ptr->cmd = SERVO_CMD_MOVE_VELOCITY;
        res_data_ptr->param1 = -1;
    }
}

static void handleMoveAbsolute(ServoComm_t* comm_ptr, CoreProcessCallAppData1000_t* req_data_ptr,
                               CoreProcessCallAppData1000_t* res_data_ptr)
{
    /*need add check mode*/
	if(*g_servo1000.servo[comm_ptr->servo_index].servo_mode== SERVO_OP_MODE_PROFILE_POSITION_MODE)
	{
		Servo_PosTraj_SetPara(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index),(int32_t*)req_data_ptr);
	    res_data_ptr->cmd = SERVO_CMD_MOVE_ABSOLUTE;
	    res_data_ptr->param1 = 0;

	}else{
		 res_data_ptr->cmd = SERVO_CMD_MOVE_ABSOLUTE;
		 res_data_ptr->param1 = -1;
	}
}

static void initVirtualServoConfig(VirtualServoConfig_t* config)
{
    config->core_comm_config_ready = false;
    config->from_block_ptr = NULL;
    config->from_block_number = 0;
    config->to_block_ptr = NULL;
    config->to_block_number = 0;
}

static void initVirtualServoCpu(VirtualServoCpu_t* cpu)
{
    cpu->servo_id = 2;
    cpu->isr_count = 0;
    cpu->sampling_sync_reg = 0;
    cpu->sampling_sync_reg_back = 0;
    cpu->servo_cpu_comm_ptr = NULL;

    for(uint8_t i=0; i<AXIS_MAX; i++)
    {
        cpu->ctrl_pdo_sync_reg[i] = 0;
    }
}

static void initVirtualServo(VirtualServo_t* servo)
{
    servo->servo_comm_ptr = NULL;
    servo->core_comm_sm.expect_state = 0;
    servo->core_comm_sm.comm_ptr = NULL;
    servo->core_comm_sm.run = NULL;
    initServoSm(&servo->servo_sm);
    memset(&servo->fdb_pdo_position_mode, 0, sizeof(CircleBufferAppData3000_t));
    memset(&servo->ctrl_pdo_position_mode, 0, sizeof(CircleBufferAppData4000_t));
    servo->ctrl_pdo_sync = 0;
    servo->actual_position_lsb = 0;
    servo->actual_position_msb = 0;
    servo->actual_velocity = 0;
    servo->actual_torque = 0;
    servo->upload_param_running = false;
    servo->download_param_running = false;
}

void virtual_servo_device::servo_cmd_init(void)
{
    cmd_ana[SERVO_CMD_TRANS_COMM_STATE] = handleTransCommState;
    cmd_ana[SERVO_CMD_SHUT_DOWN] = handleShutDown;
    cmd_ana[SERVO_CMD_SWITCH_ON] = handleSwitchOn;
    cmd_ana[SERVO_CMD_DISABLE_VOLTAGE] = handleDisableVoltage;
    cmd_ana[SERVO_CMD_ENABLE_OPERATION] = handleEnableOperation;
    cmd_ana[SERVO_CMD_SWITCH_ON_AND_ENABLE_OPERATION] = handleSwitchOnAndEnableOperation;
    cmd_ana[SERVO_CMD_DISABLE_OPERATION] = handleDisableOperation;
    cmd_ana[SERVO_CMD_QUICK_STOP] = handleQuickStop;
    cmd_ana[SERVO_CMD_FAULT_RESET] = handleFaultReset;
    cmd_ana[SERVO_CMD_RESET_FAULT_RESET] = handleResetFaultReset;
    cmd_ana[SERVO_CMD_READ_PARAMETER] = handleReadParameter;
    cmd_ana[SERVO_CMD_WRITE_PARAMETER] = handleWriteParameter;
    cmd_ana[SERVO_CMD_UPLOAD_PARAMETERS] = handleUploadParameters;
    cmd_ana[SERVO_CMD_DOWNLOAD_PARAMETERS] = handleDownloadParameters;
    cmd_ana[SERVO_CMD_MOVE_VELOCITY] = handleMoveVelocity;
    cmd_ana[SERVO_CMD_MOVE_ABSOLUTE] = handleMoveAbsolute;


    initVirtualServoConfig(&g_servo1000.config);
    initVirtualServoCpu(&g_servo1000.cpu);

    if(!g_servo1000.config.core_comm.initAsSlave())
    {
        std::cout<<"core comm init failed"<<std::endl;  
    }    


    for(size_t i = 0; i < 8; ++i)
    {
        initVirtualServo(&g_servo1000.servo[i]);
        Servo_b_Get_ServoMode(static_cast<SERVO_AXIS_ENUM>(i),&g_servo1000.servo[i].servo_mode);//获取模式
    }


}


bool virtual_servo_device::servo_cmd_get_gservo_ptr(VirtualServo1000_t** g_servo_ptr)
{
    *g_servo_ptr = &g_servo1000;

    return true;
}

static void Servo_Cmd_RespError(SERVO_AXIS_ENUM axis_id)
{
    error_process_report(axis_id,RESP_FAIL);

    Servo_Axis_Set_AxisErr(axis_id);
}



void virtual_servo_device::process_core_process_call(ServoComm_t* comm_ptr)
{
    if(comm_ptr == NULL)
    {
        return;
    }

    CoreProcessCallAppData1000_t req_data, res_data;
    int32_t data_size;

    if(recvCoreProcessCallRequest(comm_ptr->service_ptr,
                                  (uint8_t*)&req_data, &data_size))
    {
        if(req_data.cmd>SERVO_CMD_MAX)
        {
            
            res_data.param1 = UNKNOW_CMD;//next be response by if sendCoreProcessCallResponse

        }else{
            cmd_ana[req_data.cmd](comm_ptr,&req_data,&res_data);
        }

        if(!sendCoreProcessCallResponse(comm_ptr->service_ptr,
                                        (uint8_t*)&res_data, sizeof(CoreProcessCallAppData1000_t)))
        {
            
            Servo_Cmd_RespError(static_cast<SERVO_AXIS_ENUM>(comm_ptr->servo_index));
        }
    }
}

void virtual_servo_device::Servo_Cmd_Get_SamplePtr(ServoCpuComm_t** get_ptr)
{
	if(g_servo1000.cpu.servo_cpu_comm_ptr!=NULL)
	{
		*get_ptr = g_servo1000.cpu.servo_cpu_comm_ptr;
	}

}

