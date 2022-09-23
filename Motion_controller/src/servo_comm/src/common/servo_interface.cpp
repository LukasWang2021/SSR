#ifdef COMPILE_IN_BARE
#include "./servo_comm_inc/servo_interface.h"
#include "./core_protocal_inc/core_comm_config_base.h"
#include "./core_protocal_inc/buffer_base.h"
#include "./core_protocal_inc/circle_buffer_base.h"
#include "./core_protocal_inc/core_process_call_base.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#else
#include "common/servo_interface.h"
#include "common/core_comm_config_base.h"
#include "common/buffer_base.h"
#include "common/circle_buffer_base.h"
#include "common/core_process_call_base.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#endif

ServoComm_t* createServoCommByController(int32_t controller_id, int32_t servo_id, int32_t servo_index)
{
    ServoComm_t* comm_ptr = (ServoComm_t*)malloc(sizeof(ServoComm_t));
    if(comm_ptr == NULL)
    {
        return NULL;
    }

    comm_ptr->servo_index = servo_index;
    comm_ptr->from = controller_id;
    comm_ptr->to = servo_id;
    comm_ptr->comm_reg_ptr = NULL;
    comm_ptr->service_ptr = NULL;
    comm_ptr->download_param_buffer_ptr = NULL;
    comm_ptr->upload_param_buffer_ptr = NULL;
    comm_ptr->ctrl_pdo_ptr = NULL;
    comm_ptr->fdb_pdo_ptr = NULL;

    return comm_ptr;
}

bool initServoCommInit2PreOpByController(ServoComm_t* comm_ptr,
                                                    CommBlockData_t* from_block_ptr, size_t from_block_number,
                                                    CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    if(from_block_ptr == NULL
        || to_block_ptr == NULL)
    {
        return false;
    }
    
    for(size_t i = 0; i < from_block_number; ++i)
    {
        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].param1 == comm_ptr->servo_index
            && from_block_ptr[i].application_id == SERVO_COMM_APP_ID_SERVICE)
        {
            comm_ptr->service_ptr = &from_block_ptr[i];
            continue;
        }

        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].param1 == comm_ptr->servo_index
            && from_block_ptr[i].application_id == SERVO_COMM_APP_ID_DOWNLOAD_PARAM)
        {
            comm_ptr->download_param_buffer_ptr = &from_block_ptr[i];
            continue;
        }
    }

    for(size_t i = 0; i < to_block_number; ++i)
    {
        if(to_block_ptr[i].from == comm_ptr->to
            && to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].param1 == comm_ptr->servo_index
            && to_block_ptr[i].application_id == SERVO_COMM_APP_ID_COMM_REG)
        {
            comm_ptr->comm_reg_ptr = &to_block_ptr[i];
            continue;
        } 
            
        if(to_block_ptr[i].from == comm_ptr->to
            && to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].param1 == comm_ptr->servo_index
            && to_block_ptr[i].application_id == SERVO_COMM_APP_ID_UPLOAD_PARAM)
        {
            comm_ptr->upload_param_buffer_ptr = &to_block_ptr[i];
            continue;
        }            
    }

    if(comm_ptr->comm_reg_ptr == NULL
        || comm_ptr->service_ptr == NULL
        || comm_ptr->download_param_buffer_ptr == NULL
        || comm_ptr->upload_param_buffer_ptr == NULL)
    {
        return false;
    }
    else
    {
        return true;
    }
}
                                                            
bool initServoCommPreOp2SafeOpByController(ServoComm_t* comm_ptr, CommBlockData_t* to_block_ptr, size_t to_block_number, int32_t app_id)
{
    if(comm_ptr == NULL
       || to_block_ptr == NULL)
    {
        return false;
    }
       
    for(size_t i = 0; i < to_block_number; ++i)
    {
        if(to_block_ptr[i].from == comm_ptr->to
            && to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].param1 == comm_ptr->servo_index
            && to_block_ptr[i].application_id == app_id)
        {
            comm_ptr->fdb_pdo_ptr = &to_block_ptr[i];
            continue;
        } 
    }
    
    if(comm_ptr->fdb_pdo_ptr == NULL
        && app_id != -1)    // -1 means don't need fdb pdo
    {
        return false;
    }
    else
    {
        return true;
    }

}

bool initServoCommSafeOp2OpByController(ServoComm_t* comm_ptr, CommBlockData_t* from_block_ptr, size_t from_block_number, int32_t app_id)
{
    if(comm_ptr == NULL
       || from_block_ptr == NULL)
    {
        return false;
    }
       
    for(size_t i = 0; i < from_block_number; ++i)
    {
        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].param1 == comm_ptr->servo_index
            && from_block_ptr[i].application_id == app_id)
        {
            comm_ptr->ctrl_pdo_ptr = &from_block_ptr[i];
            continue;
        }
    }

    if(comm_ptr->ctrl_pdo_ptr == NULL
        && app_id != -1)    // -1 means don't need ctrl pdo
    {
        return false;
    }
    else
    {
        return true;
    }

}

bool doServoCmdFastService(CommBlockData_t* block_ptr, CoreProcessCallAppData1000_t* req_data_ptr)
{
    if(block_ptr == NULL
        || req_data_ptr == NULL)
    {
        return false;
    }
        
    return sendCoreProcessCallRequest(block_ptr, (uint8_t*)req_data_ptr, sizeof(CoreProcessCallAppData1000_t), false);
}

bool doServoCmdNormalService(CommBlockData_t* block_ptr, 
                                    CoreProcessCallAppData1000_t* req_data_ptr, 
                                    CoreProcessCallAppData1000_t* res_data_ptr,
                                    bool enable_async)
{
    if(block_ptr == NULL
        || req_data_ptr == NULL
        || res_data_ptr == NULL)
    {
        return false;
    }

    if(!sendCoreProcessCallRequest(block_ptr, (uint8_t*)req_data_ptr, sizeof(CoreProcessCallAppData1000_t), enable_async))
    {
        return false;
    }
    bool ret, timeout;
    int32_t app_data_size;
    while(1)
    {        
        usleep(10000);
        ret = recvCoreProcessCallResponse(block_ptr, (uint8_t*)res_data_ptr, &app_data_size, &timeout);
        if(ret || timeout)
        {
            break;
        }
    }

    if(timeout)
    {
        printf("ERROR:: %s\n", "return time out");
        return false;
    }
    if(res_data_ptr->cmd == req_data_ptr->cmd)
    {
        return true;
    }
    else
    {
        printf("ERROR:: cmd return res & req value:%d \t %d", res_data_ptr->cmd, req_data_ptr->cmd);
        return false;
    }
}

bool doServoCmdAsyncService(CommBlockData_t* block_ptr, 
                                    CoreProcessCallAppData1000_t* req_data_ptr, 
                                    CoreProcessCallAppData1000_t* res_data_ptr, 
                                    int32_t** async_ack_ptr_ptr)
{
    if(doServoCmdNormalService(block_ptr, req_data_ptr, res_data_ptr, true))
    {
        *async_ack_ptr_ptr = CORE_PROCESS_CALL_ASYNC_ACK_PTR;
        return true;
    }
    else
    {
        *async_ack_ptr_ptr = NULL;
        return false;
    }
}

bool isAsyncServiceDone(int32_t* async_ack_ptr)
{
    return ((*async_ack_ptr) == 1) ? true : false;
}

bool getServoCommControllerIdByServoIndex(int32_t servo_index, CommBlockData_t* from_block_ptr, size_t from_block_number, int32_t& controller_id)
{
    controller_id = -1;
    for(size_t i = 0; i < from_block_number; ++i)
    {
        if(from_block_ptr[i].type == COMM_BLOCK_TYPE_COMM_REG
            && from_block_ptr[i].param1 == servo_index)
        {
            controller_id = from_block_ptr[i].to;
        }
    }
    if(controller_id == -1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

ServoComm_t* createServoCommByServo(int32_t controller_id, int32_t servo_id, int32_t servo_index)
{
    ServoComm_t* comm_ptr = (ServoComm_t*)malloc(sizeof(ServoComm_t));
    if(comm_ptr == NULL)
    {
        return NULL;
    }
     
    comm_ptr->servo_index = servo_index;
    comm_ptr->from = servo_id;
    comm_ptr->to = controller_id;
    comm_ptr->comm_reg_ptr = NULL;
    comm_ptr->ctrl_pdo_ptr = NULL;
    comm_ptr->fdb_pdo_ptr = NULL;
    comm_ptr->service_ptr = NULL;
    comm_ptr->download_param_buffer_ptr = NULL;
    comm_ptr->upload_param_buffer_ptr = NULL;

    return comm_ptr;
}

bool initServoCommInit2PreOpByServo(ServoComm_t* comm_ptr,
                                             CommBlockData_t* from_block_ptr, size_t from_block_number,
                                             CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    if(from_block_ptr == NULL
        || to_block_ptr == NULL)
    {
        return false;
    }

     
    for(size_t i = 0; i < from_block_number; ++i)
    {
        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].param1 == comm_ptr->servo_index
            && from_block_ptr[i].application_id == SERVO_COMM_APP_ID_COMM_REG)
        {
            comm_ptr->comm_reg_ptr = &from_block_ptr[i];
            continue;
        }
             
        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].param1 == comm_ptr->servo_index
            && from_block_ptr[i].application_id == SERVO_COMM_APP_ID_UPLOAD_PARAM)
        {
            comm_ptr->upload_param_buffer_ptr = &from_block_ptr[i];
            continue;
        }           
    }
     
    for(size_t i = 0; i < to_block_number; ++i)
    {
        if(to_block_ptr[i].from == comm_ptr->to
            && to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].param1 == comm_ptr->servo_index
            && to_block_ptr[i].application_id == SERVO_COMM_APP_ID_SERVICE)
        {
            comm_ptr->service_ptr = &to_block_ptr[i];
            continue;
        }            
             
        if(to_block_ptr[i].from == comm_ptr->to
            && to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].param1 == comm_ptr->servo_index
            && to_block_ptr[i].application_id == SERVO_COMM_APP_ID_DOWNLOAD_PARAM)
        {
            comm_ptr->download_param_buffer_ptr = &to_block_ptr[i];
            continue;
        }            
    }
     
    if(comm_ptr->comm_reg_ptr == NULL
        || comm_ptr->service_ptr == NULL
        || comm_ptr->download_param_buffer_ptr == NULL
        || comm_ptr->upload_param_buffer_ptr == NULL)     
    {
        return false;
    }
    else
    {
        return true;
    }
}
                                                 
bool initServoCommPreOp2SafeOpByServo(ServoComm_t* comm_ptr, CommBlockData_t* from_block_ptr, size_t from_block_number, int32_t app_id)
{
    if(comm_ptr == NULL
       || from_block_ptr == NULL)
    {
        return false;
    }
     
    for(size_t i = 0; i < from_block_number; ++i)
    {
        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].param1 == comm_ptr->servo_index
            && from_block_ptr[i].application_id == app_id)
        {
            comm_ptr->fdb_pdo_ptr = &from_block_ptr[i];
            continue;
        } 
    }
     
    if(comm_ptr->fdb_pdo_ptr == NULL
        && app_id != -1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool initServoCommSafeOp2OpByServo(ServoComm_t* comm_ptr, CommBlockData_t* to_block_ptr, size_t to_block_number, int32_t app_id)
{
    if(comm_ptr == NULL 
        || to_block_ptr == NULL)
    {
        return false;
    }
    
    for(size_t i = 0; i < to_block_number; ++i)
    {
        if(to_block_ptr[i].from == comm_ptr->to
            && to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].param1 == comm_ptr->servo_index
            && to_block_ptr[i].application_id == app_id)
        {
            comm_ptr->ctrl_pdo_ptr = &to_block_ptr[i];
            continue;
        }  
    }

    if(comm_ptr->ctrl_pdo_ptr == NULL
        && app_id != -1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool setServoCommState(ServoComm_t* comm_ptr, CoreCommState_e comm_state)
{
    if(comm_ptr == NULL
        || comm_ptr->comm_reg_ptr == NULL
        || comm_state == CORE_COMM_STATE_UNKNOWN)
    {
        return false;
    }
    setCommReg0State(comm_ptr->comm_reg_ptr, (int32_t)comm_state);
    return true;
}

CoreCommState_e getServoCommState(ServoComm_t* comm_ptr)
{
    if(comm_ptr == NULL
        || comm_ptr->comm_reg_ptr == NULL)
    {
        return CORE_COMM_STATE_UNKNOWN;
    }
        
    int32_t state;
    getCommReg0State(comm_ptr->comm_reg_ptr, &state);
    return (CoreCommState_e)state;
}

void freeServoComm(ServoComm_t* comm_ptr)
{
    if(comm_ptr != NULL)
    {
        free(comm_ptr);
    }
}


