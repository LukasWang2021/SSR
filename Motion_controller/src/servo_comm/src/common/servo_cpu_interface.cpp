#ifdef COMPILE_IN_BARE
#include "./servo_comm_inc/servo_cpu_interface.h"
#include "./core_protocal_inc/core_comm_config_base.h"
#include "./core_protocal_inc/buffer_base.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#else
#include "common/servo_cpu_interface.h"
#include "common/core_comm_config_base.h"
#include "common/buffer_base.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#endif

ServoCpuComm_t* createServoCpuCommByController(int32_t controller_id, int32_t servo_id)
{
    ServoCpuComm_t* comm_ptr = (ServoCpuComm_t*)malloc(sizeof(ServoCpuComm_t));
    if(comm_ptr == NULL)
    {
        return NULL;
    }

    comm_ptr->from = controller_id;
    comm_ptr->to = servo_id;
    comm_ptr->comm_reg_ptr = NULL;
    comm_ptr->sampling_buffer_ptr = NULL;
    comm_ptr->param_reg_ptr = NULL;
    comm_ptr->tsd_reg_pt = NULL;
    return comm_ptr;
}

bool initServoCpuCommByController(ServoCpuComm_t* comm_ptr,
                                            CommBlockData_t* from_block_ptr, size_t from_block_number,
                                            CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    if(comm_ptr == NULL 
    || from_block_ptr == NULL 
    || to_block_ptr == NULL)
    {
        return false;
    }

    for(size_t i = 0; i < from_block_number; ++i)
    {
        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].application_id == SERVO_CPU_COMM_APP_ID_COMM_REG)
        {
            comm_ptr->comm_reg_ptr = &from_block_ptr[i];
            continue;
        }        
    }

    for(size_t i = 0; i < from_block_number; ++i)
    {
        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].application_id == SERVO_CPU_COMM_APP_ID_PARAM_REG)
        {
            comm_ptr->param_reg_ptr = &from_block_ptr[i];
            continue;
        }        
    }
    for(size_t i=0;i< from_block_number;++i)//20210917---xzc
    {
        if(from_block_ptr[i].from == comm_ptr->from
            && from_block_ptr[i].to == comm_ptr->to
            && from_block_ptr[i].application_id == SERVO_CPU_COMM_APP_ID_TSD_REG)
        {
            comm_ptr->tsd_reg_pt = &from_block_ptr[i];
            continue;
        }        
    }
    for(size_t i = 0; i < to_block_number; ++i)
    {
        if(to_block_ptr[i].from == comm_ptr->to
            && to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].application_id == SERVO_CPU_COMM_APP_ID_SAMPLING)
        {
            comm_ptr->sampling_buffer_ptr = &to_block_ptr[i];
            continue;
        } 
    }
    return true;
}

uint32_t getServoCpuCommMajorVersion(ServoCpuComm_t* comm_ptr)
{
    uint32_t major_version;
    getCommReg1MajorVersion(comm_ptr->comm_reg_ptr, &major_version);
    return major_version;
}

uint32_t getServoCpuCommMinorVersion(ServoCpuComm_t* comm_ptr)
{
    uint32_t minor_version;
    getCommReg1MinorVersion(comm_ptr->comm_reg_ptr, &minor_version);
    return minor_version;
}

void setServoCpuCommCtrlPdoSync(ServoCpuComm_t* comm_ptr, uint32_t index, uint32_t ctrl_pdo_sync)
{
    if(comm_ptr == NULL
        || comm_ptr->comm_reg_ptr == NULL)
    {
        return;
    }     
    setCommReg1CtrlPdoSync(comm_ptr->comm_reg_ptr, index, ctrl_pdo_sync);
}

void setServoCpuCommSamplingInterval(ServoCpuComm_t* comm_ptr, uint32_t sampling_interval)
{
    if(comm_ptr == NULL
        || comm_ptr->comm_reg_ptr == NULL)
    {
        return;
    }     
    setCommReg1SamplingInterval(comm_ptr->comm_reg_ptr, sampling_interval);
}

void setServoCpuCommSamplingMaxTimes(ServoCpuComm_t* comm_ptr, uint32_t sampling_max_times)
{
    if(comm_ptr == NULL
        || comm_ptr->comm_reg_ptr == NULL)
    {
        return;
    }     
    setCommReg1SamplingMaxTimes(comm_ptr->comm_reg_ptr, sampling_max_times);
}

void setServoCpuCommSamplingChannel(ServoCpuComm_t* comm_ptr, uint32_t channel_index, uint32_t channel_value)
{
    if(comm_ptr == NULL
        || comm_ptr->comm_reg_ptr == NULL)
    {
        return;
    }     
    setCommReg1SamplingChannel(comm_ptr->comm_reg_ptr, channel_index, channel_value);
}

void getServoCpuCommSamplingBuffer(ServoCpuComm_t* comm_ptr, uint8_t* data_ptr, int32_t* data_byte_size_ptr)
{
    getBufferType2(comm_ptr->sampling_buffer_ptr, data_ptr, data_byte_size_ptr);
}

ServoCpuComm_t* createServoCpuCommByServo(int32_t servo_id)
{
    ServoCpuComm_t* comm_ptr = (ServoCpuComm_t*)malloc(sizeof(ServoCpuComm_t));
    if(comm_ptr == NULL)
    {
        return NULL;
    }
     
    comm_ptr->from = servo_id;
    comm_ptr->to = -1;  // don't care
    comm_ptr->comm_reg_ptr = NULL;
    comm_ptr->sampling_buffer_ptr = NULL;
    comm_ptr->param_reg_ptr = NULL;
    return comm_ptr;
}

bool initServoCpuCommByServo(ServoCpuComm_t* comm_ptr,
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
            && from_block_ptr[i].application_id == SERVO_CPU_COMM_APP_ID_SAMPLING)
        {
            comm_ptr->sampling_buffer_ptr = &from_block_ptr[i];
            continue;
        }          
    }
     
    for(size_t i = 0; i < to_block_number; ++i)
    {
        if(to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].application_id == SERVO_CPU_COMM_APP_ID_COMM_REG)
        {
            comm_ptr->comm_reg_ptr = &to_block_ptr[i];
            continue;
        }                       
    }

    for(size_t i = 0; i < to_block_number; ++i)
    {
        if(to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].application_id == SERVO_CPU_COMM_APP_ID_PARAM_REG)
        {
            comm_ptr->param_reg_ptr = &to_block_ptr[i];
            continue;
        }                       
    }
    for(size_t i = 0; i < to_block_number; ++i)//20210917 xzc
    {
        if(to_block_ptr[i].to == comm_ptr->from
            && to_block_ptr[i].application_id == SERVO_CPU_COMM_APP_ID_TSD_REG)
        {
            comm_ptr->tsd_reg_pt = &to_block_ptr[i];
            continue;
        }                       
    } 
    if(comm_ptr->comm_reg_ptr == NULL
        || comm_ptr->sampling_buffer_ptr == NULL
        || comm_ptr->param_reg_ptr == NULL)     
    {
        return false;
    }
    else
    {
        return true;
    }

}

uint32_t getServoCpuCommSamplingInterval(ServoCpuComm_t* comm_ptr)
{
    uint32_t sampling_interval;
    getCommReg1SamplingInterval(comm_ptr->comm_reg_ptr, &sampling_interval);
    return sampling_interval;
}

uint32_t getServoCpuCommSamplingMaxTimes(ServoCpuComm_t* comm_ptr)
{
    uint32_t sampling_max_times;
    getCommReg1SamplingMaxTimes(comm_ptr->comm_reg_ptr, &sampling_max_times);
    return sampling_max_times;
}

uint32_t getServoCpuCommSamplingChannel(ServoCpuComm_t* comm_ptr, uint32_t channel_index)
{
    uint32_t channel_value;
    getCommReg1SamplingChannel(comm_ptr->comm_reg_ptr, channel_index, &channel_value);
    return channel_value;
}

void setServoCpuCommMajorVersion(ServoCpuComm_t* comm_ptr, uint32_t major_version)
{
    setCommReg1MajorVersion(comm_ptr->comm_reg_ptr, major_version);
}

void setServoCpuCommMinorVersion(ServoCpuComm_t* comm_ptr, uint32_t minor_version)
{
    setCommReg1MinorVersion(comm_ptr->comm_reg_ptr, minor_version);
}
                                    
uint32_t getServoCpuCommCtrlPdoSync(ServoCpuComm_t* comm_ptr, uint32_t index)
{        
    uint32_t ctrl_pdo_sync;
    getCommReg1CtrlPdoSync(comm_ptr->comm_reg_ptr, index, &ctrl_pdo_sync);
    return ctrl_pdo_sync;
}

void setServoCpuCommSamplingSync(ServoCpuComm_t* comm_ptr, uint32_t sampling_sync)
{
    setCommReg1SamplingSync(comm_ptr->comm_reg_ptr, sampling_sync);
}

uint32_t getServoCpuCommSamplingSync(ServoCpuComm_t* comm_ptr)
{
    uint32_t sampling_sync;
    getCommReg1SamplingSync(comm_ptr->comm_reg_ptr, &sampling_sync);
    return sampling_sync;
}

void setServoCpuCommSamplingCfg(ServoCpuComm_t* comm_ptr, uint32_t sampling_cfg)
{
    setCommReg1SamplingCfg(comm_ptr->comm_reg_ptr, sampling_cfg);
}

uint32_t getServoCpuCommSamplingCfg(ServoCpuComm_t* comm_ptr)
{
    uint32_t sampling_cfg;
    getCommReg1SamplingCfg(comm_ptr->comm_reg_ptr, &sampling_cfg);
    return sampling_cfg;
}

void setServoCpuCommControlMode(ServoCpuComm_t* comm_ptr, uint32_t control_mode)
{
    setCommReg1ControlMode(comm_ptr->comm_reg_ptr, control_mode);
}

uint32_t getServoCpuCommControlMode(ServoCpuComm_t* comm_ptr)
{
    uint32_t control_mode;
    getCommReg1ControlMode(comm_ptr->comm_reg_ptr, &control_mode);
    return control_mode;
}

bool setServoCpuCommForceControlUpdateFlag(ServoCpuComm_t* comm_ptr, uint32_t value)
{
    return setCommReg2UpdateFlag(comm_ptr->param_reg_ptr, value);
}
bool getServoCpuCommForceControlUpdateFlag(ServoCpuComm_t* comm_ptr, uint32_t* value_ptr)
{
    return getCommReg2UpdateFlag(comm_ptr->param_reg_ptr, value_ptr);
}
bool setServoCpuCommForceControlParameters(ServoCpuComm_t* comm_ptr, uint8_t* data_ptr, uint32_t data_byte_size)
{
    return setCommReg2Parameters(comm_ptr->param_reg_ptr, data_ptr, data_byte_size);
}
bool getServoCpuCommForceControlParameters(ServoCpuComm_t* comm_ptr, uint8_t* data_ptr, uint32_t* data_byte_size_ptr)
{
    return getCommReg2Parameters(comm_ptr->param_reg_ptr, data_ptr, data_byte_size_ptr);
}


bool setServoCpuCommTorqueSensorUpdateFlag(ServoCpuComm_t* comm_ptr, uint32_t value)
{
 return setCommReg3UpdateFlag(comm_ptr->tsd_reg_pt, value);
}
bool getServoCpuCommTorqueSensorUpdateFlag(ServoCpuComm_t* comm_ptr, uint32_t* value_ptr)
{
    return getCommReg3UpdateFlag(comm_ptr->tsd_reg_pt, value_ptr);
}
bool getServoCpuCommTorqueSensorData(ServoCpuComm_t* comm_ptr, uint8_t* data_ptr, uint32_t* data_byte_size_ptr)
{
    return getCommReg3Data(comm_ptr->tsd_reg_pt, data_ptr, data_byte_size_ptr);
}
void freeServoCpuComm(ServoCpuComm_t* comm_ptr)
{
    if(comm_ptr != NULL)
    {
        free(comm_ptr);
    }
}


