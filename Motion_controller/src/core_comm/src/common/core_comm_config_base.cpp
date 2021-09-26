#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_config_base.h"
#include "./core_protocal_inc/common_file_path.h"
#include "./core_protocal_inc/comm_reg_0.h"
#include "./core_protocal_inc/comm_reg_1.h"
#include "./core_protocal_inc/comm_reg_2.h"
#include "./core_protocal_inc/core_process_call_1000.h"
#include "./core_protocal_inc/buffer_2000.h"
#include "./core_protocal_inc/buffer_2001.h"
#include "./core_protocal_inc/buffer_2002.h"
#include "./core_protocal_inc/circle_buffer_3000.h"
#include "./core_protocal_inc/circle_buffer_4000.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#else
#include "common/core_comm_config_base.h"
#include "common_file_path.h"
#include "common/comm_reg_0.h"
#include "common/comm_reg_1.h"
#include "common/comm_reg_2.h"
#include "common/comm_reg_3.h"

#include "common/core_process_call_1000.h"
#include "common/buffer_2000.h"
#include "common/buffer_2001.h"
#include "common/buffer_2002.h"
#include "common/circle_buffer_3000.h"
#include "common/circle_buffer_3001.h"
#include "common/circle_buffer_3002.h"
#include "common/circle_buffer_4000.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#endif

void initCoreCommConfigMemeoryPtr(CoreCommConfig_t* core_comm_config_ptr, char* comm_ptr)
{
    assert(core_comm_config_ptr != NULL);
    assert(comm_ptr != NULL);
    assert(comm_ptr != NULL);

    for(size_t i = 0; i <core_comm_config_ptr->cpu_ack_block_num; ++i)
    {
        core_comm_config_ptr->cpu_ack_data[i].memory_ptr = comm_ptr + core_comm_config_ptr->cpu_ack_data[i].byte_offset;
    }
    
    for(size_t i = 0; i < core_comm_config_ptr->comm_block_num; ++i)
    {
        core_comm_config_ptr->comm_data[i].memory_ptr = comm_ptr + core_comm_config_ptr->comm_data[i].byte_offset;
    }
}

void initLocalChannel(CoreCommConfig_t* core_comm_config_ptr, int32_t cpu_id, 
                           CommBlockData_t** from_block_ptr_ptr, size_t* from_block_number_ptr,
                           CommBlockData_t** to_block_ptr_ptr, size_t* to_block_number_ptr)
{      
    *from_block_ptr_ptr = getCommBlockDataByFrom(core_comm_config_ptr, cpu_id, from_block_number_ptr);
    for(size_t i = 0; i < *from_block_number_ptr; ++i)
    {
        switch((*from_block_ptr_ptr)[i].application_id)
        {
            case 0: initCommReg0(&((*from_block_ptr_ptr)[i])); break;   // only called by slave side
            case 1: initCommReg1(&((*from_block_ptr_ptr)[i])); break;   // only called by master side
            case 2: initCommReg2(&((*from_block_ptr_ptr)[i])); break;
            case 3: initCommReg3(&((*from_block_ptr_ptr)[i])); break;
            case 1000: initCoreProcessCall1000(&((*from_block_ptr_ptr)[i])); break;
            case 2000: initBuffer2000(&((*from_block_ptr_ptr)[i])); break;
            case 2001: initBuffer2001(&((*from_block_ptr_ptr)[i])); break;
            case 2002: initBuffer2002(&((*from_block_ptr_ptr)[i])); break;
            case 3000: initCircleBuffer3000(&((*from_block_ptr_ptr)[i])); break;
            case 3001: initCircleBuffer3001(&((*from_block_ptr_ptr)[i])); break;
            case 3002: initCircleBuffer3002(&((*from_block_ptr_ptr)[i])); break;
            case 4000: initCircleBuffer4000(&((*from_block_ptr_ptr)[i])); break;
            default:
                ;
        }
    }    
    *to_block_ptr_ptr = getCommBlockDataByTo(core_comm_config_ptr, cpu_id, to_block_number_ptr);
}

BoardcastBlockData_t* getBoardcastBlockDataByFrom(CoreCommConfig_t* config_ptr, int32_t from_id, size_t* number_ptr)
{
    BoardcastBlockData_t* data_ptr = NULL;
    *number_ptr = 0;
    for(size_t i = 0; i < config_ptr->boardcast_block_num; ++i)
    {
        if(config_ptr->boardcast_data.from == from_id)
        {
            ++(*number_ptr);
            data_ptr = (BoardcastBlockData_t*)malloc(sizeof(BoardcastBlockData_t));
            memcpy(data_ptr, &config_ptr->boardcast_data, sizeof(BoardcastBlockData_t));            
        }
    }
    return data_ptr;
}

CpuAckBlockData_t* getCpuAckBlockDataByFrom(CoreCommConfig_t* config_ptr, int32_t from_id, size_t* number_ptr)
{
    CpuAckBlockData_t* data_ptr = NULL;
    *number_ptr = 0;
    for(size_t i = 0; i < config_ptr->cpu_ack_block_num; ++i)
    {
        if(config_ptr->cpu_ack_data[i].from == from_id)
        {
            ++(*number_ptr);
            CpuAckBlockData_t* temp_ptr = (CpuAckBlockData_t*)realloc(data_ptr, sizeof(CpuAckBlockData_t) * (*number_ptr));
            if (temp_ptr != NULL)
            {
                data_ptr = temp_ptr;
                memcpy(&data_ptr[(*number_ptr) - 1], &config_ptr->cpu_ack_data[i], sizeof(CpuAckBlockData_t));
            }
        }
    }
    return data_ptr;
}

CpuAckBlockData_t* getCpuAckBlockDataByTo(CoreCommConfig_t* config_ptr, int32_t to_id, size_t* number_ptr)
{
    CpuAckBlockData_t* data_ptr = NULL;
    *number_ptr = 0;
    for(size_t i = 0; i < config_ptr->cpu_ack_block_num; ++i)
    {
        if(config_ptr->cpu_ack_data[i].to == to_id)
        {
            ++(*number_ptr);
            CpuAckBlockData_t* temp_ptr = (CpuAckBlockData_t*)realloc(data_ptr, sizeof(CpuAckBlockData_t) * (*number_ptr));
            if (temp_ptr != NULL)
            {
                data_ptr = temp_ptr;
                memcpy(&data_ptr[(*number_ptr) - 1], &config_ptr->cpu_ack_data[i], sizeof(CpuAckBlockData_t));
            }
        }
    }
    return data_ptr;
}

CommBlockData_t* getCommBlockDataByFrom(CoreCommConfig_t* config_ptr, int32_t from_id, size_t* number_ptr)
{
    CommBlockData_t* data_ptr = NULL;
    *number_ptr = 0;
    for(size_t i = 0; i < config_ptr->comm_block_num; ++i)
    {
        if(config_ptr->comm_data[i].from == from_id)
        {
            ++(*number_ptr);
            CommBlockData_t* temp_ptr = (CommBlockData_t*)realloc(data_ptr, sizeof(CommBlockData_t) * (*number_ptr));
            if (temp_ptr != NULL)
            {
                data_ptr = temp_ptr;
                memcpy(&data_ptr[(*number_ptr) - 1], &config_ptr->comm_data[i], sizeof(CommBlockData_t));
            }
        }
    }
    return data_ptr;
}

CommBlockData_t* getCommBlockDataByTo(CoreCommConfig_t* config_ptr, int32_t to_id, size_t* number_ptr)
{
    CommBlockData_t* data_ptr = NULL;
    *number_ptr = 0;
    for(size_t i = 0; i < config_ptr->comm_block_num; ++i)
    {
        if(config_ptr->comm_data[i].to == to_id)
        {
            ++(*number_ptr);
            CommBlockData_t* temp_ptr = (CommBlockData_t*)realloc(data_ptr, sizeof(CommBlockData_t) * (*number_ptr));
            if (temp_ptr != NULL)
            {
                data_ptr = temp_ptr; 
                memcpy(&data_ptr[(*number_ptr) - 1], &config_ptr->comm_data[i], sizeof(CommBlockData_t));
            }
        }
    }
    return data_ptr;
}


