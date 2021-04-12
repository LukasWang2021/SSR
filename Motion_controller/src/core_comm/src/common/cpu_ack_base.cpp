#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/cpu_ack_base.h"
#include <string.h>
#else
#include "common/cpu_ack_base.h"
#include <string.h>
#endif

bool setCpuAck(CpuAckBlockData_t* block_ptr, CpuAckCommData_t* data_ptr)
{
    if(block_ptr->memory_ptr != NULL)
    {
        memcpy(block_ptr->memory_ptr, data_ptr, sizeof(CpuAckCommData_t));
        return true;
    }
    else
    {
        return false;
    }
}

bool getCpuAck(CpuAckBlockData_t* block_ptr, CpuAckCommData_t* data_ptr)
{
    if(block_ptr->memory_ptr != NULL)
    {

        memcpy(data_ptr, block_ptr->memory_ptr, sizeof(CpuAckCommData_t));
        return true;
    }
    else
    {
        return false;
    }
}

