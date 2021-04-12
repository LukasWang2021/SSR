#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_comm_bare.h"
#include "./core_protocal_inc/boardcast_base.h"
#include "./core_protocal_inc/cpu_ack_base.h"
#include "./core_protocal_inc/core_comm_config_base.h"
#include <string.h>
#include <stdlib.h>
#else
#include "bare/core_comm_bare.h"
#include "common/boardcast_base.h"
#include "common/cpu_ack_base.h"
#include "common/core_comm_config_base.h"
#include <string.h>
#include <stdlib.h>
#endif

int32_t g_cpu_id = 2;
uint32_t g_boardcast_device_addr = CORE_COMM_BASE_ADDRESS;
uint32_t g_config_device_addr = CORE_COMM_BASE_ADDRESS + CORE_COMM_BOARDCAST_BYTE_SIZE;
uint32_t g_comm_device_addr = 0;

CoreCommConfig_t g_core_comm_config;
bool g_core_comm_config_ready = false;
CommBlockData_t* g_from_block_ptr = NULL;
CommBlockData_t* g_to_block_ptr = NULL;
size_t g_from_block_number = 0;
size_t g_to_block_number = 0;


/*bool copyMemeoryToConfigData()
{
    return false;
}*/

bool isMasterBooted()
{
    BoardcastCommData_t data;
    if(getBoardcast((char*)g_boardcast_device_addr, &data)
        && data.data == 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

ErrorCode bootAsSlave()
{
    // copy config data to local memory
    memcpy(&g_core_comm_config, (void*)g_config_device_addr, sizeof(CoreCommConfig_t));
           
    // do core communication initialization here
    g_comm_device_addr = g_core_comm_config.base_address; 
    initCoreCommConfigMemeoryPtr(&g_core_comm_config, (char*)g_comm_device_addr);

    // init local channel
    initLocalChannel(&g_core_comm_config, g_cpu_id, 
                     &g_from_block_ptr, &g_from_block_number,
                     &g_to_block_ptr, &g_to_block_number);
    
    // notify master the slave is configured
    size_t number;  
    CpuAckBlockData_t* cpu_ack_block_data_ptr = getCpuAckBlockDataByFrom(&g_core_comm_config, g_cpu_id, &number);
    if(number != 1)
    {
        if(cpu_ack_block_data_ptr != NULL)
        {
            free(cpu_ack_block_data_ptr);
        }
        return CORE_COMM_SLAVE_EVENT_CONFIG_INVALID;
    }    
    CpuAckCommData_t data;
    data.data = 1;
    setCpuAck(&cpu_ack_block_data_ptr[0], &data);

    if(cpu_ack_block_data_ptr != NULL)
    {
        free(cpu_ack_block_data_ptr);
    }
    return SUCCESS;
}

