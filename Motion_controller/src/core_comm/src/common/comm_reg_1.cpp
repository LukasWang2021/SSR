#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/comm_reg_1.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#else
#include "common/comm_reg_1.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#endif


void initCommReg1(CommBlockData_t* block_ptr)
{
    CommRegAppData1_t comm_data;
    for(uint32_t i = 0; i < COMM_REG1_CTRL_PDO_SYNC_NUMBER; ++i)
    {
        comm_data.ctrl_pdo_sync[i] = 0;
    }
    comm_data.sampling_sync = 0;
    comm_data.sampling_cfg = 0;
    comm_data.sampling_interval = 0;
    comm_data.sampling_max_times = 0;
    for(uint32_t i = 0; i < COMM_REG1_SAMPLING_CHANNEL_NUMBER; ++i)
    {
        comm_data.sampling_channel[i] = 0;
    }
    // major_version & minor_version are inited by servo core
    memcpy(block_ptr->memory_ptr + 8, ((uint8_t*)&comm_data) + 8, sizeof(CommRegAppData1_t) - 8);
}

void setCommReg1MajorVersion(CommBlockData_t* block_ptr, uint32_t major_version)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG1_MAJOR_VERSION_PTR, &major_version, sizeof(uint32_t));
}

void getCommReg1MajorVersion(CommBlockData_t* block_ptr, uint32_t* major_version_ptr)
{
    assert(block_ptr != NULL);
    assert(major_version_ptr != NULL);
    memcpy(major_version_ptr, COMM_REG1_MAJOR_VERSION_PTR, sizeof(uint32_t));
}

void setCommReg1MinorVersion(CommBlockData_t* block_ptr, uint32_t minor_version)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG1_MINOR_VERSION_PTR, &minor_version, sizeof(uint32_t));
}

void getCommReg1MinorVersion(CommBlockData_t* block_ptr, uint32_t* minor_version_ptr)
{
    assert(block_ptr != NULL);
    assert(minor_version_ptr != NULL);
    memcpy(minor_version_ptr, COMM_REG1_MINOR_VERSION_PTR, sizeof(uint32_t));
}

void setCommReg1CtrlPdoSync(CommBlockData_t* block_ptr, uint32_t index, uint32_t ctrl_pdo_sync)
{
    assert(block_ptr != NULL);
    assert(index < COMM_REG1_CTRL_PDO_SYNC_NUMBER);
    memcpy(COMM_REG1_CTRL_PDO_SYNC_PTR + (index<<2), &ctrl_pdo_sync, sizeof(uint32_t));
}

void getCommReg1CtrlPdoSync(CommBlockData_t* block_ptr, uint32_t index, uint32_t* ctrl_pdo_sync_ptr)
{
    assert(block_ptr != NULL);
    assert(index < COMM_REG1_CTRL_PDO_SYNC_NUMBER);
    assert(ctrl_pdo_sync_ptr != NULL);
    memcpy(ctrl_pdo_sync_ptr, COMM_REG1_CTRL_PDO_SYNC_PTR + (index<<2), sizeof(uint32_t));
}

void setCommReg1SamplingSync(CommBlockData_t* block_ptr, uint32_t sampling_sync)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG1_SAMPLING_SYNC_PTR, &sampling_sync, sizeof(uint32_t));
}

void getCommReg1SamplingSync(CommBlockData_t* block_ptr, uint32_t* sampling_sync_ptr)
{
    assert(block_ptr != NULL);
    assert(sampling_sync_ptr != NULL);
    memcpy(sampling_sync_ptr, COMM_REG1_SAMPLING_SYNC_PTR, sizeof(uint32_t));
}

void setCommReg1SamplingCfg(CommBlockData_t* block_ptr, uint32_t sampling_cfg)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG1_SAMPLING_CFG_PTR, &sampling_cfg, sizeof(uint32_t));
}

void getCommReg1SamplingCfg(CommBlockData_t* block_ptr, uint32_t* sampling_cfg_ptr)
{
    assert(block_ptr != NULL);
    assert(sampling_cfg_ptr != NULL);
    memcpy(sampling_cfg_ptr, COMM_REG1_SAMPLING_CFG_PTR, sizeof(uint32_t));
}

void setCommReg1SamplingInterval(CommBlockData_t* block_ptr, uint32_t sampling_interval)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG1_SAMPLING_INTERVAL_PTR, &sampling_interval, sizeof(uint32_t));
}

void getCommReg1SamplingInterval(CommBlockData_t* block_ptr, uint32_t* sampling_interval_ptr)
{
    assert(block_ptr != NULL);
    assert(sampling_interval_ptr != NULL);
    memcpy(sampling_interval_ptr, COMM_REG1_SAMPLING_INTERVAL_PTR, sizeof(uint32_t));
}

void setCommReg1SamplingMaxTimes(CommBlockData_t* block_ptr, uint32_t sampling_max_times)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG1_SAMPLING_MAX_TIMES_PTR, &sampling_max_times, sizeof(uint32_t));
}

void getCommReg1SamplingMaxTimes(CommBlockData_t* block_ptr, uint32_t* sampling_max_times_ptr)
{
    assert(block_ptr != NULL);
    assert(sampling_max_times_ptr != NULL);
    memcpy(sampling_max_times_ptr, COMM_REG1_SAMPLING_MAX_TIMES_PTR, sizeof(uint32_t));
}

void setCommReg1SamplingChannel(CommBlockData_t* block_ptr, uint32_t channel_index, uint32_t channel_value)
{
    assert(block_ptr != NULL);
    assert(channel_index < COMM_REG1_SAMPLING_CHANNEL_NUMBER);
    memcpy(COMM_REG1_SAMPLING_CHANNEL_PTR + (channel_index<<2), &channel_value, sizeof(uint32_t));
}

void getCommReg1SamplingChannel(CommBlockData_t* block_ptr, uint32_t channel_index, uint32_t* channel_value_ptr)
{
    assert(block_ptr != NULL);
    assert(channel_index < COMM_REG1_SAMPLING_CHANNEL_NUMBER);
    assert(channel_value_ptr != NULL);
    memcpy(channel_value_ptr, COMM_REG1_SAMPLING_CHANNEL_PTR + (channel_index<<2), sizeof(uint32_t));
}

