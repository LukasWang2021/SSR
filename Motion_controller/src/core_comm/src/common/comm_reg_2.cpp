#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/comm_reg_2.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#else
#include "common/comm_reg_2.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#endif

void initCommReg2(CommBlockData_t* block_ptr)
{
    CommRegAppData2_t comm_data;
    comm_data.update_flag = 0;
    for(uint32_t i = 0; i < COMM_REG2_PARAMETER_NUMBER; ++i)
    {
        comm_data.force_control.parameter[i] = 0;
    }
    memcpy(block_ptr->memory_ptr, &comm_data, sizeof(CommRegAppData2_t));
}

bool setCommReg2UpdateFlag(CommBlockData_t* block_ptr, uint32_t value)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG2_UPDATE_FLAG_PTR, &value, sizeof(uint32_t));
    return true;
}

bool getCommReg2UpdateFlag(CommBlockData_t* block_ptr, uint32_t* value_ptr)
{
    assert(block_ptr != NULL);
    memcpy(value_ptr, COMM_REG2_UPDATE_FLAG_PTR, sizeof(uint32_t));
    return true;
}

bool setCommReg2Parameters(CommBlockData_t* block_ptr, uint8_t* data_ptr, uint32_t data_byte_size)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_byte_size > 0);
    memcpy(COMM_REG2_PARAMETERS_PTR, data_ptr, data_byte_size);
    return true;
}

bool getCommReg2Parameters(CommBlockData_t* block_ptr, uint8_t* data_ptr, uint32_t* data_byte_size_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_byte_size_ptr != NULL);

    *data_byte_size_ptr = sizeof(CommRegForceControlParam_t);
    memcpy(data_ptr, COMM_REG2_PARAMETERS_PTR, *data_byte_size_ptr);
    return true;
}
