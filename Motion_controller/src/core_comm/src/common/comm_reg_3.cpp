#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/comm_reg_2.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#else
#include "common/comm_reg_3.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#endif

void initCommReg3(CommBlockData_t* block_ptr)
{
    CommRegAppData3_t comm_data;
    comm_data.update_flag = 0;
    for(uint32_t i = 0; i < COMM_REG3_DATA_NUMBER; ++i)
    {
        comm_data.F_torque.data[i] = 0;
    }
    memcpy(block_ptr->memory_ptr, &comm_data, sizeof(CommRegAppData3_t));
}

bool setCommReg3UpdateFlag(CommBlockData_t* block_ptr, uint32_t value)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG3_UPDATE_FLAG_PTR, &value, sizeof(uint32_t)); //COMM_REG3_UPDATE_FLAG_PTR   <===>  ((int8_t*)(block_ptr->memory_ptr))
    return true;
}

bool getCommReg3UpdateFlag(CommBlockData_t* block_ptr, uint32_t* value_ptr)
{
    assert(block_ptr != NULL);
    memcpy(value_ptr, COMM_REG3_UPDATE_FLAG_PTR, sizeof(uint32_t));
    return true;
}

bool getCommReg3Data(CommBlockData_t* block_ptr, uint8_t* data_ptr, uint32_t* data_byte_size_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_byte_size_ptr != NULL);
    *data_byte_size_ptr = sizeof(CommRegTorqueData_t);
    memcpy(data_ptr, COMM_REG3_DATA_PTR, *data_byte_size_ptr);
    return true;
}








