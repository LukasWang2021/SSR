#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/comm_reg_0.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#else
#include "common/comm_reg_0.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#endif

void initCommReg0(CommBlockData_t* block_ptr)
{
    CommRegAppData0_t comm_data;
    comm_data.state = 0;
    comm_data.error_flag = 0;
    memcpy(block_ptr->memory_ptr, &comm_data, sizeof(CommRegAppData0_t));
}

void setCommReg0State(CommBlockData_t* block_ptr, int32_t state)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG0_STATE_PTR, &state, sizeof(int32_t));
}

void getCommReg0State(CommBlockData_t* block_ptr, int32_t* state_ptr)
{
    assert(block_ptr != NULL);
    assert(state_ptr != NULL);
    memcpy(state_ptr, COMM_REG0_STATE_PTR, sizeof(int32_t));
}

void setCommReg0ErrorFlag(CommBlockData_t* block_ptr, int32_t error_flag)
{
    assert(block_ptr != NULL);
    memcpy(COMM_REG0_ERROR_FLAG_PTR, &error_flag, sizeof(int32_t));
}

void getCommReg0ErrorFlag(CommBlockData_t* block_ptr, int32_t* error_flag_ptr)
{
    assert(block_ptr != NULL);
    assert(error_flag_ptr != NULL);
    memcpy(error_flag_ptr, COMM_REG0_ERROR_FLAG_PTR, sizeof(int32_t));
}

