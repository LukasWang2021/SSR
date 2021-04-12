#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/core_process_call_1000.h"
#else
#include "common/core_process_call_1000.h"
#endif

void initCoreProcessCall1000(CommBlockData_t* block_ptr)
{
    CoreProcessCallCommData_t comm_data;
    comm_data.request_toggle = 0;
    comm_data.response_toggle = 0;
    comm_data.valid_data_byte_offset = 0;
    comm_data.max_data_byte_offset = block_ptr->param2;
    comm_data.timeout_count = block_ptr->param3;
    initCoreProcessCallMemoryByFrom(block_ptr, &comm_data);
}

