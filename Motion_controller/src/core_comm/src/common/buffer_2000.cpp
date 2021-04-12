#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/buffer_2000.h"
#else
#include "common/buffer_2000.h"
#endif

void initBuffer2000(CommBlockData_t* block_ptr)
{
    BufferCommData_t comm_data;
    comm_data.valid_data_byte_offset = 0;
    comm_data.max_data_byte_offset = block_ptr->param1;
    initBufferMemoryByFrom(block_ptr, &comm_data);
}

