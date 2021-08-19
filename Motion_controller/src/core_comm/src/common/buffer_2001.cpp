#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/buffer_2001.h"
#else
#include "common/buffer_2001.h"
#endif

void initBuffer2001(CommBlockData_t* block_ptr)
{
    BufferCommData_t comm_data;
    comm_data.valid_data_byte_offset = 0;
    comm_data.max_data_byte_offset = block_ptr->param2;
    initBufferMemoryByFrom(block_ptr, &comm_data);
}
