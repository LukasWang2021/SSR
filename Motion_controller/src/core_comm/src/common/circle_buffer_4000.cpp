#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/circle_buffer_4000.h"
#else
#include "common/circle_buffer_4000.h"
#endif

void initCircleBuffer4000(CommBlockData_t* block_ptr)
{
    CircleBufferCommData_t comm_data;
    comm_data.head_byte_offset = 0;
    comm_data.tail_byte_offset = 0;
    comm_data.element_byte_size = block_ptr->param2;
    comm_data.max_data_byte_offset = block_ptr->param3;
    comm_data.min_data_byte_offset = block_ptr->param4;
    initCircleBufferMemoryByFrom(block_ptr, &comm_data);
}



