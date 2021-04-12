#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/buffer_base.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#else
#include "common/buffer_base.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#endif

void initBufferMemoryByFrom(CommBlockData_t* block_ptr, BufferCommData_t* data_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    memcpy(block_ptr->memory_ptr, data_ptr, sizeof(BufferCommData_t));
    memset(block_ptr->memory_ptr + sizeof(BufferCommData_t), 0, data_ptr->max_data_byte_offset);    
}

bool setBufferType1(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t data_byte_size)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_byte_size > 0);
    
    if(data_byte_size > BUFFER_MAX_BYTE_OFFSET)
    {
        return false;
    }
    memcpy(BUFFER_DATA_PTR, data_ptr, data_byte_size);
    BUFFER_VALID_BYTE_OFFSET = data_byte_size;
    return true;
}

bool getBufferType1(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t* data_byte_size_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_byte_size_ptr != NULL);

    *data_byte_size_ptr = BUFFER_VALID_BYTE_OFFSET;
    memcpy(data_ptr, BUFFER_DATA_PTR, *data_byte_size_ptr);
    return true;
}

void resetBufferType2(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    BUFFER_VALID_BYTE_OFFSET = 0;
}

bool setBufferType2(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t data_byte_size)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_byte_size > 0);

    int32_t expect_valid_byte_offset = data_byte_size + BUFFER_VALID_BYTE_OFFSET;
    if(expect_valid_byte_offset > BUFFER_MAX_BYTE_OFFSET)
    {
        return false;
    }
    else
    {
        memcpy(BUFFER_DATA_PTR + BUFFER_VALID_BYTE_OFFSET, data_ptr, data_byte_size);
        BUFFER_VALID_BYTE_OFFSET = expect_valid_byte_offset;
        return true;
    }
}

bool getBufferType2(CommBlockData_t* block_ptr, uint8_t* data_ptr, int32_t* data_byte_size_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    assert(data_byte_size_ptr != NULL);

    *data_byte_size_ptr = BUFFER_VALID_BYTE_OFFSET;
    memcpy(data_ptr, BUFFER_DATA_PTR, *data_byte_size_ptr);
    return true;
}

