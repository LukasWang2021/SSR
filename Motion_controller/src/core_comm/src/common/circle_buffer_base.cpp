#ifdef COMPILE_IN_BARE
#include "./core_protocal_inc/circle_buffer_base.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#else
#include "common/circle_buffer_base.h"
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#endif

void initCircleBufferMemoryByFrom(CommBlockData_t* block_ptr, CircleBufferCommData_t* data_ptr)
{
    assert(block_ptr != NULL);
    assert(data_ptr != NULL);
    memcpy(block_ptr->memory_ptr, data_ptr, sizeof(CircleBufferCommData_t));
    memset(block_ptr->memory_ptr + sizeof(CircleBufferCommData_t), 0, data_ptr->max_data_byte_offset);
}

int32_t getOccupiedByteSize(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    int32_t current_head_byte_offset = CIRCLE_BUFFER_HEAD_BYTE_OFFSET;
    int32_t current_tail_byte_offset = CIRCLE_BUFFER_TAIL_BYTE_OFFSET;
    int32_t delta_byte_offset = current_tail_byte_offset - current_head_byte_offset;
    if(current_tail_byte_offset < current_head_byte_offset)
    {
        delta_byte_offset += CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET;
    }
    return delta_byte_offset;
}

int32_t getOccupiedElementNumber(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    int32_t occupided_byte_size = getOccupiedByteSize(block_ptr);
    return (occupided_byte_size / CIRCLE_BUFFER_ELEMENT_BYTE_SIZE);
}

int32_t getSpareByteSize(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    int32_t occupided_byte_size = getOccupiedByteSize(block_ptr);
    return (CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET - CIRCLE_BUFFER_ELEMENT_BYTE_SIZE - occupided_byte_size);
}

int32_t getSpareElementNumber(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    int32_t spare_byte_size = getSpareByteSize(block_ptr);
    return (spare_byte_size / CIRCLE_BUFFER_ELEMENT_BYTE_SIZE);
}

bool pushCircleBufferType1(CommBlockData_t* block_ptr, uint8_t* domain_ptr, int32_t element_number)
{
    assert(block_ptr != NULL);
    assert(domain_ptr != NULL);
    assert(element_number >= 0);

    if(element_number == 0)
    {
        return true;
    }

    int32_t current_head_byte_offset = CIRCLE_BUFFER_HEAD_BYTE_OFFSET;
    int32_t current_tail_byte_offset = CIRCLE_BUFFER_TAIL_BYTE_OFFSET;
    int32_t total_element_byte_size = CIRCLE_BUFFER_ELEMENT_BYTE_SIZE * element_number;
    int32_t delta_byte_offset = current_tail_byte_offset - current_head_byte_offset;
    if(current_tail_byte_offset < current_head_byte_offset)
    {
        delta_byte_offset += CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET;
    }

    if((delta_byte_offset + total_element_byte_size) >= CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET)
    {
        return false;
    }

    int32_t new_tail_byte_offset = current_tail_byte_offset + total_element_byte_size;
    if(new_tail_byte_offset >= CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET)
    {
        int32_t upper_half_byte_size = CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET - current_tail_byte_offset;
        int32_t lower_half_byte_size = total_element_byte_size - upper_half_byte_size;
        memcpy(CIRCLE_BUFFER_DATA_PTR + current_tail_byte_offset, domain_ptr, upper_half_byte_size);        
        memcpy(CIRCLE_BUFFER_DATA_PTR, domain_ptr + upper_half_byte_size, lower_half_byte_size);
        new_tail_byte_offset = new_tail_byte_offset - CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET;
    }
    else
    {
        memcpy(CIRCLE_BUFFER_DATA_PTR + current_tail_byte_offset, domain_ptr, total_element_byte_size);
    }
    CIRCLE_BUFFER_TAIL_BYTE_OFFSET = new_tail_byte_offset;
    return true;
}

bool pullCircleBufferType1(CommBlockData_t* block_ptr, uint8_t* domain_ptr, int32_t element_number)
{
    assert(block_ptr != NULL);
    assert(domain_ptr != NULL);
    assert(element_number > 0);

    int32_t current_head_byte_offset = CIRCLE_BUFFER_HEAD_BYTE_OFFSET;
    int32_t current_tail_byte_offset = CIRCLE_BUFFER_TAIL_BYTE_OFFSET;
    int32_t total_element_byte_size = CIRCLE_BUFFER_ELEMENT_BYTE_SIZE * element_number;
    int32_t delta_byte_offset = current_tail_byte_offset - current_head_byte_offset;
    if(current_tail_byte_offset < current_head_byte_offset)
    {
        delta_byte_offset += CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET;
    }

    if(delta_byte_offset < total_element_byte_size)
    {
        return false;
    }

    int32_t new_head_byte_offset = current_head_byte_offset + total_element_byte_size;
    if(new_head_byte_offset >= CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET) // may be wrong
    {
        int32_t upper_half_byte_size = CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET - current_head_byte_offset;
        int32_t lower_half_byte_size = total_element_byte_size - upper_half_byte_size;
        memcpy(domain_ptr, CIRCLE_BUFFER_DATA_PTR + current_head_byte_offset, upper_half_byte_size);        
        memcpy(domain_ptr + upper_half_byte_size, CIRCLE_BUFFER_DATA_PTR, lower_half_byte_size);
        new_head_byte_offset = new_head_byte_offset - CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET;
    }
    else
    {
        memcpy(domain_ptr, CIRCLE_BUFFER_DATA_PTR + current_head_byte_offset, total_element_byte_size);
    }
    CIRCLE_BUFFER_HEAD_BYTE_OFFSET = new_head_byte_offset;
    return true;
}

int32_t getPreviousHeadByteOffset(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    int32_t previous_head = CIRCLE_BUFFER_HEAD_BYTE_OFFSET - CIRCLE_BUFFER_ELEMENT_BYTE_SIZE;
    if(previous_head < 0)
    {
        previous_head = CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET - CIRCLE_BUFFER_ELEMENT_BYTE_SIZE;
    }
    return previous_head;
}

int32_t getPreviousTailByteOffset(CommBlockData_t* block_ptr)
{
    assert(block_ptr != NULL);
    int32_t previous_tail = CIRCLE_BUFFER_TAIL_BYTE_OFFSET - CIRCLE_BUFFER_ELEMENT_BYTE_SIZE;
    if(previous_tail < 0)
    {
        previous_tail = CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET - CIRCLE_BUFFER_ELEMENT_BYTE_SIZE;
    }
    return previous_tail;
}

bool pushCircleBufferType2(CommBlockData_t* block_ptr, uint8_t* domain_ptr)
{
    assert(block_ptr != NULL);
    assert(domain_ptr != NULL);

    int32_t current_head_byte_offset = CIRCLE_BUFFER_HEAD_BYTE_OFFSET;
    int32_t current_tail_byte_offset = CIRCLE_BUFFER_TAIL_BYTE_OFFSET;
    int32_t element_byte_size = CIRCLE_BUFFER_ELEMENT_BYTE_SIZE;
    int32_t new_tail_byte_offset = current_tail_byte_offset + element_byte_size;
    int32_t new_head_byte_offset = current_head_byte_offset + element_byte_size;
    
    if(new_tail_byte_offset >= CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET)
    {
        new_tail_byte_offset = 0;
    }
    if(new_head_byte_offset >= CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET)
    {
        new_head_byte_offset = 0;
    }
    memcpy(CIRCLE_BUFFER_DATA_PTR + new_tail_byte_offset, domain_ptr, element_byte_size);
    CIRCLE_BUFFER_TAIL_BYTE_OFFSET = new_tail_byte_offset;
    CIRCLE_BUFFER_HEAD_BYTE_OFFSET = new_head_byte_offset;    
    return true;
}

bool pullCircleBufferType2ByHead(CommBlockData_t* block_ptr, uint8_t* domain_ptr, uint32_t* time_stamp_ptr)
{
    assert(block_ptr != NULL);
    assert(domain_ptr != NULL);
    assert(time_stamp_ptr != NULL);

    memcpy(domain_ptr, CIRCLE_BUFFER_DATA_PTR + CIRCLE_BUFFER_HEAD_BYTE_OFFSET, CIRCLE_BUFFER_ELEMENT_BYTE_SIZE);
    *time_stamp_ptr = *((uint32_t*)domain_ptr);  // time stamp element must be the first app data element for circle buffer type2
    return true;
}

bool pullCircleBufferType2ByTimeStamp(CommBlockData_t* block_ptr, uint8_t* domain_ptr, uint32_t time_stamp)
{
    assert(block_ptr != NULL);
    assert(domain_ptr != NULL);

    // get the time stamp of head frame
    uint32_t head_time_stamp = *((uint32_t*)(CIRCLE_BUFFER_DATA_PTR + CIRCLE_BUFFER_HEAD_BYTE_OFFSET));
    int32_t delta_time_stamp = head_time_stamp - time_stamp;
    if(delta_time_stamp < 0)
    {
        return false;
    }

    int32_t expect_byte_offset = CIRCLE_BUFFER_HEAD_BYTE_OFFSET - CIRCLE_BUFFER_ELEMENT_BYTE_SIZE * delta_time_stamp;
    if(expect_byte_offset < 0)
    {
        expect_byte_offset += CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET;
    }
    memcpy(domain_ptr, CIRCLE_BUFFER_DATA_PTR + expect_byte_offset, CIRCLE_BUFFER_ELEMENT_BYTE_SIZE);
    return true;
}


