#include <stdlib.h>
#include <string.h>

#include "ring_buffer.h"

int32_t ring_buffer_init(RingBuffer_t* rbuff)
{
    rbuff->begin_offset = (int8_t*)malloc(rbuff->element_count * rbuff->element_size);
    if (rbuff->begin_offset == NULL)
        return -1;
    rbuff->end_ofset = rbuff->begin_offset + (rbuff->element_count) * (rbuff->element_size);
    rbuff->read_offset = rbuff->begin_offset;
    rbuff->write_offset = rbuff->begin_offset;

    return 0;
}

int32_t ring_buffer_push(RingBuffer_t* rbuff, void* data, int32_t num)
{
    if (rbuff == NULL || data == NULL) return -1;

    int32_t sparing = ring_buffer_sparing(rbuff);
    int32_t count = sparing < num ? sparing : num;
    int8_t* tmp_data = (int8_t*)data;

    for (int i = 0; i < count; ++i)
    {
        if (rbuff->write_offset >= rbuff->end_ofset)
        {
            rbuff->write_offset = rbuff->begin_offset;
        }
        memcpy(rbuff->write_offset, tmp_data, rbuff->element_size);
        tmp_data += rbuff->element_size;
        rbuff->write_offset += rbuff->element_size;
    }

    return count;
}

int32_t ring_buffer_pull(RingBuffer_t* rbuff, void* data, int32_t num)
{
    if (rbuff == NULL || data == NULL) return -1;

    int32_t occupied = ring_buffer_occupied(rbuff);
    int32_t count = occupied < num ? occupied : num;
    int8_t* tmp_data = (int8_t *)data;
    for (int i = 0; i < count; ++i)
    {
        if (rbuff->read_offset >= rbuff->end_ofset)
        {
            rbuff->read_offset = rbuff->begin_offset;
        }
        memcpy(tmp_data, rbuff->read_offset, rbuff->element_size);
        tmp_data += rbuff->element_size;
        rbuff->read_offset += rbuff->element_size;
    }
    return count;

}

int32_t ring_buffer_occupied(RingBuffer_t* rbuff)
{
    int32_t occupied = (int32_t)((rbuff->write_offset - rbuff->read_offset) / rbuff->element_size);
    return occupied < 0 ? -occupied : occupied;
}

int32_t ring_buffer_sparing(RingBuffer_t* rbuff)
{
    return (int32_t)rbuff->element_count - ring_buffer_occupied(rbuff);
}

int32_t ring_buffer_full(RingBuffer_t* rbuff)
{
    if (rbuff->write_offset - rbuff->read_offset == rbuff->element_count)
        return 1;

    return 0;
}

int32_t ring_buffer_empty(RingBuffer_t* rbuff)
{
    if (rbuff->read_offset == rbuff->write_offset)
        return 1;
    return 0;
}