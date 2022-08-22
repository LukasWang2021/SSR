#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include "stdint.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int8_t* begin_offset;
    int8_t* end_ofset;
    int8_t *read_offset;
    int8_t *write_offset;
    uint64_t element_size;
    uint64_t element_count;
}RingBuffer_t;

extern int32_t ring_buffer_init(RingBuffer_t* rbuff);

extern int32_t ring_buffer_push(RingBuffer_t* rbuff, void* data, int32_t num);

extern int32_t ring_buffer_pull(RingBuffer_t* rbuff, void* data, int32_t num);

extern int32_t ring_buffer_occupied(RingBuffer_t* rbuff);

extern int32_t ring_buffer_sparing(RingBuffer_t* rbuff);

extern int32_t ring_buffer_full(RingBuffer_t* rbuff);

extern int32_t ring_buffer_empty(RingBuffer_t* rbuff);

#ifdef __cplusplus
}
#endif

#endif
