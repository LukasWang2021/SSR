#ifndef CIRCLE_BUFFER_DATATYPE_H
#define CIRCLE_BUFFER_DATATYPE_H

/**
 * @file circle_buffer_datatype.h
 * @brief The file includes the definition of the control section of a circle buffer channel.
 * @author zhengyu.shen
 */

#include <stdint.h>

#define CIRCLE_BUFFER_HEAD_BYTE_OFFSET_PTR      ((int32_t*)block_ptr->memory_ptr)           /**< The address to store the current head byte offset.*/
#define CIRCLE_BUFFER_TAIL_BYTE_OFFSET_PTR      ((int32_t*)(block_ptr->memory_ptr + 4))     /**< The address to store the current tail byte offset.*/
#define CIRCLE_BUFFER_ELEMENT_BYTE_SIZE_PTR     ((int32_t*)(block_ptr->memory_ptr + 8))     /**< The address to store the byte size of element.*/
#define CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET_PTR  ((int32_t*)(block_ptr->memory_ptr + 12))    /**< The address to store the maximum byte size of data section.*/
#define CIRCLE_BUFFER_MIN_DATA_BYTE_OFFSET_PTR  ((int32_t*)(block_ptr->memory_ptr + 16))    /**< The address to store the minimum byte size of data section.*/
#define CIRCLE_BUFFER_DATA_PTR                  ((int8_t*)(block_ptr->memory_ptr + 20))     /**< The start address of data section.*/

#define CIRCLE_BUFFER_HEAD_BYTE_OFFSET      (*CIRCLE_BUFFER_HEAD_BYTE_OFFSET_PTR)       /**< The current head byte offset.*/
#define CIRCLE_BUFFER_TAIL_BYTE_OFFSET      (*CIRCLE_BUFFER_TAIL_BYTE_OFFSET_PTR)       /**< The current tail byte offset.*/
#define CIRCLE_BUFFER_ELEMENT_BYTE_SIZE     (*CIRCLE_BUFFER_ELEMENT_BYTE_SIZE_PTR)      /**< The byte size of element.*/
#define CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET  (*CIRCLE_BUFFER_MAX_DATA_BYTE_OFFSET_PTR)   /**< The maximum byte size of data section. It must be integer multiple of CIRCLE_BUFFER_ELEMENT_BYTE_SIZE.*/
#define CIRCLE_BUFFER_MIN_DATA_BYTE_OFFSET  (*CIRCLE_BUFFER_MIN_DATA_BYTE_OFFSET_PTR)   /**< The minimum byte size of data section. It must be integer multiple of CIRCLE_BUFFER_ELEMENT_BYTE_SIZE.*/

/**
 * @brief Defines the data structure for the control section of a circle buffer channel.
 */
typedef struct
{
    int32_t head_byte_offset;       /**< The current head byte offset.*/
    int32_t tail_byte_offset;       /**< The current tail byte offset.*/
    int32_t element_byte_size;      /**< The byte size of element.*/
    int32_t max_data_byte_offset;   /**< The maximum byte size of data section. It must be integer multiple of CIRCLE_BUFFER_ELEMENT_BYTE_SIZE.*/
    int32_t min_data_byte_offset;   /**< The minimum byte size of data section. It must be integer multiple of CIRCLE_BUFFER_ELEMENT_BYTE_SIZE.*/
}CircleBufferCommData_t;

#endif
