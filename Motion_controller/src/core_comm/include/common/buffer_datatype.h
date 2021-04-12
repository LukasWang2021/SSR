#ifndef BUFFER_DATATYPE_H
#define BUFFER_DATATYPE_H

/**
 * @file buffer_datatype.h
 * @brief The file includes the definition of the control section of a buffer channel.
 * @author zhengyu.shen
 */

#include <stdint.h>

#define BUFFER_VALID_BYTE_OFFSET_PTR    ((int32_t*)block_ptr->memory_ptr)       /**< The address to store the currently valid byte size of the data section.*/
#define BUFFER_MAX_BYTE_OFFSET_PTR      ((int32_t*)(block_ptr->memory_ptr + 4)) /**< The address to store the maximum byte size of the data section.*/
#define BUFFER_DATA_PTR                 ((int8_t*)(block_ptr->memory_ptr + 8))  /**< The start address of data section.*/

#define BUFFER_VALID_BYTE_OFFSET      (*BUFFER_VALID_BYTE_OFFSET_PTR)   /**< The currently valid byte size of the data section.*/
#define BUFFER_MAX_BYTE_OFFSET      (*BUFFER_MAX_BYTE_OFFSET_PTR)       /**< The maximum byte size of the data section.*/

/**
 * @brief Defines the data structure for the control section of a buffer channel.
 */
typedef struct
{
    int32_t valid_data_byte_offset; /**< The currently valid byte size of the data section.*/
    int32_t max_data_byte_offset;   /**< The maximum byte size of the data section.*/
}BufferCommData_t;

#endif

