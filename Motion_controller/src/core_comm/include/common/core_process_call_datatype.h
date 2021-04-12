#ifndef CORE_PROCESS_CALL_DATATYPE_H
#define CORE_PROCESS_CALL_DATATYPE_H

/**
 * @file core_process_call_base.h
 * @brief The file includes the APIs for operating core process call channel.
 * @author zhengyu.shen
 */

#include <stdint.h>

#define CORE_PROCESS_CALL_REQUEST_TOGGLE_PTR      ((int32_t*)block_ptr->memory_ptr)         /**< The address to store the request toggle.*/
#define CORE_PROCESS_CALL_RESPONSE_TOGGLE_PTR     ((int32_t*)(block_ptr->memory_ptr + 4))   /**< The address to store the response toggle.*/
#define CORE_PROCESS_CALL_VALID_BYTE_OFFSET_PTR   ((int32_t*)(block_ptr->memory_ptr + 8))   /**< The address to store the valid byte size of data section.*/
#define CORE_PROCESS_CALL_TIMEOUT_COUNT_PTR       ((int32_t*)(block_ptr->memory_ptr + 12))  /**< The address to store the timeout count.*/
#define CORE_PROCESS_CALL_ASYNC_ACK_PTR           ((int32_t*)(block_ptr->memory_ptr + 16))  /**< The address to store the asynchronous acknowledgement flag.*/
#define CORE_PROCESS_CALL_MAX_BYTE_OFFSET_PTR     ((int32_t*)(block_ptr->memory_ptr + 20))  /**< The address to store the max byte size of data section.*/
#define CORE_PROCESS_CALL_DATA_PTR                ((uint8_t*)(block_ptr->memory_ptr + 24))  /**< The start address of data section.*/

#define CORE_PROCESS_CALL_REQUEST_TOGGLE        (*CORE_PROCESS_CALL_REQUEST_TOGGLE_PTR)     /**< The request toggle.*/
#define CORE_PROCESS_CALL_RESPONSE_TOGGLE       (*CORE_PROCESS_CALL_RESPONSE_TOGGLE_PTR)    /**< The response toggle.*/
#define CORE_PROCESS_CALL_VALID_BYTE_OFFSET     (*CORE_PROCESS_CALL_VALID_BYTE_OFFSET_PTR)  /**< The valid byte of data section.*/
#define CORE_PROCESS_CALL_TIMEOUT_COUNT         (*CORE_PROCESS_CALL_TIMEOUT_COUNT_PTR)      /**< The timeout count.*/
#define CORE_PROCESS_CALL_ASYNC_ACK             (*CORE_PROCESS_CALL_ASYNC_ACK_PTR)          /**< The asynchronous acknowledgement flag.*/
#define CORE_PROCESS_CALL_MAX_BYTE_OFFSET       (*CORE_PROCESS_CALL_MAX_BYTE_OFFSET_PTR)    /**< The max byte size of data section.*/

/**
 * @brief Defines the data structure for the control section of a core process call channel.
 */
typedef struct
{
    int32_t request_toggle;         /**< The request toggle.*/
    int32_t response_toggle;        /**< The response toggle.*/
    int32_t valid_data_byte_offset; /**< The valid byte of data section.*/
    int32_t timeout_count;          /**< The timeout count.*/
    int32_t async_ack;              /**< The asynchronous acknowledgement flag.*/
    int32_t max_data_byte_offset;   /**< The max byte size of data section.*/
}CoreProcessCallCommData_t;


#endif
