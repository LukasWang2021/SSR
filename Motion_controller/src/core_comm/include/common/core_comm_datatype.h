#ifndef CORE_COMM_CONFIG_DATATYPE_H
#define CORE_COMM_CONFIG_DATATYPE_H

/**
 * @file core_comm_datatype.h
 * @brief The file includes the definitions of data structures and macros for inter core communication.
 * @author zhengyu.shen
 */

#include <stdint.h>
#include <stddef.h>

#define CORE_COMM_BASE_ADDRESS      0x70000000 // 0x70000000 //0x30000000  /**< The base address of the share memory for inter core communication.*/
#define CORE_COMM_TOTAL_BYTE_SIZE   0x8000000  /**< The share memory byte size for inter core communication.*/

#define CORE_COMM_INIT_SECTION_BYTE_SIZE 65536  /**< The share memory byte size for establish the inter core communication. The size of CoreCommConfigMemoryMap_t must be smaller than it.*/
#define CORE_COMM_BOARDCAST_BYTE_SIZE 4         /**< The share memory byte size for boardcasting signals, only the first byte is used.*/
#define CORE_COMM_CONFIG_BYTE_SIZE (CORE_COMM_INIT_SECTION_BYTE_SIZE - CORE_COMM_BOARDCAST_BYTE_SIZE) /**< The share memory byte size for storing configuration information.*/
#define CORE_COMM_CONFIG_HEAD_BYTE_SIZE  16 /**< The byte size of the head section of the configuration information.*/
#define CORE_COMM_CONFIG_DATA_BYTE_SIZE (CORE_COMM_CONFIG_BYTE_SIZE - CORE_COMM_CONFIG_HEAD_BYTE_SIZE)  /**< The byte size of the data section of the configuration information.*/

#define CORE_COMM_MAX_CPU_NUMBER 256            /**< The maximum supported core number.*/
#define CORE_COMM_MAX_COMM_CHANNEL_NUMBER 1024  /**< The maximum supported channel number.*/

#define COMM_BLOCK_TYPE_COMM_REG            0   /**< Type value for register channel.*/ 
#define COMM_BLOCK_TYPE_FPGA_BUFFER         1   /**< Type value for fpga buffer channel.*/ 
#define COMM_BLOCK_TYPE_BUFFER              2   /**< Type value for buffer channel.*/
#define COMM_BLOCK_TYPE_CIRCLE_BUFFER       3   /**< Type value for circle buffer channel.*/
#define COMM_BLOCK_TYPE_CORE_PROCESS_CALL   4   /**< Type value for core process call channel.*/

#define COMM_BLOCK_TYPE_COMM_REG_STR            "comm_reg"          /**< String value for register channel.*/ 
#define COMM_BLOCK_TYPE_FPGA_BUFFER_STR         "fpga_buffer"       /**< String value for fpga buffer channel.*/ 
#define COMM_BLOCK_TYPE_BUFFER_STR              "buffer"            /**< String value for buffer channel.*/
#define COMM_BLOCK_TYPE_CIRCLE_BUFFER_STR       "circle_buffer"     /**< String value for circle buffer channel.*/
#define COMM_BLOCK_TYPE_CORE_PROCESS_CALL_STR   "core_process_call" /**< String value for core process call channel.*/

/**
 * @brief Defines the state of inter core communication.
 */
typedef enum
{
    CORE_COMM_STATE_INIT = 0,       /**< INIT state: no communication channel is avaiable.*/
    CORE_COMM_STATE_PREOP = 1,      /**< PREOP state: register/fpga buffer/buffer/core process call channels are avaibale.*/
    CORE_COMM_STATE_SAFEOP = 2,     /**< SAFEOP state: circle buffer channels for feedback are avaiable.*/
    CORE_COMM_STATE_OP = 3,         /**< OP state: circle buffer channels for control are avaiable.*/
    CORE_COMM_STATE_UNKNOWN = 4,    /**< Unknown state.*/
}CoreCommState_e;

/** 
 * @note In the following struct definitions, it is tricky for the memory_ptr of BoardcastBlockData_t/CpuAckBlockData_t/CommBlockData_t.\n
 *       They are used only for the convenience of local configuration.
 */
/**
 * @brief Defines boardcast channel.
 */
typedef struct
{
    uint32_t byte_offset;   /**< Byte offset from the CORE_COMM_BASE_ADDRESS.*/
    int32_t from;           /**< The cpu id of the boardcaster.*/
}BoardcastBlockData_t;
/**
 * @brief Defines cpu ack channel.
 */
typedef struct
{
    uint32_t byte_offset;   /**< Byte offset from the base address of the communication block section.*/
    int32_t from;           /**< Cpu id of the sponsor.*/
    int32_t to;             /**< Cpu id of the responder.*/
    char* memory_ptr;       /**< The start address of the channel in share memory.*/
}CpuAckBlockData_t;
/**
 * @brief Defines common communication channel.
 */
typedef struct
{
    int32_t type;           /**< Channel type.*/
    int32_t application_id; /**< Channel application id.*/
    uint32_t byte_offset;   /**< Byte offset from the base address of the communication block section.*/
    int32_t from;           /**< Cpu id of the sponsor.*/
    int32_t to;             /**< Cpu id of the responder.*/
    int32_t param1;         /**< Channel configuration parameter 1.*/
    int32_t param2;         /**< Channel configuration parameter 2.*/
    int32_t param3;         /**< Channel configuration parameter 3.*/
    int32_t param4;         /**< Channel configuration parameter 4.*/
    char* memory_ptr;       /**< The start address of the channel in share memory.*/
}CommBlockData_t;
/**
 * @brief Defines the configuration information of the inter core communication.
 */
typedef struct
{
    uint32_t base_address;          /**< The base address of the communication block section.*/
    uint32_t boardcast_block_num;   /**< The number of boardcast channels. It should always be 1.*/
    uint32_t cpu_ack_block_num;     /**< The number of cpu ack channels.*/
    uint32_t comm_block_num;        /**< The number of common communication channels.*/
    BoardcastBlockData_t boardcast_data;    /**< The configuration information for the boardcast channel.*/
    CpuAckBlockData_t cpu_ack_data[CORE_COMM_MAX_CPU_NUMBER];       /**< The configuration information for the cpu ack channels.*/
    CommBlockData_t comm_data[CORE_COMM_MAX_COMM_CHANNEL_NUMBER];   /**< The configuration information for the common communication channels.*/
}CoreCommConfig_t;

#endif
