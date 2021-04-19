#ifndef LOG_MANAGER_DATATYPE_H
#define LOG_MANAGER_DATATYPE_H

/**
 * @file log_manager_datatype.h
 * @brief The file is the header file of the common used data structure.
 * @author Feng.Wu
 */

#define LOG_OCCUPIED_NUMBER 0x12345678   /**< The magic number indicates the block is occupied.*/
#define LOG_SHMEM_ADDRESS 0x3D120000     /**< The address of the share memory.*/
#define LOG_SHMEM_NAME "/dev/fst_shmem"  /**< The name of the device when opening the share memory.*/
#define LOG_SHMEM_SIZE (8*1024*1024)     /**< The size of the share memory.*/
#define LOG_BLOCK_NUMBER 32              /**< The maximum number of the log blocks in the share memory.*/
#define LOG_BLOCK_TEXT_ITEM 399          /**< The maximum number of the texts in one log block.*/
#define LOG_CTRL_AREA_SIZE 512           /**< The byte size of the control header occupies.*/
#define LOG_ITEM_AREA_SIZE LOG_CTRL_AREA_SIZE           /**< The byte size of text area occupies.*/
#define LOG_ITEM_AREA_TEXT_SIZE LOG_ITEM_AREA_SIZE - 4  /**< The byte size of the main body of one text.*/
#define LOG_NAME_SIZE 32                                /**< The byte size of the name of the bind thread.*/
#define LOG_TEXT_SIZE LOG_ITEM_AREA_TEXT_SIZE //add for p7a motion_control

/**
 * @brief LogControlArea is the control header of the log block.
 * @details The header contains the resources to operate the text area.
 */
typedef struct{
    uint32_t    shm_occupied;   /**< Indicate the log block is occupied or not, default = 0x12345678.*/
    uint32_t    head_index;     /**< The reading position for log comsumer.*/
    uint32_t    tail_index;     /**< The writing position for log producer.*/
    uint32_t    lost_item_count;/**< The counter increases one if any text failed to be logged.*/
    uint32_t    max_item;       /**< The maximum capacity of the texts, default = 399.*/ 
    char        thread_name[LOG_NAME_SIZE]; /**< The name of the thread occupies the log block.*/
}LogControlArea;

/**
 * @brief LogItemArea is the area of the log texts.
 * @details The area contains the level and the main body of the text.
 */
typedef struct{
	int32_t  level;                             /**< The level of the log text.*/
	char     text_buf[LOG_ITEM_AREA_TEXT_SIZE]; /**< The text.*/
}LogItemArea;

typedef enum{
    LOG_DEBUG = 0,
    LOG_INFO  = 1,
    LOG_WARN  = 2,
    LOG_ERROR = 3,
}MessageLevel;

#endif
