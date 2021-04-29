#ifndef LOG_MANAGER_DATATYPE_H
#define LOG_MANAGER_DATATYPE_H

#define LOG_OCCUPIED_NUMBER 0x12345678
#define LOG_SHMEM_ADDRESS 0x3A000000
#define LOG_SHMEM_NAME "/dev/mem"
#define LOG_SHMEM_SIZE (8*1024*1024)
#define LOG_BLOCK_NUMBER 32
#define LOG_BLOCK_TEXT_ITEM 399
#define LOG_CTRL_AREA_SIZE 512
#define LOG_ITEM_AREA_SIZE LOG_CTRL_AREA_SIZE
#define LOG_ITEM_AREA_TEXT_SIZE LOG_ITEM_AREA_SIZE - 4
#define LOG_NAME_SIZE 32
namespace virtual_servo_device{

typedef struct{
    uint32_t    shm_occupied;   //default = 0x12345678.
    uint32_t    head_index;     //read.
    uint32_t    tail_index;     //write.
    uint32_t    lost_item_count;//in case buffer is not enough.
    uint32_t    max_item;       //defaut = 399   
    char        thread_name[LOG_NAME_SIZE];

}LogControlArea;

typedef struct{
    int32_t  level;
    char     text_buf[LOG_ITEM_AREA_TEXT_SIZE];

}LogItemArea;

typedef enum{
    LOG_DEBUG = 0,
    LOG_INFO  = 1,
    LOG_WARN  = 2,
    LOG_ERROR = 3,

}MessageLevel;
}
#endif

