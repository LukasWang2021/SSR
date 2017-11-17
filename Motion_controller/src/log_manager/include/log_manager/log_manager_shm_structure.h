/*************************************************************************
	> File Name: log_manager_shm_structure.h
	> Author: 
	> Mail: 
	> Created Time: 2017年10月25日 星期三 09时47分42秒
 ************************************************************************/

#ifndef _LOG_MANAGER_SHM_STRUCTURE_H
#define _LOG_MANAGER_SHM_STRUCTURE_H

#include <atomic>
#include <sys/time.h>


#define LOG_MEM_CTRL_AREA   _8KB 
#define LOG_MEM_FLAG_AREA   _8KB
#define LOG_MEM_TEXT_AREA   (LOG_MEM_SIZE - LOG_MEM_CTRL_AREA - LOG_MEM_FLAG_AREA)

#define LOG_ITEM_SIZE       256
#define LOG_ITEM_COUNT      (LOG_MEM_TEXT_AREA / LOG_ITEM_SIZE)
#define LOG_TEXT_SIZE       (LOG_ITEM_SIZE - 28)

#define     ITEM_FREE       0
#define     ITEM_INUSE      1

namespace fst_log {

typedef     char            LogFlag;

struct RegisterArea {
    std::atomic<int>    reg_in;
    bool                flag_in;
    bool                flag_out;
    char                name[256];
    char                id;
};

struct ControlArea {
    std::atomic<int>    server_on;
    std::atomic<int>    index_in;
    std::atomic<int>    index_out;
    RegisterArea        register_block;
};

struct LogItem {
    char                id;
    char                level;
    unsigned short      number;
    //char                dummy_1;
    //char                dummy_2;
    int                 dummy_3;
    struct timeval      stamp;
    char                text[LOG_TEXT_SIZE];
};

}

#define GET_CONTROL_AREA_PTR(ptr)   (ControlArea *)(ptr)
#define GET_FLAG_AREA_PTR(ptr)      (LogFlag *)(ptr + LOG_MEM_CTRL_AREA)
#define GET_TEXT_AREA_PTR(ptr)      (LogItem *)(ptr + LOG_MEM_CTRL_AREA + LOG_MEM_FLAG_AREA);


#endif
