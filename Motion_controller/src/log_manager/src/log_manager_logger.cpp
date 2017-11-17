/*************************************************************************
	> File Name: log_manager_logger.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 17时59分34秒
 ************************************************************************/

#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <sys/time.h>
#include <stdlib.h>  
#include <sys/mman.h>  
#include <fcntl.h>  
#include <sys/stat.h> 

#include <log_manager/log_manager_shm_structure.h>
#include <log_manager/log_manager_logger.h>

using std::string;
using std::cout;
using std::endl;

namespace fst_log {

static const unsigned int UNDEFINED     = 0x5555;
static const unsigned int INITIALIZED   = 0x5556;

Logger::Logger(void)
{
    current_state_ = UNDEFINED;
    display_level_ = MSG_LEVEL_INFO;
    logging_level_ = MSG_LEVEL_LOG;
    serial_num_ = 0;

    ctrl_area_ = NULL;
    flag_area_ = NULL;
    text_area_ = NULL;

    id_ = 0;
}

Logger::~Logger(void)
{
    if (current_state_ != INITIALIZED) {return;}

    info("Log client log-out ...");
    
    int exp = 0;
    int cnt = 0;

    while (!ctrl_area_->register_block.reg_in.compare_exchange_weak(exp, 1)) {
        exp = 0;
        cnt++;
        usleep(10 * 1000);
        
        if (cnt > 50)
            break;
    }

    if (cnt > 50) {
        error(" -Conmunication time-out when sending request");
        return;
    }
    
    ctrl_area_->register_block.id = id_;
    ctrl_area_->register_block.flag_in  = true;

    cnt = 0;
    while (!ctrl_area_->register_block.flag_out) {
        cnt++;
        usleep(10 * 1000);
        
        if (cnt > 10)
            break;
    }

    if (cnt > 10) {
        error(" -Conmunication time-out when wating response");
        ctrl_area_->register_block.flag_in  = false;
        ctrl_area_->register_block.reg_in = 0;
        return;
    }

    ctrl_area_->register_block.flag_out = false;
    ctrl_area_->register_block.reg_in = 0;

    info(" -Success");
}

bool Logger::initLogger(const char *name)
{
    if (current_state_ != UNDEFINED) {return false;}

    info("Initializing log client (%s) ...", name);

    int fd = shm_open("fst_log_shm", O_RDWR, 00777);
    
    if (-1 == fd) {
        error(" -Error in openMem(): failed on opening sharedmem");
        return false;
    }

    char *ptr = (char*) mmap(NULL, LOG_MEM_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    
    if (ptr == MAP_FAILED) {
        close(fd);
        error(" -Error in openMem(): failed on mapping process sharedmem");
        return false;
    }

    ctrl_area_ = GET_CONTROL_AREA_PTR(ptr);
    flag_area_ = GET_FLAG_AREA_PTR(ptr);
    text_area_ = GET_TEXT_AREA_PTR(ptr);

    int exp = 0;
    int cnt = 0;
    while (!ctrl_area_->register_block.reg_in.compare_exchange_weak(exp, 1)) {
        cnt++;
        exp = 0;
        usleep(10 * 1000);
        
        if (cnt > 50)
            break;
    }

    if (cnt > 50) {
        error(" -Conmunication time-out");
        return false;
    }
    
    if (strlen(name) > 255) {
        memcpy(ctrl_area_->register_block.name, name, 255);
        ctrl_area_->register_block.name[255] = '\0';
    }
    else {
        memset(ctrl_area_->register_block.name, 0, 256);
        strcpy(ctrl_area_->register_block.name, name);
    }

    ctrl_area_->register_block.id = 0;
    ctrl_area_->register_block.flag_out = false;
    ctrl_area_->register_block.flag_in  = true;

    cnt = 0;
    while (!ctrl_area_->register_block.flag_out) {
        cnt++;
        usleep(10 * 1000);
        
        if (cnt > 10)
            break;
    }

    if (cnt > 10) {
        error(" -Conmunication time-out");
        ctrl_area_->register_block.flag_in  = false;
        ctrl_area_->register_block.reg_in = 0;
        return false;
    }

    id_ = ctrl_area_->register_block.id;
    ctrl_area_->register_block.id = 0;
    ctrl_area_->register_block.flag_out = false;
    ctrl_area_->register_block.flag_in  = false;
    ctrl_area_->register_block.reg_in   = 0;

    if (0 != id_) {
        current_state_ = INITIALIZED;
        info("Log client initialize Success, ID=%d, logging message...", id_);
        return true;
    }
    else {
        error(" -Log-in request refused: cannot get channel ID");
        return false;
    }
}

void Logger::setDisplayLevel(MessageLevel level)
{
    display_level_ = level;
}

void Logger::setLoggingLevel(MessageLevel level)
{
    logging_level_ = level;
}

void Logger::displayItem(LogItem *pitem)
{
    if (pitem->level < display_level_)
        return;
    
    switch (pitem->level) {
        case MSG_LEVEL_LOG:
            printf("\033[0m[  LOG][%ld.%6ld]%s", pitem->stamp.tv_sec, pitem->stamp.tv_usec, pitem->text);
            break;

        case MSG_LEVEL_INFO:
            printf("\033[0m[ INFO][%ld.%6ld]%s", pitem->stamp.tv_sec, pitem->stamp.tv_usec, pitem->text);
            break;

        case MSG_LEVEL_WARN:
            printf("\033[33m[ WARN][%ld.%6ld]%s\033[0m", pitem->stamp.tv_sec, pitem->stamp.tv_usec, pitem->text);
            break;
        
        case MSG_LEVEL_ERROR:
            printf("\033[31m[ERROR][%ld.%6ld]%s\033[0m", pitem->stamp.tv_sec, pitem->stamp.tv_usec, pitem->text);
            break;
        
        default:
            printf("\033[41m[OTHER][%ld.%6ld]Log client internal fault\033[0m\n",
                   pitem->stamp.tv_sec, pitem->stamp.tv_usec);
    }
}

void Logger::writeShareMemory(LogItem *pitem)
{
    if (current_state_ != INITIALIZED) return;

    if (pitem->level < logging_level_) return;
    
    int cnt = 0;
    int target, tmp_out, next;

    serial_num_++;
    pitem->number = serial_num_;

    for (;;) {
        target  = ctrl_area_->index_in;
        tmp_out = ctrl_area_->index_out;

        if (flag_area_[target] == ITEM_FREE) {
            next    = target + 1;
            if (next == LOG_ITEM_COUNT) next = 0;
            
            if (true == ctrl_area_->index_in.compare_exchange_weak(target, next)) {
                memcpy(&text_area_[target], pitem, sizeof(LogItem));
                flag_area_[target]  = ITEM_INUSE;
                break;
            }
        }
        else {
            // shm full
            next    = tmp_out + 1;
            if (next == LOG_ITEM_COUNT) next = 0;

            if (true == ctrl_area_->index_out.compare_exchange_weak(tmp_out, next)) {
                // drop a log item
                flag_area_[tmp_out] = ITEM_FREE;
            }
        }

        cnt++;
        if (cnt > 10)     break;
    }
}

void Logger::log(const char *format, ...)
{
    LogItem item;

    struct timeval time_now;
    gettimeofday(&time_now, NULL);

    item.id    = id_;
    item.level = MSG_LEVEL_LOG;
    item.stamp = time_now;

    va_list vp;
    va_start(vp, format);
    int len = vsnprintf(item.text, LOG_TEXT_SIZE, format, vp);
    va_end(vp);
    if (len >= LOG_TEXT_SIZE - 1)   len = LOG_TEXT_SIZE - 2;
    item.text[len++] = '\n';
    item.text[len]   = '\0';

    displayItem(&item);
    writeShareMemory(&item);
}

void Logger::info(const char *format, ...)
{
    LogItem item;

    struct timeval time_now;
    gettimeofday(&time_now, NULL);

    item.id    = id_;
    item.level = MSG_LEVEL_INFO;
    item.stamp = time_now;

    va_list vp;
    va_start(vp, format);
    int len = vsnprintf(item.text, LOG_TEXT_SIZE, format, vp);
    va_end(vp);
    if (len >= LOG_TEXT_SIZE - 1)   len = LOG_TEXT_SIZE - 2;
    item.text[len++] = '\n';
    item.text[len]   = '\0';

    displayItem(&item);
    writeShareMemory(&item);
}

void Logger::warn(const char *format, ...)
{
    LogItem item;

    struct timeval time_now;
    gettimeofday(&time_now, NULL);

    item.id    = id_;
    item.level = MSG_LEVEL_WARN;
    item.stamp = time_now;

    va_list vp;
    va_start(vp, format);
    int len = vsnprintf(item.text, LOG_TEXT_SIZE, format, vp);
    va_end(vp);
    if (len >= LOG_TEXT_SIZE - 1)   len = LOG_TEXT_SIZE - 2;
    item.text[len++] = '\n';
    item.text[len]   = '\0';

    displayItem(&item);
    writeShareMemory(&item);
}

void Logger::error(const char *format, ...)
{
    LogItem item;

    struct timeval time_now;
    gettimeofday(&time_now, NULL);

    item.id    = id_;
    item.level = MSG_LEVEL_ERROR;
    item.stamp = time_now;

    va_list vp;
    va_start(vp, format);
    int len = vsnprintf(item.text, LOG_TEXT_SIZE, format, vp);
    va_end(vp);
    if (len >= LOG_TEXT_SIZE - 1)   len = LOG_TEXT_SIZE - 2;
    item.text[len++] = '\n';
    item.text[len]   = '\0';

    displayItem(&item);
    writeShareMemory(&item);
}


}








