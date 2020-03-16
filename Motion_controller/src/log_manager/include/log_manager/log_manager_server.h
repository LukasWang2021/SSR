/*************************************************************************
	> File Name: log_manager_server.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月27日 星期二 14时08分56秒
 ************************************************************************/

#ifndef _LOG_MANAGER_SERVER_H
#define _LOG_MANAGER_SERVER_H
#include <fstream>
#include <string>

#include <log_manager/log_manager_common.h>


#define MAX_LOG_FILE_RETENTION_TIME     _1D
#define MAX_LOG_FILE_SIZE               _16MB
#define MAX_LOG_FILE_SPACE              _1GB

#define NO_ENOUGH_LOG_SPACE_WARNING     (MAX_LOG_FILE_SPACE * 0.9)

#define MAX_LOG_CONTROL_BLOCK           200     // MAX_LOG_CONTROL_BLOCK should less than 256,
                                                // cause there are 256 IDs at most, and ID=0 is reserved
#define SERVER_ITEM_POOL_SIZE           1000
#define LOG_DIRECTORY                   "/root/log/"

namespace fst_log {

struct LogControlBlock {
    char            id;
    bool            working;
    std::string     name;
    std::string     file_name;
    time_t          file_create_time;
    uintmax_t       character_cnt;
    std::ofstream   file_handle;
    unsigned short  serial_num;
};

}

void log(const char *format, ...);
void info(const char *format, ...);
void warn(const char *format, ...);
void error(const char *format, ...);


uintmax_t totalLogFileSize(boost::filesystem::path &path);
bool delOldestLogFile(boost::filesystem::path &path);
void checkLogSpace(boost::filesystem::path &path);
void cleanControlBlockQueue(void);
bool isFileExist(const char *file);
bool createLogFile(fst_log::LogControlBlock *plcb);

char buildLogControlBlock(std::string &name);
std::string getNameFromID(char id);

int initShareMemory(void);
int initLogSpace(const char *dir);
int initServerLog(const char *dir);
void createLogSpace(const char *dir);

//bool lockFile(std::string &file);
//bool unlockFile(std::string &file);

void public_thread(void);
void receive_thread(void);
void io_thread(void);


#endif
