/*************************************************************************
	> File Name: log_manager_logger.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 18时02分51秒
 ************************************************************************/

#ifndef _LOG_MANAGER_LOGGER_H
#define _LOG_MANAGER_LOGGER_H

#include <comm_interface/comm_interface.h>
#include <log_manager/log_manager_common.h>
#include <pthread.h>

namespace fst_log {

class Logger {
  public:
    Logger(void);
    ~Logger(void);
    bool initLogger(const char *log_file_name);
    
    void setDisplayLevel(unsigned int level);
    void setLoggingLevel(unsigned int level);
    
    void info(const char *format, ...);
    void warn(const char *format, ...);
    void error(const char *format, ...);
    
    void info(const std::string &info);
    void warn(const std::string &warn);
    void error(const std::string &error);
  
  private:
    void logMessage(const char *msg);
    void logMessage(const std::string &msg);
    
    unsigned int current_state_;
    unsigned int display_level_;
    unsigned int logging_level_;

    struct CommBuffer{
        char buffer[LOG_BUFFER_SIZE];
        bool isSend;
        bool isAvailable;
    } comm_buffer_;

    bool overflow_flag_;
    int  overflow_count_;

    std::string     log_content_;
    pthread_mutex_t log_mutex_;
    fst_comm_interface::CommInterface comm_interface_;
};

}
#endif
