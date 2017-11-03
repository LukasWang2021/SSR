/*************************************************************************
	> File Name: log_manager_logger.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 18时02分51秒
 ************************************************************************/

#ifndef _LOG_MANAGER_LOGGER_H
#define _LOG_MANAGER_LOGGER_H

#include <log_manager/log_manager_common.h>
#include <pthread.h>

namespace fst_log {

class Logger {
  public:
    Logger(void);
    ~Logger(void);
    
    bool initLogger(const char *name);
    
    void setDisplayLevel(MessageLevel level);
    void setLoggingLevel(MessageLevel level);
    
    void log(const char *format, ...);
    void info(const char *format, ...);
    void warn(const char *format, ...);
    void error(const char *format, ...);
  
  private:
    void writeShareMemory(LogItem *pitem);
    void displayItem(LogItem *pitem);
    
    unsigned int current_state_;
    unsigned int display_level_;
    unsigned int logging_level_;


    ControlArea *ctrl_area_;
    LogFlag     *flag_area_;
    LogItem     *text_area_;

    char            id_;
    unsigned short  serial_num_;
};

}
#endif
