/*************************************************************************
	> File Name: log_manager_transmitter.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 18时02分51秒
 ************************************************************************/

#ifndef _LOG_MANAGER_TRANSMITTER_H
#define _LOG_MANAGER_TRANSMITTER_H

namespace fst_log {

class LogTransmitter {
  public:
    void logInfo(const char *format, ...);
    // void logWarn(const char *format, ...);
    // void logError(const char *format, ...);
    // void logInfo(std::string info);
    // void logWarn(std::string warn);
    // void logError(std::string error);
  private:
    
};

}

#endif
