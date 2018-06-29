/**
 * @file common.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-17
 */
#ifndef INTERPRETER_COMMON_H_
#define INTERPRETER_COMMON_H_
#include "log_manager/log_manager_logger.h"

extern fst_log::Logger glog;

#define LOG_INIT()    \
    do {\
            glog.initLogger("interpreter");\
            glog.setDisplayLevel(fst_log::MSG_LEVEL_INFO);\
    }while(0)

#define FST_INFO    glog.info
#define FST_ERROR   glog.error
#define FST_WARN    glog.warn


#endif
