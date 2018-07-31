#ifndef COMMON_LOG_H
#define COMMON_LOG_H

#include "log_manager/log_manager_logger.h"

#define FST_LOG_INIT(name) log_.initLogger(name)
#define FST_LOG_SET_LEVEL(level) log_.setDisplayLevel(level)
#define FST_ERROR   log_.error
#define FST_WARN    log_.warn
#define FST_INFO    log_.info

#endif

