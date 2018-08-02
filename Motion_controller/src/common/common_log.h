#ifndef COMMON_LOG_H
#define COMMON_LOG_H

#include "log_manager/log_manager_logger.h"

#define FST_LOG_INIT(name) log_ptr_->initLogger(name)
#define FST_LOG_SET_LEVEL(level) log_ptr_->setDisplayLevel(level)
#define FST_ERROR   log_ptr_->error
#define FST_WARN    log_ptr_->warn
#define FST_INFO    log_ptr_->info

#endif

