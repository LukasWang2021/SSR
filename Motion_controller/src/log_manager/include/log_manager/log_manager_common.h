/*************************************************************************
	> File Name: log_manager_common.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月28日 星期三 17时27分37秒
 ************************************************************************/

#ifndef _LOG_MANAGER_COMMON_H
#define _LOG_MANAGER_COMMON_H

#define SINGLE_LOG_SIZE    256
#define LOG_BUFFER_SIZE    ( 16 * 1024)
#define MAX_BUFFER_SIZE    (256 * 1024)

#define _1H                              ( 1 * 60 * 60) // 1 Hour
#define _2H                              ( 2 * 60 * 60) // 2 Hour
#define _4H                              ( 4 * 60 * 60) // 4 Hour
#define _8H                              ( 8 * 60 * 60) // 8 Hour
#define _16H                              (16 * 60 * 60) // 16 Hour
#define _24H                              (24 * 60 * 60) // 24 Hour
#define _1D                         ( 1 * 24 * 60 * 60) // 1 Day
#define _2D                         ( 2 * 24 * 60 * 60) // 2 Day
#define _4D                         ( 4 * 24 * 60 * 60) // 4 Day
#define _8D                         ( 8 * 24 * 60 * 60) // 8 Day
#define _16D                        (16 * 24 * 60 * 60) // 16 Day
#define _24D                        (24 * 24 * 60 * 60) // 24 Day
#define _32D                        (32 * 24 * 60 * 60) // 32 Day

#define _1MB                         (   1 * 1024 * 1024)  // 1MB
#define _2MB                         (   2 * 1024 * 1024)  // 2MB
#define _4MB                         (   4 * 1024 * 1024)  // 4MB
#define _8MB                         (   8 * 1024 * 1024)  // 8MB
#define _16MB                        (  16 * 1024 * 1024)  // 16MB
#define _32MB                        (  32 * 1024 * 1024)  // 32MB
#define _64MB                        (  64 * 1024 * 1024)  // 64MB
#define _128MB                       ( 128 * 1024 * 1024)  // 128MB
#define _256MB                       ( 256 * 1024 * 1024)  // 256MB
#define _512MB                       ( 512 * 1024 * 1024)  // 512MB
#define _1GB                     (1 * 1024 * 1024 * 1024)  // 1GB
#define _2GB                     (2 * 1024 * 1024 * 1024)  // 2GB
#define _4GB                     (4 * 1024 * 1024 * 1024)  // 4GB
#define _8GB                     (8 * 1024 * 1024 * 1024)  // 8GB

#define MAX_LOG_FILE_RETENTION_TIME     _1D
#define MAX_LOG_FILE_SIZE               _16MB
#define MAX_LOG_FILE_SPACE              _1GB

#define MAX_LOG_FILE_WRITE_COUNT    (MAX_LOG_FILE_SIZE / LOG_BUFFER_SIZE)
#define NO_ENOUGH_LOG_SPACE_WARNING (MAX_LOG_FILE_SPACE * 0.9)

#define MSG_LEVEL_INFO   0x35
#define MSG_LEVEL_WARN   0x3A
#define MSG_LEVEL_ERROR  0x45
#define MSG_LEVEL_NONE   0x4A
#define MSG_DISPLAY_LEVEL  MSG_LEVEL_INFO
#define MSG_LOGGING_LEVEL  MSG_LEVEL_INFO

#endif
