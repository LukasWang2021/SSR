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

#define MAX_LOG_FILE_RETENTION_TIME (24 * 60 * 60)      // 24 Hour
#define MAX_LOG_FILE_SIZE           (64 * 1024 * 1024)  // 64MB
#define MAX_LOG_FILE_WRITE_COUNT    (MAX_LOG_FILE_SIZE / LOG_BUFFER_SIZE)

#define MSG_LEVEL_INFO   0x35
#define MSG_LEVEL_WARN   0x3A
#define MSG_LEVEL_ERROR  0x45
#define MSG_LEVEL_NONE   0x4A
#define MSG_DISPLAY_LEVEL  MSG_LEVEL_INFO
#define MSG_LOGGING_LEVEL  MSG_LEVEL_INFO

#endif
