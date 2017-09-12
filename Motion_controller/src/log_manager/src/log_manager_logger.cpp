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
#include "log_manager/log_manager_logger.h"

#define SUCCESS (unsigned long long int)0
#define LOCK    pthread_mutex_lock(&log_mutex_);
#define UNLOCK  pthread_mutex_unlock(&log_mutex_);

using std::string;
using std::cout;
using std::endl;

namespace fst_log {

static const unsigned int UNDEFINED     = 0x5555;
static const unsigned int INITIALIZED   = 0x5556;

Logger::Logger(void)
{
    current_state_ = UNDEFINED;
    display_level_ = MSG_DISPLAY_LEVEL;
    logging_level_ = MSG_LOGGING_LEVEL;
    log_content_.clear();
    pthread_mutex_init(&log_mutex_, NULL);
    memset(comm_buffer_.buffer, 0, sizeof(comm_buffer_.buffer));
    comm_buffer_.isSend = false;
    comm_buffer_.isAvailable = true;
    overflow_flag_  = false;
    overflow_count_ = 0;
}

Logger::~Logger(void)
{
    if (current_state_ == INITIALIZED) {
        //printf("Transmiting local cached logs to server ... ");
        LOCK;
        while (log_content_.size() > LOG_BUFFER_SIZE) {
            string tmp;
            tmp.assign(log_content_, 0, LOG_BUFFER_SIZE - 1);
            if (comm_interface_.send(tmp.c_str(), LOG_BUFFER_SIZE, COMM_DONTWAIT) == 0) {
                log_content_.erase(0, LOG_BUFFER_SIZE - 1);
                usleep(20 * 1000);
            }
        }

        int length = log_content_.size();
        if (length > 0) {
            string tmp;
            tmp.assign(log_content_, 0, length);
            if (comm_interface_.send(tmp.c_str(), length + 1, COMM_DONTWAIT) == 0) {
                log_content_.erase(0, length);
                usleep(20 * 1000);
                //printf("Done.\n");
            }
        }

        if (log_content_.size() != 0) {
            printf("\033[31mThe end of log is lost, lost length=%zu.\033[0m\n", log_content_.size());
        }
        UNLOCK;

        // printf("Send log-out request to server ...\n");
        char buf[SINGLE_LOG_SIZE] = "\33CLOSE\33";
        buf[7] = 0;
        if (comm_interface_.send(buf, sizeof(buf), COMM_DONTWAIT) == 0) {
            usleep(10 * 1000);
            int loop_cnt = 5;
            memset(buf, 0, sizeof(buf));
            // printf("Done!\n");
            while (loop_cnt > 0) {
                loop_cnt--;
                if (comm_interface_.recv(buf, sizeof(buf), COMM_DONTWAIT) == 0) {
                    // exit successfully
                    if (buf[0] == 'O' && buf[1] == 'K') {
                        pthread_mutex_destroy(&log_mutex_);
                        return;
                    }
                }
                usleep(10 * 1000);
            }
            printf("\033[31mLogger error: cannot receive response from server.\033[0m\n");
        }
        else {
            printf("\033[31mLogger error: fail to send log-out signal to server.\033[0m\n");
        }
    }
    else {
        pthread_mutex_destroy(&log_mutex_);
        return;
    }

    printf("\033[31mLogger terminated with errors.\033[0m\n");
    return;
}

bool Logger::initLogger(const char *log_file_name)
{
    if (current_state_ != UNDEFINED) {return false;}

    info("Initializing log client ...");
    fst_comm_interface::CommInterface tmp_interface;
    if (tmp_interface.createChannel(COMM_REQ, COMM_IPC, "log_public") != SUCCESS) {
        error("Cannot create public channel.");
        return false;
    }

    string tmp(log_file_name);
    if (tmp_interface.send(tmp.c_str(), tmp.size(), COMM_DONTWAIT) != SUCCESS) {
        error("Cannot communicate with log server.");
        return false;
    }
    usleep(200 * 1000);

    char buf[256];
    memset(buf, 0, sizeof(buf));
    if(tmp_interface.recv(buf, sizeof(buf), COMM_DONTWAIT) == 0) {
        if (tmp == buf) {
            info("channel name: '%s'", buf);
            info("Creating log communication channel ...");
            LOCK;
            if (comm_interface_.createChannel(COMM_REQ, COMM_IPC, buf) == SUCCESS) {
                current_state_ = INITIALIZED;
                UNLOCK;
                info("Success! Logging message...");
                return true;
            }
            else {
                error("Cannot create communication channel.");
                UNLOCK;
                return false;
            }
        }
        else {
            error("Cannot create communication channel.");
            return false;
        }
    }
    else {
        error("Cannot receive server response");
        return false;
    }
}

void Logger::setDisplayLevel(unsigned int level)
{
    if      (level == MSG_LEVEL_INFO)   display_level_ = MSG_LEVEL_INFO;
    else if (level == MSG_LEVEL_WARN)   display_level_ = MSG_LEVEL_WARN;
    else if (level == MSG_LEVEL_ERROR)  display_level_ = MSG_LEVEL_ERROR;
    else if (level == MSG_LEVEL_NONE)   display_level_ = MSG_LEVEL_NONE;
}

void Logger::setLoggingLevel(unsigned int level)
{
    if      (level == MSG_LEVEL_INFO)   logging_level_ = MSG_LEVEL_INFO;
    else if (level == MSG_LEVEL_WARN)   logging_level_ = MSG_LEVEL_WARN;
    else if (level == MSG_LEVEL_ERROR)  logging_level_ = MSG_LEVEL_ERROR;
    else if (level == MSG_LEVEL_NONE)   logging_level_ = MSG_LEVEL_NONE;
}

void Logger::info(const char *format, ...)
{
    char buf[SINGLE_LOG_SIZE];
    memset(buf, 0, SINGLE_LOG_SIZE);
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ INFO][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len = len + vsnprintf(buf + len, SINGLE_LOG_SIZE - len - 1, format, vp);
    va_end(vp);
    if (len > SINGLE_LOG_SIZE - 2) len = SINGLE_LOG_SIZE - 2;
    buf[len]     = '\n';
    buf[len + 1] = '\0';
    
    logMessage(buf);
}

void Logger::warn(const char *format, ...)
{
    char buf[SINGLE_LOG_SIZE];
    memset(buf, 0, SINGLE_LOG_SIZE);
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ WARN][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len = len + vsnprintf(buf + len, SINGLE_LOG_SIZE - len - 1, format, vp);
    va_end(vp);
    if (len > SINGLE_LOG_SIZE - 2) len = SINGLE_LOG_SIZE - 2;
    buf[len] = '\n';
    buf[len + 1] = '\0';
    
    logMessage(buf);
}

void Logger::error(const char *format, ...)
{
    char buf[SINGLE_LOG_SIZE];
    memset(buf, 0, SINGLE_LOG_SIZE);
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ERROR][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len += vsnprintf(buf + len, SINGLE_LOG_SIZE - len - 1, format, vp);
    va_end(vp);
    if (len > SINGLE_LOG_SIZE - 2) len = SINGLE_LOG_SIZE - 2;
    buf[len] = '\n';
    buf[len + 1] = '\0';
    
    logMessage(buf);
}

void Logger::info(const string &info)
{
    char buf[SINGLE_LOG_SIZE];
    memset(buf, 0, SINGLE_LOG_SIZE);
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    sprintf(buf, "[ INFO][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    string tmp = buf;
    tmp = tmp + info + '\n';
    if (tmp.size() > SINGLE_LOG_SIZE) {
        tmp.erase(SINGLE_LOG_SIZE);
    }

    logMessage(tmp);
}

void Logger::warn(const string &warn)
{
    char buf[SINGLE_LOG_SIZE];
    memset(buf, 0, SINGLE_LOG_SIZE);
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    sprintf(buf, "[ WARN][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    string tmp = buf;
    tmp = tmp + warn + '\n';
    if (tmp.size() > SINGLE_LOG_SIZE) {
        tmp.erase(SINGLE_LOG_SIZE);
    }

    logMessage(tmp);
}

void Logger::error(const string &error)
{
    char buf[SINGLE_LOG_SIZE];
    memset(buf, 0, SINGLE_LOG_SIZE);
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    sprintf(buf, "[ERROR][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    string tmp = buf;
    tmp = tmp + error + '\n';
    if (tmp.size() > SINGLE_LOG_SIZE) {
        tmp.erase(SINGLE_LOG_SIZE);
    }

    logMessage(tmp);
}

void Logger::logMessage(const char *msg)
{
    string tmp = msg;
    logMessage(tmp);
}

void Logger::logMessage(const string &msg)
{
    if (msg[5] == 'O') {
        if (display_level_ <= MSG_LEVEL_INFO) {
            printf("%s", msg.c_str());
        }
        if (current_state_ == INITIALIZED && logging_level_ <= MSG_LEVEL_INFO) {
            LOCK;
            if (log_content_.size() < MAX_BUFFER_SIZE) {
                if (overflow_flag_) {
                    char buf[SINGLE_LOG_SIZE];
                    memset(buf, 0, SINGLE_LOG_SIZE);
                    struct timeval time_now;
                    gettimeofday(&time_now, NULL);
                    sprintf(buf, "[ WARN][%ld.%6ld]%d log messages above this line has been dropped, caused by buffer overflow\n",
                            time_now.tv_sec, time_now.tv_usec, overflow_count_);
                    log_content_ += buf;
                    overflow_flag_  = false;
                    overflow_count_ = 0;
                }
                log_content_ += msg;
            }
            else {
                overflow_count_++;
                if (overflow_flag_ == false)
                    overflow_flag_ = true;
            }
            UNLOCK;
        }
    }
    else if (msg[5] == 'N') {
        if (display_level_ <= MSG_LEVEL_WARN) {
            printf("\033[33m%s\033[0m", msg.c_str());
        }
        if (current_state_ == INITIALIZED && logging_level_ <= MSG_LEVEL_WARN) {
            LOCK;
            if (log_content_.size() < MAX_BUFFER_SIZE) {
                if (overflow_flag_) {
                    char buf[SINGLE_LOG_SIZE];
                    memset(buf, 0, SINGLE_LOG_SIZE);
                    struct timeval time_now;
                    gettimeofday(&time_now, NULL);
                    sprintf(buf, "[ WARN][%ld.%6ld]%d log messages above this line has been dropped, caused by buffer overflow\n",
                            time_now.tv_sec, time_now.tv_usec, overflow_count_);
                    log_content_ += buf;
                    overflow_flag_  = false;
                    overflow_count_ = 0;
                }
                log_content_ += msg;
            }
            else {
                overflow_count_++;
                if (overflow_flag_ == false)
                    overflow_flag_ = true;
            }
            UNLOCK;
        }
    }
    else if (msg[5] == 'R') {
        if (display_level_ <= MSG_LEVEL_ERROR) {
            printf("\033[31m%s\033[0m", msg.c_str());
        }
        if (current_state_ == INITIALIZED && logging_level_ <= MSG_LEVEL_ERROR) {
            LOCK;
            if (log_content_.size() < MAX_BUFFER_SIZE) {
                if (overflow_flag_) {
                    char buf[SINGLE_LOG_SIZE];
                    memset(buf, 0, SINGLE_LOG_SIZE);
                    struct timeval time_now;
                    gettimeofday(&time_now, NULL);
                    sprintf(buf, "[ WARN][%ld.%6ld]%d log messages above this line has been dropped, caused by buffer overflow\n",
                            time_now.tv_sec, time_now.tv_usec, overflow_count_);
                    log_content_ += buf;
                    overflow_flag_  = false;
                    overflow_count_ = 0;
                }
                log_content_ += msg;
            }
            else {
                overflow_count_++;
                if (overflow_flag_ == false)
                    overflow_flag_ = true;
            }
            UNLOCK;
        }
    }

    if (current_state_ == INITIALIZED) {
        LOCK;
        if (log_content_.size() >= LOG_BUFFER_SIZE) {
            if (comm_buffer_.isAvailable == true) {
                memset(comm_buffer_.buffer, 0, LOG_BUFFER_SIZE);
                size_t len = log_content_.copy(comm_buffer_.buffer, LOG_BUFFER_SIZE);
                log_content_.erase(0, len);
                if (comm_interface_.send(comm_buffer_.buffer, LOG_BUFFER_SIZE, COMM_DONTWAIT) == 0) {
                    comm_buffer_.isSend = true;
                    comm_buffer_.isAvailable = false;
                }
                else {
                    comm_buffer_.isSend = false;
                    comm_buffer_.isAvailable = false;
                }
            }
        }

        if (comm_buffer_.isAvailable == false) {
            if (comm_buffer_.isSend == true) {
                char buf[SINGLE_LOG_SIZE];
                memset(buf, 0, SINGLE_LOG_SIZE);
                if (comm_interface_.recv(buf, SINGLE_LOG_SIZE, COMM_DONTWAIT) == 0) {
                    comm_buffer_.isAvailable = true;
                }
            }
            else {
                if (comm_interface_.send(comm_buffer_.buffer, LOG_BUFFER_SIZE, COMM_DONTWAIT) == 0) {
                    comm_buffer_.isSend = true;
                }
            }
        }
        UNLOCK;
    }
}

}








