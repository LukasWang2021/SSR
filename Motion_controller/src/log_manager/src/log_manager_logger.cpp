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

Logger::Logger(void) {
    current_state_ = UNDEFINED;
    display_level_ = MSG_DISPLAY_LEVEL;
    logging_level_ = MSG_LOGGING_LEVEL;
    this->log_content_.clear();
    pthread_mutex_init(&log_mutex_, NULL);
}

Logger::~Logger(void) {
    if (current_state_ == INITIALIZED) {
        cout << "Transmiting local cached logs to server ..." << endl;
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
                cout << "Done!" << endl;
            }
        }

        if (log_content_.size() != 0) {
            cout << "\033[31mThe end of log is lost, lost length=" << log_content_.size() << ".\033[0m" << endl;
        }
        UNLOCK;

        cout << "Send log-out request to server." << endl;
        char buf[8] = "\33CLOSE\33";
        buf[7] = 0;
        if (comm_interface_.send(buf, sizeof(buf), COMM_DONTWAIT)) {
            usleep(200 * 1000);
            cout << "Done!" << endl;
        }
    }

    pthread_mutex_destroy(&log_mutex_);
    cout << "logger exit" << endl;
}

bool Logger::initLogger(const char *log_file_name) {
    if (current_state_ != UNDEFINED) {
        return false;
    }

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
        error("Cannot recevie server response");
        return false;
    }
}

void Logger::setDisplayLevel(unsigned int level) {
    if      (level == MSG_LEVEL_INFO)   display_level_ = MSG_LEVEL_INFO;
    else if (level == MSG_LEVEL_WARN)   display_level_ = MSG_LEVEL_WARN;
    else if (level == MSG_LEVEL_ERROR)  display_level_ = MSG_LEVEL_ERROR;
    else if (level == MSG_LEVEL_NONE)   display_level_ = MSG_LEVEL_NONE;
}

void Logger::setLoggingLevel(unsigned int level) {
    if      (level == MSG_LEVEL_INFO)   logging_level_ = MSG_LEVEL_INFO;
    else if (level == MSG_LEVEL_WARN)   logging_level_ = MSG_LEVEL_WARN;
    else if (level == MSG_LEVEL_ERROR)  logging_level_ = MSG_LEVEL_ERROR;
    else if (level == MSG_LEVEL_NONE)   logging_level_ = MSG_LEVEL_NONE;
}

void Logger::info(const char *format, ...) {
    char buf[SINGLE_LOG_SIZE];
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

void Logger::warn(const char *format, ...) {
    char buf[SINGLE_LOG_SIZE];
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

void Logger::error(const char *format, ...) {
    char buf[SINGLE_LOG_SIZE];
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

void Logger::info(const string &info) {
    char buf[SINGLE_LOG_SIZE];
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

void Logger::warn(const string &warn) {
    char buf[SINGLE_LOG_SIZE];
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

void Logger::error(const string &error) {
    char buf[SINGLE_LOG_SIZE];
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

void Logger::logMessage(const char *msg) {
    string tmp = msg;
    logMessage(tmp);
}

void Logger::logMessage(const string &msg) {
    if (msg[5] == 'O') {
        if (display_level_ <= MSG_LEVEL_INFO) {
            cout << msg;
        }
        if (logging_level_ <= MSG_LEVEL_INFO) {
            LOCK;
            log_content_ += msg;
            UNLOCK;
        }
    }
    else if (msg[5] == 'N') {
        if (display_level_ <= MSG_LEVEL_WARN) {
            cout << "\033[33m" << msg << "\033[0m";
        }
        if (logging_level_ <= MSG_LEVEL_WARN) {
            LOCK;
            log_content_ += msg;
            UNLOCK;
        }
    }
    else {
        if (display_level_ <= MSG_LEVEL_ERROR) {
            cout << "\033[31m" << msg << "\033[0m";
        }
        if (logging_level_ <= MSG_LEVEL_ERROR) {
            LOCK;
            log_content_ += msg;
            UNLOCK;
        }
    }

    LOCK;
    if (log_content_.size() >= LOG_BUFFER_SIZE) {
        if (current_state_ == INITIALIZED) {
            string tmp;
            tmp.assign(log_content_, 0, LOG_BUFFER_SIZE - 1);
            if (comm_interface_.send(tmp.c_str(), LOG_BUFFER_SIZE, COMM_DONTWAIT) == 0) {
                // cout << "sended!" << endl;
                log_content_.erase(0, LOG_BUFFER_SIZE - 1);
            }
        }
        else {
            log_content_.clear();
        }
    }
    UNLOCK;
}

}








