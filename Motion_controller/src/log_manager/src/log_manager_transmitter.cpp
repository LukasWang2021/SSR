/*************************************************************************
	> File Name: log_manager_transmitter.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 17时59分34秒
 ************************************************************************/

#include <stdarg.h>
#include <stdio.h>
#include <iostream>
#include "log_manager/log_manager_transmitter.h"

using std::cout;
using std::endl;

namespace fst_log {

void LogTransmitter::logInfo(const char *format, ...) {
    va_list vp;
    va_start(vp, format);
    char buf[1024];
    int result = vsprintf(buf, format, vp);
    va_end(vp);

    printf(buf);
}

}








