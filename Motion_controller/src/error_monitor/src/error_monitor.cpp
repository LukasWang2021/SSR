#include "error_monitor.h"
#include "error_code.h"
#include <iostream>

using namespace std;
using namespace fst_base;

ErrorMonitor::ErrorMonitor() 
{ 
    err_cnt_ = 0;
    warning_level_ = 0;
    pre_code_ = SUCCESS;
}

ErrorMonitor::~ErrorMonitor(){ }

ErrorMonitor* ErrorMonitor::instance()
{
    static ErrorMonitor error_monitor;
    return &error_monitor;
}

void ErrorMonitor::clear()
{
    
    unsigned long long err;
    while (err_queue_.pop(err))
    {
        err_cnt_ --;
    }

    err_cnt_ = 0;
    warning_level_ = 0;
    pre_code_ = SUCCESS;
}

bool ErrorMonitor::add(unsigned long long code)
{
    //the same error or no up errors
    if ((code ==  pre_code_) || ((code & 0x0001000000000000) == 0))
    {
        return false;
    }
    pre_code_ = code;
    err_cnt_ ++;
            
    int level = (code & 0x0000FFFF00000000) >> 32;
    if (level > warning_level_)
        warning_level_ = level;

    err_queue_.push(code);  
    return true;
}

bool ErrorMonitor::pop(unsigned long long& error_code)
{
    if(err_queue_.pop(error_code))
    {
        --err_cnt_;
        return true;
    }
    else 
    {
        error_code = 0;
        return false;
    }
}

int ErrorMonitor::getWarningLevel()
{
    return warning_level_;
}

