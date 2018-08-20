#include "error_monitor.h"
#include "error_code.h"

using namespace fst_base;

ErrorMonitor::ErrorMonitor() 
{ 
    init_err_flag_ = false;
    err_cnt_ = 0;
    warning_level_ = 0;
    pre_code_ = API_SUCCESS;
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
    init_err_flag_ = false;
    updated_flag_ = false;
    pre_code_ = API_SUCCESS;
}

bool ErrorMonitor::add(unsigned long long code)
{
    //the same error or no up errors
    if ((code ==  pre_code_) || ((code & 0x0001000000000000) == 0))
    {
        return false;
    }
    updated_flag_ = true;
    pre_code_ = code;
    err_cnt_ ++;
            
    int level = (code & 0x0000FFFF00000000) >> 32;
    if (level > warning_level_)
        warning_level_ = level;

    err_queue_.push(code);
    return true;
}

std::string ErrorMonitor::getErrorBytes()
{
    int cnt = 0;
    unsigned long long err[MAX_ERRORS] = {0};
    for (int i = 0; i < MAX_ERRORS; ++i)
    {
        if (!err_queue_.pop(err[i]))
            break;
        err_cnt_ --;
        cnt++;
        // if there is init error 
        // they cannot be popped out
        if (err[i] & 0x0010000000000000) 
        {
            init_err_flag_ = true;
            init_err_qu_.push(err[i]);
            err_cnt_++;
            break;
        }
    }

    for (int i = 0; i < MAX_ERRORS; ++i)
    {
        if (0 != err[i]) err_queue_.push(err[i]);
    }

    return std::string((const char*)err, MAX_ERRORS*sizeof(unsigned long long));
}


bool ErrorMonitor::updated()
{
    bool flag = updated_flag_;
    updated_flag_ = false;
    return flag;
}

bool ErrorMonitor::isInitError()
{
    return init_err_flag_;
}

int ErrorMonitor::getWarningLevel()
{
    return warning_level_;
}

