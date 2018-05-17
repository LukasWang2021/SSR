#include "error_monitor.h"
#include "error_code.h"

using namespace rcs;
Error::Error() 
{ 
    init_err_flag_ = false;
    err_cnt_ = 0;
    warning_level_ = 0;
    pre_code_ = TPI_SUCCESS;
}

Error::~Error(){ }

Error* Error::instance()
{
    static Error error;
    return &error;
}

void Error::clear()
{
    
    U64 err;
    while (err_queue_.pop(err))
    {
        err_cnt_ --;
    }

    err_cnt_ = 0;
    warning_level_ = 0;
    //qianjin 20180312 begin
    init_err_flag_ = false;
    updated_flag_ = false;
    printf("ErrorMonitor: Clear all the errors!\n");
    //qianjin 20180312 begin

    pre_code_ = TPI_SUCCESS;
}

bool Error::add(U64 code)
{
    //the same error or no up errors
    
    if ((code ==  pre_code_) || ((code & 0x0001000000000000) == 0))
    {
        return false;
    }
    updated_flag_ = true;
    FST_ERROR("add error:%llx", code);
    pre_code_ = code;
    err_cnt_ ++;
            
    int level = (code & 0x0000FFFF00000000) >> 32;
    if (level > warning_level_)
        warning_level_ = level;

    err_queue_.push(code);
}

std::string Error::getErrorBytes()
{
    int cnt = 0;
    U64 err[MAX_ERRORS] = {0};
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

    return std::string((const char*)err, MAX_ERRORS*sizeof(U64));
}


bool Error::updated()
{
    bool flag = updated_flag_;
    updated_flag_ = false;
    if(flag) {
        printf("ErrorMonitor: Check Error state updated!\n");
    }
    return flag;
}

bool Error::isInitError()
{
    return init_err_flag_;
}

int Error::getWarningLevel()
{
    return warning_level_;
}
