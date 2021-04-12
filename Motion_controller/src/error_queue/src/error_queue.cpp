#include "error_queue.h"
#include <iostream>

using namespace std;
using namespace base_space;

ErrorQueue::ErrorQueue() 
{ 
    err_cnt_ = 0;
}

ErrorQueue::~ErrorQueue(){ }

ErrorQueue& ErrorQueue::instance()
{
    static ErrorQueue error_queue;
    return error_queue;
}

void ErrorQueue::clear()
{
    unsigned long long err;
    while (err_queue_.pop(err))
    {
        err_cnt_ --;
    }

    err_cnt_ = 0;
}

bool ErrorQueue::push(ErrorCode error_code)
{
    if (error_code  == SUCCESS)
    {
        return false;
    }
    err_cnt_ ++;

    err_queue_.push(error_code);  
    return true;
}

bool ErrorQueue::pop(ErrorCode& error_code)
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

