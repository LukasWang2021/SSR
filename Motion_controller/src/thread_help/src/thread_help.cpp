#include "thread_help.h"


using namespace base_space;

ThreadHelp::ThreadHelp()
{

}

ThreadHelp::~ThreadHelp()
{

}

bool ThreadHelp::run(threadFunc func_ptr, void* data, int priority, bool is_rt)
{
    if(func_ptr == NULL
        || priority < 1
        || priority > 99)
    {
        return false;
    }
    pthread_attr_t attr;
    struct sched_param param;
    pthread_attr_init(&attr);
    if(is_rt)
    {
        param.sched_priority = priority;  
        pthread_attr_setschedpolicy(&attr, SCHED_RR);
        pthread_attr_setschedparam(&attr,   &param);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    }
    pthread_create(&pid_, &attr, func_ptr, data);  
    pthread_attr_destroy(&attr);

    return true;
}

void ThreadHelp::join()
{
    if(pid_ == 0) return;
    pthread_join(pid_, NULL);
}

void ThreadHelp::detach()
{
    if(pid_ == 0) return;
    pthread_detach(pid_);
}


