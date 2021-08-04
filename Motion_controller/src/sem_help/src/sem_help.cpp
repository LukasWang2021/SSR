#include "sem_help.h"

using namespace base_space;

SemHelp::SemHelp()
{
    sem_init(&sem_, 0, 0);
    is_taken_ = false;
}

SemHelp::SemHelp(int pshared, int value)
{
    sem_init(&sem_, pshared, value);
    if(value == 0)
        is_taken_ = true;
    else
        is_taken_ = false;
}

SemHelp::~SemHelp()
{
    sem_destroy(&sem_);
}

int SemHelp::take(void)
{
    int ret = sem_wait(&sem_);

    if(ret == 0) is_taken_ = true;
    else         is_taken_ = false;
    
    return ret;
}

int SemHelp::take(int timeout)
{
    struct timespec t;
    t.tv_sec = timeout / 1000; // seconds
    t.tv_nsec = timeout % 1000 * 1000 * 1000; // nano seconds

    int ret = sem_timedwait(&sem_, &t);

    if(ret == 0) is_taken_ = true;
    else         is_taken_ = false;

    return ret;
}

int SemHelp::give(void)
{
    if(!is_taken_) return 0;

    return sem_post(&sem_);
}

bool SemHelp::isTaken(void)
{
    return is_taken_;
}

int SemHelp::getValue(int *val)
{
    return sem_getvalue(&sem_, val);
}
