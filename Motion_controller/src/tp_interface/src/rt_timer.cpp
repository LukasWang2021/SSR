#include "timer.h" 
#include <sys/time.h>  
#include <signal.h> 
#include <sys/syscall.h>
#include <map>
#include "lw_signal.h"

#define sigev_notify_thread_id _sigev_un._tid
#define gettid() syscall(__NR_gettid)

using std::map;

typedef struct _TimerInfo
{
	int		    count;
    int         timer_val;
	bool	    expire_flag;
    Semaphore	*sem;
}TimeInfo;


static map<int, TimeInfo> timer_info_list;
//static unsigned int count_;
static timer_t timer;
static Semaphore semaphore(0);


void sigHandler(int sigo)
{
   // bool sig_flag = false;
    map<int,TimeInfo>::iterator it;
    for (it=timer_info_list.begin();it!=timer_info_list.end();++it)
	{
        it->second.count--;
        if(it->second.count == 0)
        {
            //it->second.sem->signal();
            //sig_flag = true;
            it->second.expire_flag = true;
            it->second.count = it->second.timer_val;
        }
    }
    //if(sig_flag)
     //   semaphore.signal();
}

timer_t createTimer(int sig, int msec)
{
    timer_t timerid;
    struct sigevent sev;
    struct itimerspec its;

    sev.sigev_notify = SIGEV_THREAD_ID | SIGEV_SIGNAL;
    sev.sigev_signo = sig;
    sev.sigev_value.sival_ptr = &timerid;
    sev.sigev_notify_thread_id = gettid();
    signal(sig, sigHandler);
    assert(timer_create(CLOCK_REALTIME, &sev, &timerid) != -1);

    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = msec * 1000000;
    its.it_interval.tv_sec = its.it_value.tv_sec;
    its.it_interval.tv_nsec = its.it_value.tv_nsec;

    assert(timer_settime(timerid, 0, &its, NULL) != -1);
    
    return timerid;
}

void rtTimerStart()
{
    timer = createTimer(SIGALRM, 1);
}

void rtTimerStop()
{
    map<int,TimeInfo>::iterator it;
    for (it=timer_info_list.begin();it!=timer_info_list.end();++it)
	{
        delete it->second.sem;
    }
    timer_info_list.clear();
    timer_delete(timer);
}

void rtTimerAdd(int id, int mseconds)
{
    TimeInfo time_info;
	time_info.timer_val = mseconds; //cnt every time
    time_info.count = time_info.timer_val;
    time_info.sem = new Semaphore(0);
	time_info.expire_flag = false;
	timer_info_list.insert(map<int, TimeInfo>::value_type(id, time_info));  
}

void rtTimerWait()
{
    //timer_info_list[id].sem->wait();
    semaphore.wait();
}

bool rtTimerIsExpired(int id)
{
    try
	{
		if (timer_info_list[id].expire_flag == true)
		{
			timer_info_list[id].expire_flag = false;
			return true;
		}
	}
	catch(std::exception e)
	{
		return false;
	}
	return false;
}

void rtMsSleep(int ms)
{
    struct timespec deadline;
    clockid_t clk = CLOCK_REALTIME;  //CLOCK_REALTIME or CLOCK_MONOTONIC
    clock_gettime(clk, &deadline);

    // Add the time you want to sleep
    deadline.tv_nsec += 1000000*ms;

    // Normalize the time to account for the second boundary
    if(deadline.tv_nsec >= 1000000000) 
    {
        deadline.tv_nsec -= 1000000000;
        deadline.tv_sec++;
    }
    clock_nanosleep(clk, TIMER_ABSTIME, &deadline, NULL);
}


/**
 * @brief: get current time
 *
 * @return: the millisecond of time 
 */
long getCurTime()
{
	struct timeval t_time;
	gettimeofday(&t_time, NULL);
	long cost_time = ((long)t_time.tv_sec)*1000+(long)t_time.tv_usec/1000;

	return cost_time;
}

long getCurTimeSecond()
{
    time_t seconds;

    seconds = time((time_t *)NULL);

    return seconds;
}

bool setTimeSecond(long seconds)
{
    struct timeval tv;
    tv.tv_sec = seconds;  
    tv.tv_usec = 0; 
    if(settimeofday (&tv, (struct timezone *) 0) < 0)  
    {  
        printf("Set system datatime error!/n");  
        return false;  
    }  
    return true;
}
