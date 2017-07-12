/**
 * @file timer.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-11-29
 */
#ifndef TP_INTERFACE_RT_TIMER_H_
#define TP_INTERFACE_RT_TIMER_H_

#include <stdio.h> 
#include <stdlib.h>  

void rtTimerStart();
void rtTimerStop();
void rtTimerAdd(int id, int msec);
void rtTimerWait();
bool rtTimerIsExpired(int id);

void rtMsSleep(int ms);

inline void mSleep(int ms)
{
	usleep(ms*1000);
}

/**
 * @brief: get current time
 *
 * @return: the millisecond of time 
 */
long getCurTime();
long getCurTimeSecond();
bool setTimeSecond(long seconds);

#endif
