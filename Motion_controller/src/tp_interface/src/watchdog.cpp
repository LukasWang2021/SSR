#include "watchdog.h"
#include <iostream>

Watchdog::Watchdog() :
    interval_(0),
    timer_(0),
    running_(false) 
{
}

Watchdog::Watchdog(unsigned int milliseconds, std::function<void(void*)> callback, void* params)
{
    Start(milliseconds, callback, params);
}

Watchdog::~Watchdog()
{
}

/**
 * @brief Watchdog start to set the time and callback function
 *
 * @param milliseconds :input>>Watchdog timer 
 * @param callback: intput
 * @param params
 */
void Watchdog::Start(unsigned int milliseconds, std::function<void(void*)> callback, void* params)
{
    interval_ = milliseconds;
    timer_ = 0;
    callback_ = callback;
    running_ = true;
    thread_ = std::thread(&Watchdog::Loop, this,params);
	thread_.detach();
}

/**
 * @brief stop Watchdog
 */
void Watchdog::Stop()
{
    running_ = false;
    thread_.join();
}

/**
 * @brief if you don't want to trigger callback function,then use this function timely
 */
void Watchdog::Pet()
{
    timer_ = 0;
}

/**
 * @brief 
 *
 * @param params :input
 */
void Watchdog::Loop(void* params)
{
    while (running_)
    {
        timer_++;
        if (timer_ >= interval_)
        {
            running_ = false;
            callback_(params);
        }

        std::this_thread::sleep_for (std::chrono::milliseconds(1));
    }
}
