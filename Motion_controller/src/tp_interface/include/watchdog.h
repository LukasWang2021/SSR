/**
 * @file watchdog.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-10-18
 */
#ifndef TP_INTERFACE_WATCHDOG_H_
#define TP_INTERFACE_WATCHDOG_H_
//#pragma once

#include <thread>
#include <atomic>

class Watchdog
{
  public:
    Watchdog();
    Watchdog(unsigned int milliseconds, std::function<void(void*)> callback, void* params);
    ~Watchdog();
	/**
	 * @brief Watchdog start to set the time and callback function
	 *
	 * @param milliseconds :input>>Watchdog timer 
	 * @param callback: intput
	 * @param params
	 */
    void Start(unsigned int milliseconds, std::function<void(void*)> callback, void* params);
	/**
	 * @brief stop Watchdog
	 */
    void Stop();
	/**
	 * @brief if you don't want to trigger callback function,then use this function timely
	 */
    void Pet();

  private:
    unsigned int interval_;					//time interval
    std::atomic<unsigned int> timer_;		//
    std::atomic<bool> running_;				//running flag
    std::thread thread_;					//
    std::function<void(void*)> callback_;	//callback function
	/**
	 * @brief 
	 *
	 * @param params :input
	 */
    void Loop(void* params);
};

#endif
