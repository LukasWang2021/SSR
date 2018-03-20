#ifndef TP_INTERFACE_LW_SIGNAL_H_
#define TP_INTERFACE_LW_SIGNAL_H_
/**
 * @file lw_signal.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-11-03
 */

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>    
#include <boost/thread/lock_types.hpp>
#include <boost/thread.hpp> 
 
class Semaphore
{
    unsigned int count_;
    boost::mutex mutex_;
    boost::condition_variable condition_;
 
public:
    explicit Semaphore(unsigned int initial) : count_(initial){}
 
    void signal()
    {
        {
            boost::lock_guard<boost::mutex> lock(mutex_);
            ++count_;
        }
        condition_.notify_one(); 
    }
 
    void wait() 
    {
        boost::unique_lock<boost::mutex> lock(mutex_);
        while (count_ == 0)
        {
             condition_.wait(lock);
        }
        --count_;
    }
};

#endif //TP_INTERFACE_LW_SIGNAL_H_
