#include "task.h"
#include <functional>  
#include <sched.h>
#include <sys/mman.h>

#define MAX_PRIO    (90)
#define MIN_PRIO    (0)

namespace rcs
{

    Task* Task::this_task()
    {
        return instance_;
    }

    bool Task::run()
    {
        thread_ = std::thread(&Task::threadLoop, this);
        return true;
    }

    bool Task::runOnce()
    {
        thread_ = std::thread(std::bind(&Task::threadOnce, this));
        return true;
    }


    bool Task::stop() 
    {
        terminate(true);
        thread_.join();
        return true;
    }

    bool Task::terminate() const 
    {
        return terminate_;
    }

    void Task::terminate(bool value) 
    {
        terminate_ = value;
    }

    void Task::suspend()
    {
        state_ = SUSPENDING;
    }
    void Task::resume()
    {
        /*if (state_ == NORMAL)*/
            //return;
        //std::unique_lock<std::mutex> lock(action_mtx_);
        /*condition_.notify_one();*/

        state_ = NORMAL;
    }


    void Task::function(Fn f)
    {
        func_main_ = f;
    }

    int Task::period()
    {
        return period_;
    }

    void Task::setTimeout(int ms)
    {
        timeout_cnt_ = ms / period_;
    }

    void Task::clearPeriodCnt()
    {
        period_count_ = 0;
    }

    int Task::periodCnt()
    {
       return period_count_; 
    }

    bool Task::addEvent(int m_sec, Fn f, bool force)
    {
        if (m_sec < period_)
            return false;
        TimerCnt tm_cnt;
        tm_cnt.force = force;
        tm_cnt.count = 0;
        tm_cnt.exprie = m_sec / period_;
        
        std::lock_guard<std::mutex> lk(mtx_);
        tm_func_.push_back(make_pair(tm_cnt, f)); 
    }


    bool Task::lock_memory() 
    {
        return (mlockall(MCL_CURRENT | MCL_FUTURE) != -1);
    }

    bool Task::setPriority(int prio) 
    {
        struct sched_param param;
        param.sched_priority = prio; 
        return (sched_setscheduler(0, SCHED_FIFO, &param) != -1); //set priority
    }


    void Task::threadLoop() 
    {
        using seconds = std::chrono::duration<int, std::chrono::milliseconds::period>;
        
        if ((priority_ > MIN_PRIO) && (priority_ < MAX_PRIO))
        {
            setPriority(priority_);
        }
        if (realtime_)
        {
            lock_memory();
        }
        while(!terminate())
        {
            // Wait (blocking) for data changes
            if (state_ == SUSPENDING)
            {
                std::lock_guard<std::mutex> lk(mtx_);
                std::vector<std::pair<TimerCnt, Fn> >::iterator it;
                for(it = tm_func_.begin(); it != tm_func_.end();)
                {
                    //===run forced things in suspend===
                    if (!it->first.force)
                    {
                        it++;
                        continue;
                    }
                    if (it->first.count++ >= it->first.exprie)
                    {
                        Fn f = it->second;
                        f();
                        tm_func_.erase(it);
                    }
                    else
                    {
                         it++;
                    }
                }
                continue;
                /*std::unique_lock<std::mutex> lock(action_mtx_);*/
                /*condition_.wait(lock);*/
            }
            auto cycle = std::chrono::steady_clock::now() + seconds(period_);
            std::this_thread::sleep_until(cycle);
            period_count_++;
            func_main_();

            std::lock_guard<std::mutex> lk(mtx_);
            std::vector<std::pair<TimerCnt, Fn> >::iterator it;
            for(it = tm_func_.begin(); it != tm_func_.end();)
            {
                if (it->first.count++ >= it->first.exprie)
                {
                    Fn f = it->second;
                    f();
                    tm_func_.erase(it);
                }
                else
                {
                     it++;
                }
            }
        }
    }

    void Task::threadOnce() 
    {
        using seconds = std::chrono::duration<int, std::chrono::milliseconds::period>;
        auto delay = std::chrono::steady_clock::now() + seconds(period_);
        if ((priority_ > MIN_PRIO) && (priority_ < MAX_PRIO))
        {
            setPriority(priority_);
        }
        if (realtime_)
        {
            lock_memory();
        }
        // Wait (blocking) for data changes
        std::this_thread::sleep_until(delay);
        func_main_();        
    }

};
