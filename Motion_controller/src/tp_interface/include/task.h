/**
 * @file task.h
 * @brief 
 * @author WangWei
 * @version 1.0.1
 * @date 2017-07-20
 */


#ifndef RCS_TASK_H_
#define RCS_TASK_H_

#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
namespace rcs
{
    typedef struct _TimerCnt
    {
        bool    force;
        int     count;
        int     exprie;
    }TimerCnt;
         
    typedef enum _Task_State
    {
        NORMAL, 
        SUSPENDING,
    }Task_State;

    typedef std::function<void()>  Fn;

    class Task
    {
    public:
        Task(int period=100, int priority=-1, bool realtime=false)
            :period_(period), priority_(priority)
        {
            state_ = NORMAL;
            terminate_ = false;
            instance_ = this;
        }
        Task* this_task();

        bool run();
        bool runOnce();
        bool stop();
        bool terminate() const;
        void terminate(bool value);
        void suspend();
        void resume();

        void function(Fn f);
        int period();
        void setTimeout(int ms);
        void clearPeriodCnt();
        int periodCnt();

        bool addEvent(int m_sec, Fn f, bool force = false);
    private:
        std::thread thread_;

        bool lock_memory();
        bool setPriority(int prio);
        void threadLoop();
        void threadOnce();

        std::atomic_bool terminate_;
        std::atomic_bool realtime_;
        int data_;
        int period_;
        int priority_;
        int period_count_;
        int timeout_cnt_;        
        Fn                   func_main_;
        Task                    *instance_;
       // std::condition_variable condition_;
       // std::mutex              action_mtx_;
        std::mutex              mtx_;
        std::atomic<Task_State> state_;

        std::vector<std::pair<TimerCnt, Fn>>   tm_func_;
    };
}

#endif //RCS_TASK_H_
