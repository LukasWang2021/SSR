#ifndef THREAD_HELP_H
#define THREAD_HELP_H

/**
 * @file thread_help.h
 * @brief The file includes the class for handling thread operation.
 * @author zhengyu.shen
 */

#include <pthread.h>
#include <sched.h>
#include <functional>

/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief Defines the format of a thread function.
 */
typedef void* (*threadFunc)(void*);

/**
 * @brief ThreadHelp is the object to handle thread.
 */
class ThreadHelp
{
public:
    /**
     * @brief Constructor of the class.
     */    
    ThreadHelp();
    /**
     * @brief Destructor of the class.
     */    
    ~ThreadHelp();
    /**
     * @brief Create a thread.
     * @param [in] func_ptr Thread function.
     * @param [in] data Pointer of the formal parameter of the thread function.
     * @param [in] priority Priority of the thread.
     * @retval true Operation succeed.
     * @retval false Operation failed.
     */
    bool run(threadFunc func_ptr, void* data, int priority, bool is_rt=true);
    /**
     * @brief Wait the thread to exit.
     * @return void
     */    
    void join();
    /**
     * @brief Detach the thread from the main thread.
     * @return void
     */ 
    void detach();

private:
    pthread_t pid_; /**< Thread id.*/
};

}

#endif

