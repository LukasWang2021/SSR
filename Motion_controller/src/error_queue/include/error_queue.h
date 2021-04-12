#ifndef ERROR_QUEUE_H_ 
#define ERROR_QUEUE_H_

/**
 * @file error_queue.h
 * @brief The file is the header file of class "ErrorQueue".
 * @author Feng.Wu
 */

#include "lockfree_queue.h"
#include "common_error_code.h"
#include <string>
#include <list>

/**
 * @brief base_space includes all foundational definitions and realizations
 */
namespace base_space
{

/**
 * @brief ErrorQueue is a queue to store the errors.
 * @details Error producers push the errors to the tail of the singleton queue while the comsumer pops errors from the head.
 */
class ErrorQueue
{
  public:
    /**
     * @brief Destructor of the class. 
     */  
	~ErrorQueue();

    /**
     * @brief Using Mayer's singleton..
     * @return The referrence of the singleton.
     */
    static ErrorQueue& instance();
    
    /**
     * @brief clear the error queue.
     * @return void
     */
    void clear();

    /**
     * @brief Push one error into the tail of the singleton queue.
     * @param [in] error_code The value of the error code.
     * @retval true success.
     * @retval false If the code is SUCCESS(0).
     */
    bool push(ErrorCode error_code);

    /**
     * @brief Pop one error from the head of the singleton queue.
     * @param [in] error_code The value of the error code.
     * @retval true success.
     * @retval false if the code is SUCCESS(0).
     */
    bool pop(ErrorCode& error_code);

  private:
    /**
     * @brief Constructor of the class.
     */
    ErrorQueue();
    
    std::atomic_int err_cnt_;   //number of error codes
    LockFreeQueue<ErrorCode> err_queue_; //queue to store error codes
};

}

#endif

