#ifndef TB_QUEUE_H
#define TB_QUEUE_H

/**
 * @file tb_queue.h
 * @brief The file includes the template of trajectory block queue.
 * @author zhengyu.shen
 */

#include <stdint.h>
#include <mutex>
/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief The template class stores trajectory block as an first in first out queue.
 * @details The function block queue can add a function block to its tail and remove one from its head.\n
 *          The operations of the class is thread safe.\n
 */
template<typename T>
class TBQueue
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] max_size The maximum number of function blocks that is allowed to store in the queue.
     */    
    TBQueue(uint32_t max_size)
    {
        max_size_ = max_size;
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        is_last_cnt_ = false;
        object_ptr_ = new T[max_size_];
    }
    /**
     * @brief Destructor of the class.
     */    
    ~TBQueue()
    {
        if(object_ptr_ != NULL)
        {
            delete[] object_ptr_;
            object_ptr_ = NULL;
        }
    }
    /**
     * @brief Remove all trajectory blocks. Set queue size to zero.
     * @return void
     */
    void clear()
    {
        mutex_.lock();
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        is_last_cnt_ = false;
        mutex_.unlock();
    }
    /**
     * @brief Get the reference of the next trajectory block to be pushed back.
     * @return The reference of the trajectory block.
     */ 
    T& getNextPushBackTB()
    {
        return object_ptr_[tail_];
    }
    /**
     * @brief Add the trajectory block which is acquired by API getNextPushBackTB to the tail of the queue.
     * @retval true Add operation success.
     * @retval false Add operation failed because of the queue has already been full.
     * @par code example:
     * @code 
     *     TBQueue<int> tbq;
     *     int& tb_ref = tbq.getNextPushBackTB();
     *     tb_ref = 1;
     *     tbq.pushBack();
     * @endcode
     */    
    bool pushBack()
    {
        mutex_.lock();
        if(size_ >= max_size_)
        {
            mutex_.unlock();
            return false;
        }        
        pulseOne(tail_);
        ++size_;
        mutex_.unlock();
        return true;
    }
    /**
     * @brief Get the reference of the next trajectory block to be poped out.
     * @return The reference of the trajectory block.
     */ 
    T& getNextPopFrontTB()
    {
        return object_ptr_[head_];
    }
    /**
     * @brief Remove the trajectory block at the head of the queue.
     * @retval true Remove operation success.
     * @retval false Remove operation failed because of the queue has already been empty.
     * @par code example:
     * @code 
     *     TBQueue<int> tbq;
     *     ...... // insert some element into the tbq
     *     int& tb_ref = tbq.getNextPopFrontTB();
     *     std::cout<<"value of the element to be poped is "<<tb_ref<<std::endl;
     *     tbq.popFront();
     * @endcode
     */ 
    bool popFront()
    {
        mutex_.lock();
        if(size_ == 0)
        {
            mutex_.unlock();
            return false;
        }
        pulseOne(head_);
        --size_;
        mutex_.unlock();
        return true;
    }
    /**
     * @brief Get the current size of the queue.
     * @return The size of the queue.
     */    
    uint32_t size()
    {
        return size_;
    }
    /**
     * @brief Check if the queue is in full size.
     * @retval true The queue is in full size.
     * @retval false The queue is not in full size.
     */
    bool isFull()
    {
        return (size_ >= max_size_ ? true:false);
    }
    /**
     * @brief Check if the queue is empty.
     * @retval true The queue is empty.
     * @retval false The queue is not empty.
     */
    bool isEmpty()
    {
        return (size_ == 0 ? true:false);
    }
    /**
     * @brief Check if the last poped trajectory block is the end of the whole trajectory or not.
     * @retval true The last poped trajectory is not the end of the whole trajectory.
     * @retval false The last poped trajectory is the end of the whole trajectory.
     */
    bool isLastCnt()
    {
        return is_last_cnt_;
    }
    /**
     * @brief Set is_last_cnt_ flag. 
     * @param [in] value The value of is_last_cnt_.
     * @return void
     */
    void setLastCnt(bool value)
    {
        is_last_cnt_ = value;
    }
    
private:
    uint32_t max_size_; /**< Max size of the queue.*/
    T* object_ptr_;     /**< Pointer of the queue data.*/
    uint32_t head_;     /**< Index of the head of the queue.*/
    uint32_t tail_;     /**< Index of the tail of the queue.*/
    uint32_t size_;     /**< Current size of the queue.*/
    bool is_last_cnt_;  /**< The flag to store if the last poped trajectory is the end of the whole trajectory.*/
    std::mutex mutex_;  /**< Mutex to protect the queue data in multi-thread operation*/

    TBQueue(){}
    
    void pulseOne(uint32_t& data)
    {
        ++data;
        if(data >= max_size_)
        {
            data = data - max_size_;
        }
    }
};

}

#endif
