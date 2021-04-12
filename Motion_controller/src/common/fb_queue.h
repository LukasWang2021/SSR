#ifndef FB_QUEUE_H
#define FB_QUEUE_H

/**
 * @file fb_queue.h
 * @brief The file includes the template of function block queue.
 * @author zhengyu.shen
 */

#include <stdint.h>
#include <mutex>
#include <string.h>
#include <assert.h>

/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief Defines the status of a function block queue.
 */
typedef enum
{
    FBQ_STATUS_EMPTY = 0,       /**< The function block queue is empty.*/
    FBQ_STATUS_NOT_FULL = 1,    /**< The function block queue has something, but not full.*/
    FBQ_STATUS_FULL = 2,        /**< The function block queue is full.*/
}FBQueueStatus_e;

/**
 * @brief The template class stores function block as an first in first out queue.
 * @details The function block queue can add a function block to its tail and remove one from its head.\n
 *          The operations of the class is thread safe.\n
 */
template<typename T>
class FBQueue
{
public:
    /**
     * @brief Constructor of the class.
     * @param [in] max_size The maximum number of function blocks that is allowed to store in the queue.
     */
    FBQueue(uint32_t max_size)
    {
        max_size_ = max_size;
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        object_ptr_ = new T[max_size_];
        object_size_ = sizeof(T);
    }
    /**
     * @brief Destructor of the class.
     */
    ~FBQueue()
    {
        if(object_ptr_ != NULL)
        {
            delete[] object_ptr_;
            object_ptr_ = NULL;
        }
    }
    /**
     * @brief Remove all function blocks. Set queue size to zero.
     * @return void
     */
    void clear()
    {
        mutex_.lock();
        head_ = 0;
        tail_ = 0;
        size_ = 0;
        mutex_.unlock();
    }
    /**
     * @brief Add one function block to the tail of the queue.
     * @param [in] object_ptr Pointer of the function block to be added.
     * @retval true Add operation success.
     * @retval false Add operation failed because of the queue has already been full.
     */
    bool pushBack(T* object_ptr)
    {
        mutex_.lock();
        if(size_ >= max_size_)
        {
            mutex_.unlock();
            return false;
        }
        memcpy(&object_ptr_[tail_], object_ptr, object_size_);
        pulseN(tail_, 1);
        ++size_;
        mutex_.unlock();
        return true;
    }
    /**
     * @brief remove function blocks from the head of the queue.
     * @param [in] number The number of function blocks to be removed. Default number is 1.
     * @retval true Remove operation success.
     * @retval false Remove operation failed because of the size of the queue is smaller than the number of function block to be removed.
     */
    bool popFront(uint32_t number = 1)
    {
        mutex_.lock();
        if(size_ < number)
        {
            mutex_.unlock();
            return false;
        }
        pulseN(head_, number);
        size_ = size_ - number;
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
     * @brief Operator[] overload.
     * @details Get a reference of the function block by index.\n
                The index starts from 0 and should not be equal or bigger than the current size of the queue.\n
                An assertion fail will arise if the index is out of range.\n
                Index 0 will return the head of the queue.\n
     * @param [in] index The index of the expected function block.
     * @return A reference of the function block.
     */    
    T& operator[](size_t index)
    {
        assert(index < size_);
        uint32_t real_index = head_ + index;
        if(real_index >= max_size_)
        {
            real_index = real_index - max_size_;
        }
        return object_ptr_[real_index];
    }
    const T& operator[](size_t index) const
    {
        assert(index < size_);
        uint32_t real_index = head_ + index;
        if(real_index >= max_size_)
        {
            real_index = real_index - max_size_;
        }
        return object_ptr_[real_index];
    }
    
private:
    uint32_t max_size_;     /**< Max size of the queue.*/
    T* object_ptr_;         /**< Pointer of the queue data.*/
    uint32_t object_size_;  /**< Byte size of an element in the queue.*/
    uint32_t head_;         /**< Index of the head of the queue.*/
    uint32_t tail_;         /**< Index of the tail of the queue.*/
    uint32_t size_;         /**< Current size of the queue.*/
    std::mutex mutex_;      /**< Mutex to protect the queue data in multi-thread operation*/

    FBQueue(){}

    void pulseN(uint32_t& data, uint32_t n)
    {
        data = data + n;
        if(data >= max_size_)
        {
            data = data - max_size_;
        }
    }
};
    
}

#endif


