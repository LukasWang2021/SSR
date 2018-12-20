/*************************************************************************
	> File Name: lock_free_fifo.h
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时56分54秒
 ************************************************************************/

#ifndef _LOCK_FREE_FIFO_H
#define _LOCK_FREE_FIFO_H

#include <atomic>
#include <string.h>

#define LOCK_FREE_FIFO_SIZE_MIN    2
#define LOCK_FREE_FIFO_SIZE_MAX    1024

#define ITEM_FREE   0
#define ITEM_USED   1

template <typename T>
class LockFreeFIFO
{
  public:
    LockFreeFIFO(void)
    {
        capacity_  = 0;
        fifo_head_ = 0;
        fifo_tail_ = 0;
        fifo_ptr_ = NULL;
        flag_ptr_ = NULL;
    }

    ~LockFreeFIFO()
    {
        capacity_  = 0;
        fifo_head_ = 0;
        fifo_tail_ = 0;

        if (fifo_ptr_ != NULL) { delete[] fifo_ptr_; fifo_ptr_ = NULL; }
        if (flag_ptr_ != NULL) { delete[] flag_ptr_; flag_ptr_ = NULL; }
    }

    bool init(size_t capacity)
    {
        if (fifo_ptr_ == NULL && flag_ptr_ == NULL && capacity >= LOCK_FREE_FIFO_SIZE_MIN && capacity <= LOCK_FREE_FIFO_SIZE_MAX)
        {
            fifo_ptr_ = new T[capacity];
            flag_ptr_ = new char[capacity];

            if (fifo_ptr_ != NULL && flag_ptr_ != NULL)
            {
                capacity_ = capacity;
                fifo_head_ = 0;
                fifo_tail_ = 0;
                memset(fifo_ptr_, 0, capacity_ * sizeof(T));
                memset(flag_ptr_, 0, capacity_ * sizeof(char));
                return true;
            }
            else
            {
                if (fifo_ptr_ != NULL) { delete[] fifo_ptr_; flag_ptr_ = NULL; }
                if (flag_ptr_ != NULL) { delete[] flag_ptr_; flag_ptr_ = NULL; }
                return false;
            }
        }
        else
        {
            return false;
        }
    }    

    bool empty(void) const
    {
        return fifo_head_ == fifo_tail_;
    }

    bool full(void) const
    {
        size_t head = fifo_head_;
        size_t tail = fifo_tail_;
        tail = tail + 1 < capacity_ ? tail + 1 : 0;
        return tail == head;
    }
    
    size_t size(void) const
    {
        size_t head = fifo_head_;
        size_t tail = fifo_tail_;
        return tail >= head ? tail - head : tail + capacity_ - head;
    }

    bool push(const T &item)
    {
        size_t target = fifo_tail_;
        size_t next = target + 1 < capacity_ ? target + 1 : 0;

        while (next != fifo_head_ && flag_ptr_[target] == ITEM_FREE)
        {
            if (fifo_tail_.compare_exchange_weak(target, next))
            {
                fifo_ptr_[target] = item;
                flag_ptr_[target] = ITEM_USED;
                return true;
            }

            target = fifo_tail_;
            next = target + 1 < capacity_ ? target + 1 : 0;
        }

        return false;
    }

    bool fetch(T &item)
    {
        size_t target = fifo_head_;
        size_t next = target + 1 < capacity_ ? target + 1 : 0;

        while (target != fifo_tail_ && flag_ptr_[target] == ITEM_USED)
        {
            if (fifo_head_.compare_exchange_weak(target, next))
            {
                item = fifo_ptr_[target];
                flag_ptr_[target] = ITEM_FREE;
                return true;
            }

            target = fifo_head_;
            next = target + 1 < capacity_ ? target + 1 : 0;
        }

        return false;
    }

    void clear(void)
    {
        T item;
        while (fetch(item));
    }

  private:
    T*      fifo_ptr_;
    char*   flag_ptr_;
    size_t  capacity_;
    std::atomic<size_t>  fifo_head_;
    std::atomic<size_t>  fifo_tail_;
};


#endif
