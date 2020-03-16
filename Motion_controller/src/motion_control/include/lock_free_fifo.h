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

    bool init(uint32_t capacity)
    {
        if (fifo_ptr_ != NULL) { delete[] fifo_ptr_; fifo_ptr_ = NULL; }
        if (flag_ptr_ != NULL) { delete[] flag_ptr_; flag_ptr_ = NULL; }

        fifo_ptr_ = new T[capacity + 1];
        flag_ptr_ = new char[capacity + 1];

        if (fifo_ptr_ != NULL && flag_ptr_ != NULL)
        {
            capacity_ = capacity;
            fifo_head_ = 0;
            fifo_tail_ = 0;
            memset(fifo_ptr_, 0, (capacity + 1) * sizeof(T));
            memset(flag_ptr_, 0, (capacity + 1) * sizeof(char));
            return true;
        }
        else
        {
            if (fifo_ptr_ != NULL) { delete[] fifo_ptr_; flag_ptr_ = NULL; }
            if (flag_ptr_ != NULL) { delete[] flag_ptr_; flag_ptr_ = NULL; }
            return false;
        }
    }

    bool empty(void) const
    {
        return fifo_head_ == fifo_tail_;
    }

    bool full(void) const
    {
        uint32_t head = fifo_head_;
        uint32_t tail = fifo_tail_;
        tail = tail + 1 < (capacity_ + 1) ? tail + 1 : 0;
        return tail == head;
    }
    
    uint32_t size(void) const
    {
        uint32_t head = fifo_head_;
        uint32_t tail = fifo_tail_;
        return tail >= head ? tail - head : tail + (capacity_ + 1) - head;
    }

    uint32_t capacity(void) const
    {
        return capacity_;
    }

    bool push(const T &item)
    {
        uint32_t target = fifo_tail_;
        uint32_t next = target + 1 < (capacity_ + 1) ? target + 1 : 0;

        while (next != fifo_head_ && flag_ptr_[target] == ITEM_FREE)
        {
            if (fifo_tail_.compare_exchange_weak(target, next))
            {
                fifo_ptr_[target] = item;
                flag_ptr_[target] = ITEM_USED;
                return true;
            }

            target = fifo_tail_;
            next = target + 1 < (capacity_ + 1) ? target + 1 : 0;
        }

        return false;
    }

    bool fetch(T &item)
    {
        uint32_t target = fifo_head_;
        uint32_t next = target + 1 < (capacity_ + 1) ? target + 1 : 0;

        while (target != fifo_tail_ && flag_ptr_[target] == ITEM_USED)
        {
            if (fifo_head_.compare_exchange_weak(target, next))
            {
                item = fifo_ptr_[target];
                flag_ptr_[target] = ITEM_FREE;
                return true;
            }

            target = fifo_head_;
            next = target + 1 < (capacity_ + 1) ? target + 1 : 0;
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
    uint32_t  capacity_;
    std::atomic<uint32_t>  fifo_head_;
    std::atomic<uint32_t>  fifo_tail_;
};


#endif
