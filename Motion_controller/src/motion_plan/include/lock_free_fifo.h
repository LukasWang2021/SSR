/*************************************************************************
	> File Name: lock_free_fifo.h
	> Author: 
	> Mail: 
	> Created Time: 2018年05月15日 星期二 10时56分54秒
 ************************************************************************/

#ifndef _LOCK_FREE_FIFO_H
#define _LOCK_FREE_FIFO_H

#include <atomic>

namespace fst_controller
{

template <class T>
class LockFreeFIFO
{
  public:
    LockFreeFIFO(size_t capacity)
    {
        capacity_ = capacity;
        fifo_ = new T[capacity_];
        flag_ = new char[capacity_];
        head_ = 0;
        tail_ = 0;

        memset(fifo_, 0, capacity_ * sizeof(T));
        memset(flag_, 0, capacity_ * sizeof(char));
    }

    ~LockFreeFIFO()
    {
        if (fifo_ != NULL)
        {
            delete [] fifo_;
        }
        if (flag_ != NULL)
        {
            delete [] flag_;
        }
    }

    bool empty(void)  {return head_ == tail_;}

    bool full(void)
    {
        size_t head = head_;
        size_t tail = tail_;
        
        tail = tail + 1 < capacity_ ? tail + 1 : 0;
        return tail == head;
    }
    
    size_t size(void)
    {
        size_t head = head_;
        size_t tail = tail_;
        if (tail >= head)   return tail - head;
        else                return tail + capacity_ - head;
    }

    bool push(const T &item)
    {
        size_t target = tail_;
        size_t next = target + 1 < capacity_ ? target + 1 : 0;

        while (next != head_ && flag_[target] == 0)
        {
            if (tail_.compare_exchange_weak(target, next))
            {
                fifo_[target] = item;
                flag_[target] = 1;
                return true;
            }

            target = tail_;
            next = target + 1 < capacity_ ? target + 1 : 0;
        }

        return false;
    }

    bool fetch(T &item)
    {
        size_t target = head_;
        size_t next = target + 1 < capacity_ ? target + 1 : 0;

        while (target != tail_ && flag_[target] == 1)
        {
            if (head_.compare_exchange_weak(target, next))
            {
                item = fifo_[target];
                flag_[target] = 0;
                return true;
            }

            target = head_;
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
    T*      fifo_;
    char*   flag_;
    size_t  capacity_;
    std::atomic<size_t>  head_;
    std::atomic<size_t>  tail_;
};


}


#endif
