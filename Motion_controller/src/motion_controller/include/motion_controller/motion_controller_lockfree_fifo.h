/*************************************************************************
	> File Name: motion_controller_lockfree_fifo.h
	> Author: 
	> Mail: 
	> Created Time: 2017年11月16日 星期四 10时21分19秒
 ************************************************************************/

#ifndef _MOTION_CONTROLLER_LOCKFREE_FIFO_H
#define _MOTION_CONTROLLER_LOCKFREE_FIFO_H

#include <atomic>
#include <string.h>

namespace fst_controller {

template<typename T> class LockFreeFIFO {
  public:
    LockFreeFIFO()
    {
        flags_      = NULL;
        items_      = NULL;
        index_in_   = 0;
        index_out_  = 0;
        capacity_   = 0;
    }

    LockFreeFIFO(size_t size)
    {
        flags_      = NULL;
        items_      = NULL;
        index_in_   = 0;
        index_out_  = 0;
        capacity_   = 0;

        initLockFreeFIFO(size);
    }

    bool initLockFreeFIFO(size_t size)
    {
        if (capacity_ == 0) {
            if (size > 0) {
                flags_  = new bool[size];
                items_  = new T[size];

                if (flags_ != NULL && items_ != NULL) {
                    index_in_   = 0;
                    index_out_  = 0;
                    capacity_   = size;

                    memset((char*)items_, 0, capacity_ * sizeof(T));
                    memset((char*)flags_, 0, capacity_ * sizeof(bool));

                    return true;
                }
                else {
                    if (flags_ != NULL) delete [] flags_;
                    if (items_ != NULL) delete [] items_;

                    index_in_   = 0;
                    index_out_  = 0;
                    capacity_   = 0;

                    return false;
                }
            }    
            else {
                index_in_   = 0;
                index_out_  = 0;
                capacity_   = 0;

                return false;
            }
        }
        else {
            return false;
        }
    }


    ~LockFreeFIFO()
    {
        if (flags_ != NULL) delete [] flags_;
        if (items_ != NULL) delete [] items_;
    }

    bool pushItem(const T &x)
    {
        if (size() < capacity_) {
            size_t target, next;

            while (true) {
                target = index_in_;

                if (flags_[target] == false) {
                    next   = target + 1;
                    if (next == capacity_) next = 0;

                    if (index_in_.compare_exchange_weak(target, next)) {
                        items_[target] = x;
                        flags_[target] = true;
                        return true;
                    }
                }
                else {
                    // FIFO is full
                    return false;
                }
            }
        }
        else {
            return false;
        }
    }

    bool fetchItem(T &x)
    {
        if (size() > 0) {
            size_t target, next;

            while (true) {
                target = index_out_;

                if (flags_[target] == true) {
                    next   = target + 1;
                    if (next == capacity_) next = 0;

                    if (index_out_.compare_exchange_weak(target, next)) {
                        x = items_[target];
                        flags_[target] = false;
                        return true;
                    }
                }
                else {
                    // FIFO is empty
                    return false;
                }
            }
        }
        else {
            return false;
        }
    }

    size_t size(void)
    {
        size_t in   = index_in_;
        size_t out  = index_out_;

        if (in > out) {
            return in - out;
        }
        else if (in < out) {
            return in + capacity_ - out;
        }
        else {
            if (capacity_ > 0 && flags_[index_out_]) 
                return capacity_;
            else
                return 0;
        }
    }

    size_t capacity(void)
    {
        return capacity_;
    }

  private:
    bool    *flags_;
    T       *items_;
    size_t  capacity_;

    std::atomic<size_t> index_in_;
    std::atomic<size_t> index_out_;
};


}
#endif
