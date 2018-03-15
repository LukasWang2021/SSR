/**********************************************************************
  Copyright:   Foresight-Robotics
  File:        LimitedFifo.h
  Author:      Yan He
  Data:        Dec.1  2016
  Modify:      Dec.23 2016
  Description: data_monitor--Class declaration.
**********************************************************************/


#ifndef LIMITED_FIFO_H
#define LIMITED_FIFO_H

#include <atomic>
#include <string.h>

namespace fst_controller {
// Brief class for controller. This class include many default settings and functions to make life easier.

template<typename T >
class LimitedFifo
{
  // -----------------------------public functions---------------------------------------------

  public:
    //------------------------------------------------------------
    // Function:    LimitedFifo
    // Summary: The constructor of class
    // In:      ip_address: ip address; port: port number; 
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    LimitedFifo(int size);


    //------------------------------------------------------------
    // Function:    ~LimitedFifo
    // Summary: The destructor of class
    // In:      None
    // Out:     None
    // Return:  None
    //------------------------------------------------------------
    ~LimitedFifo();
    //------------------------------------------------------------
    // Function:    push_item
    // Summary: push item to the fifo
    // In:      x: the item  force: if it is true remove the oldest if fifo is full
    // Out:     None
    // Return:  >0 OK  <0 err
    //------------------------------------------------------------    
    int push_item(const T &x,bool force,int limit_size = 0);
    
    //------------------------------------------------------------
    // Function:    fetch_item
    // Summary: fetch item from the fifo
    // In:      nonel
    // Out:     x: the item fetched
    // Return:  >0 OK  <0 err
    //------------------------------------------------------------        
    int fetch_item(T &x);
    //------------------------------------------------------------
    // Function:    fetch_batch
    // Summary: fetch batch of items from the fifo
    // In:      size: batch buffer size
    // Out:     x: the item fetched
    // Return:  >0 number of item fetched <0 err
    //------------------------------------------------------------      
    int fetch_batch(T *batch,int size);
    //------------------------------------------------------------
    // Function:    size
    // Summary: get size of fifo
    // In:     none
    // Out:     none
    // Return:  size
    //------------------------------------------------------------          
    int size(void);
    
    //------------------------------------------------------------
    // Function:    lock_push
    // Summary: lock the push action, so the push will failed
    // In:      none
    // Out:     none
    // Return:  void
    //------------------------------------------------------------          
    void lock_push(void);
    //------------------------------------------------------------
    // Function:    unlock_push
    // Summary: unlock the push action
    // In:      none
    // Out:     none
    // Return:  void
    //------------------------------------------------------------              
    void unlock_push(void);

    //------------------------------------------------------------
    // Function:    is_push_locked
    // Summary: return if push is locked
    // In:      none
    // Out:     none
    // Return:  true: push is locked; false:push is allowed
    //------------------------------------------------------------              
    bool is_push_locked(void);

  private:
    bool try_lock(std::atomic<int>& lock);

      
    void unlock(std::atomic<int>& lock);

    //------------------------------------------------------------
    // Function:    get_numof_item
    // Summary: get number of items by in index and out index
    // In:      l_in_index: in index;l_out_index: out index
    // Out:     none
    // Return:  number of items
    //------------------------------------------------------------          
    int get_numof_item(int l_in_index,int l_out_index);
    //------------------------------------------------------------
    // Function:    increase_index
    // Summary: change index to next one
    // In:      index: index
    // Out:     none
    // Return:  next index
    //------------------------------------------------------------              
      
    int increase_index(int index);

  private:

    // -----------------------------member variables---------------------------------------------

    std::atomic<int> in_index_; 
    std::atomic<int> out_index_;
    std::atomic<int> forcepush_lock_;


    T *item_buf_;
    int *buf_flag_;
    int capacity_;
};  // class LimitedFifo


template<typename T >
LimitedFifo<T>::LimitedFifo(int size)
{
    item_buf_ = NULL;
    buf_flag_ = NULL;
    capacity_ = 0;
    in_index_ = 0; 
    out_index_ = 0;
    forcepush_lock_ = 0;
    item_buf_ = new T[size+1];
    if(NULL != item_buf_)
    {
        buf_flag_ = new int[size+1];
        if(NULL == buf_flag_)
        {
            delete [] item_buf_;
        }
        else
        {
            capacity_ = size;
            memset((char*)buf_flag_,0,(size+1)*sizeof(buf_flag_));
        }
    }
}
template<typename T >
LimitedFifo<T>::~LimitedFifo()
{
    if(NULL != item_buf_)
    {
        delete [] item_buf_;
    }   
    if(NULL != buf_flag_)
    {
        delete [] buf_flag_;
    }
}
template<typename T >
bool LimitedFifo<T>::try_lock(std::atomic<int>& lock)
{
    int exp = 0;
    if(true == lock.compare_exchange_weak(exp,1))
    {
        return true;
    }
    return false;
}
template<typename T >
void LimitedFifo<T>::unlock(std::atomic<int>& lock)
{
    lock = 0;
}

template<typename T >
int LimitedFifo<T>::get_numof_item(int l_in_index,int l_out_index)
{
    int num = 0;
    if (l_in_index >= l_out_index)
    {
        num = l_in_index - l_out_index;
    }
    else  //in_index_ < out_index_
    {
        num = (l_in_index - 0) + (capacity_ + 1 - l_out_index);
    }
    return num;
}
template<typename T >
int LimitedFifo<T>::increase_index(int index)
{
    if(capacity_ == index)
    {
        index = 0;
    }
    else
    {
        ++index;
    }
    return index;
}

template<typename T >
int LimitedFifo<T>::push_item(const T &x,bool force,int limit_size)
{
    int target;
    int temp_out;
    int next;
    int res = 0;
    int cap = capacity_;
    if((limit_size>0)&&(limit_size<capacity_))
    {
        cap = limit_size;
    }
    if(0 == cap) return -2;
    if(!try_lock(forcepush_lock_)) return -1;
    while(1)
    {
        target = in_index_;
        temp_out = out_index_;
        if(get_numof_item(target,temp_out)>=cap)
        {
            if(true == force)
            {
                next = increase_index(temp_out);   
                if(true == out_index_.compare_exchange_weak(temp_out,next))
                    buf_flag_[temp_out] = 0;
                res = 1;
            } 
            else
            {
                res = -1;
                break;
            }
        }
        else
        {
            if(0==buf_flag_[target])
            {
                next = increase_index(target);     
                
                if(true == in_index_.compare_exchange_weak(target,next))
                {
                    item_buf_[target] = x;
                    buf_flag_[target] = 1;
                    break;
                }   
                
            }
            else
            {
                printf("wrong buf flag: %d\n\n",target);
                res = -1;
                break;
            }
        }
    }
    unlock(forcepush_lock_);
    return res;
}
template<typename T >
int LimitedFifo<T>::fetch_item(T &x)
{
    int target;
    int next;
    if(0 == capacity_) return -2;
    target = out_index_;
	while((get_numof_item(in_index_,target)>0)&&\
        (buf_flag_[target]>0))
	{
        next = increase_index(target);
        if(true == out_index_.compare_exchange_weak(target,next))
        {
            x = item_buf_[target];
            buf_flag_[target] = 0;
            return 1;
        }

        target = out_index_;
	}
	return -1;
}
template<typename T >
int LimitedFifo<T>::fetch_batch(T* batch,int size)
{
    int target;
    int next;
    int index;
    if(0 == capacity_) return -2;
    for(index = 0;index<size;)
    {
        target = out_index_;
    	if(get_numof_item(in_index_,target)>0)
    	{
            if(buf_flag_[target]>0)
            {
                next = increase_index(target);

                if(true == out_index_.compare_exchange_weak(target,next))
                {
                    batch[index] = item_buf_[target];
                    buf_flag_[target] = 0;
                    index ++;
                }
            }
    	}
        else
        {
            break;
        }
    }
	return index;
}
template<typename T >
int LimitedFifo<T>::size(void)
{
    return get_numof_item(in_index_,out_index_);
}
template<typename T >
void LimitedFifo<T>::lock_push(void)
{
    while(try_lock(forcepush_lock_));
}
template<typename T >
void LimitedFifo<T>::unlock_push(void)
{
    unlock(forcepush_lock_);
}

template<typename T >
bool LimitedFifo<T>::is_push_locked(void)
{
    if(forcepush_lock_.load()>0)
    {
        return true;
    }
    return false;
}

}   // namespace fst_controller


#endif  // #ifndef LIMITED_FIFO_H


