/*************************************************************************
	> File Name: motion_control_cache_pool.h
	> Author: 
	> Mail: 
	> Created Time: 2018年11月15日 星期四 14时16分22秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_CACHE_POOL_H
#define _MOTION_CONTROL_CACHE_POOL_H

#include <motion_control_datatype.h>

namespace fst_mc
{

template <typename T>
class CachePool
{
  public:
    CachePool(void)
    {
        pool_ptr_ = NULL;
        free_cache_list_ = NULL;
    }

    ~CachePool()
    {
        if (pool_ptr_ != NULL)
        {
            delete[] pool_ptr_;
            pool_ptr_ = NULL;
            free_cache_list_ = NULL;
        }
    }

    ErrorCode initCachePool(size_t capacity)
    {
        if (pool_ptr_ != NULL || capacity < 2 || capacity > 32 || pthread_mutex_init(&mutex_, NULL) != 0)
        {
            return MC_INTERNAL_FAULT;
        }

        pool_ptr_ = new T[capacity];

        if (pool_ptr_ == NULL)
        {
            return MC_INTERNAL_FAULT;
        }

        pthread_mutex_lock(&mutex_);

        for (size_t i = 0; i < capacity; i++)
        {
            pool_ptr_[i].next_ptr = free_cache_list_;
            free_cache_list_ = &pool_ptr_[i];
        }

        pthread_mutex_unlock(&mutex_);
        return SUCCESS;
    }

    T* getCachePtr(void)
    {
        T* pcache = NULL;

        pthread_mutex_lock(&mutex_);

        if (free_cache_list_ != NULL)
        {
            pcache = free_cache_list_;
            free_cache_list_ = free_cache_list_->next_ptr;
        }

        pthread_mutex_unlock(&mutex_);

        if (pcache != NULL)
        {
            memset(pcache, 0, sizeof(T));
        }

        return pcache;
    }

    void freeCachePtr(T *pcache)
    {
        if (pcache != NULL)
        {
            pthread_mutex_lock(&mutex_);
            pcache->next_ptr = free_cache_list_;
            free_cache_list_ = pcache;
            pthread_mutex_unlock(&mutex_);
        }
    }

  private:
    T*  pool_ptr_;
    T*  free_cache_list_;
    pthread_mutex_t  mutex_;
};


}

#endif
