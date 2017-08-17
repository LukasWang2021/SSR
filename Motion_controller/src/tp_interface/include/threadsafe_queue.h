/**
 * @file threadsafe_queue.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-10-18
 */
#ifndef TP_INTERFACE_THREADSAFE_QUEUE_H_
#define TP_INTERFACE_THREADSAFE_QUEUE_H_
#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>
template<typename T>
class ThreadsafeQueue
{
  public:
     ThreadsafeQueue(){}
     ThreadsafeQueue(ThreadsafeQueue const& other)
     {
         std::lock_guard<std::mutex> lk(other.mut_);
         data_queue_=other.data_queue_;
     }
     void push(T new_value)
     {
         std::lock_guard<std::mutex> lk(mut_);
         data_queue_.push(new_value);
         data_cond_.notify_one();
     }
     void waitAndPop(T& value)
     {
         std::unique_lock<std::mutex> lk(mut_);
         data_cond_.wait(lk,[this]{return !data_queue_.empty();});
         value=data_queue_.front();
         data_queue_.pop();
     }
     std::shared_ptr<T> waitAndPop()
     {
         std::unique_lock<std::mutex> lk(mut_);
         data_cond_.wait(lk,[this]{return !data_queue_.empty();});
         std::shared_ptr<T> res(std::make_shared<T>(data_queue_.front()));
         data_queue_.pop();
         return res;
     }
     bool tryPop(T& value)
     {
         std::lock_guard<std::mutex> lk(mut_);
         if (data_queue_.empty())
             return false;
         value=data_queue_.front();
         data_queue_.pop();
         return true;
     }
     std::shared_ptr<T> tryPop()
     {
         std::lock_guard<std::mutex> lk(mut_);
         if (data_queue_.empty())
             return std::shared_ptr<T>();
         std::shared_ptr<T> res(std::make_shared<T>(data_queue_.front()));
         data_queue_.pop();
         return res;
     }
     bool empty() const
     {
         std::lock_guard<std::mutex> lk(mut_);
         return data_queue_.empty();
     }

  private:
     mutable std::mutex mut_; 
     std::queue<T> data_queue_;
     std::condition_variable data_cond_;
};

#endif
