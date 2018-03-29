/**
 * @file lockfree_queue.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-07-26
 */


#ifndef RCS_LOCKFREE_QUEUE_H_  
#define RCS_LOCKFREE_QUEUE_H_  
  

#include <atomic>
#include <map>
#include<type_traits>


namespace  rcs
{  
    
    template <typename T>
    class LFQueue
    {
        struct Node
        {
            T value;
            std::atomic<Node*> next;
            Node(T value): value(value), next(nullptr) { }
            Node(): next(nullptr) { }
        };

    public:
        LFQueue()
        {
            head_.store(new Node( T() ));
            tail_ = head_.load();       
        }
        ~LFQueue()
        {
            while(head_ != nullptr)
            {
                Node* p = head_.load();
                head_.store(p->next);
                delete p;
            }
        }

    public:
        void push(T const &val)
        {
            Node* node = new Node(val);
            while(true)
            {
                Node* last = tail_.load();
                Node* next = last->next.load();
                if(last == tail_.load())
                {
                    if(next == nullptr)
                    {
                        if(std::atomic_compare_exchange_weak(&(last->next), &next, node))
                        {
                            std::atomic_compare_exchange_weak(&tail_, &last, node);
                            return;
                        }
                    }
                    else
                    {
                        std::atomic_compare_exchange_weak(&tail_, &last, next);
                    }
                }
            }     
        }


        bool pop(T &res)
        {
            while(true)
            {
                Node* first = head_.load();
                Node* last = tail_.load();
                Node* next = first->next.load();
                if(first == head_.load())
                {
                    if(first == last)
                    {
                        if(next == nullptr)
                        {
                            return false;
                        }
                        std::atomic_compare_exchange_weak(&tail_,&last,next);
                    }
                    else
                    {
                        T value = next->value;
                        if(std::atomic_compare_exchange_weak(&head_,&first,next))
                        {
                            res = value;
                            break;
                        }
                    }
                }
            }       
            return true;
        }

    private:
        std::atomic<Node*> head_;
        std::atomic<Node*> tail_;    
    };
}


#endif //RCS_LOCKFREE_QUEUE_H_  
