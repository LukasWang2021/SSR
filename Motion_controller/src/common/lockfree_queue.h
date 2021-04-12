#ifndef LOCKFREE_QUEUE_H_  
#define LOCKFREE_QUEUE_H_  
  
/**
 * @file lockfree_queue.h
 * @brief The file includes the definition of lock free queue.
 * @author zhengyu.shen
 */

#include <atomic>
#include <map>
#include <type_traits>
#include <iostream>
/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{      
/**
 * @brief LockFreeQueue is a template class to realize a lock free queue.
 * @details The lock free queue is an first in first out queue. It is thread safe in use.
 */
template <typename T>
class LockFreeQueue
{
private:
    /*
     * @brief Element of the queue.
     */
    struct Node
    {
        T value;    /**< Element data*/
        std::atomic<Node*> next;    /**< Pointer points to the next element node.*/
        /**
         * @brief Constructor of the the element of the queue.
         * @param [in] value Element data.
         */
        Node(T value): value(value), next(nullptr){}
        /**
         * @brief Default constructor of the element of the queue.
         */
        Node(): next(nullptr){}
    };
public:
    /**
     * @brief Constructor of the class.
     */
    LockFreeQueue()
    {
        head_.store(new Node( T() ));
        tail_ = head_.load();       
    }
    /**
     * @brief Deconstructor of the class.
     */
    ~LockFreeQueue()
    {
        while(head_ != nullptr)
        {
            Node* p = head_.load();
            head_.store(p->next);
            delete p;
        }
    }
    /**
     * @brief Push an object to the tail of the queue.
     * @param [in] val The object to be added into the queue. 
     * @return void
     */    
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
    /**
     * @brief Pop an object from the head of the queue.
     * @param [out] res The object to be poped out of the queue. 
     * @retval true Pop operation succeed. 'res' is valid to use.
     * @retval false Pop operation failed. 'res' is not valid to use.
     */ 
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
        std::atomic<Node*> head_;   /**< Head pointer of the queue.*/
        std::atomic<Node*> tail_;   /**< Tail pointer of the queue.*/
    };
}

#endif

