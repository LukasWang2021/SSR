/**
 * @file threadsafe_list.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-02-08
 */
#ifndef THREADSAFELIST_H
#define THREADSAFELIST_H

#include <iostream>
#include <list>
#include <mutex>
#include <cstdlib>
#include <memory>
#include <iterator>
#include <algorithm>
#include <initializer_list>
#include <functional>
template <class T, class Alloc=std::allocator<T>>
class ThreadSafeList
{
    private:
        std::list<T> threadSafeList;
        std::mutex list_mutex_;
    public:
        /*need to use typename here because std::allocator<T>::size_type, std::allocator<T>::value_type, std::list<T>::iterator, 
        and std::list<T>::const_reverse_iterator are 'dependent names' in that because we are working with a templated class, 
        these expressions may depend on types of type template parameters and values of non-template parameters*/

        typedef typename std::list<T>::size_type size_type;

        typedef typename std::list<T>::value_type value_type;

        typedef typename std::list<T>::iterator iterator;

        typedef typename std::list<T>::const_iterator const_iterator;

        typedef typename std::list<T>::reverse_iterator reverse_iterator;

        typedef typename std::list<T>::const_reverse_iterator const_reverse_iterator;

        typedef typename std::list<T>::reference reference;

        typedef typename std::list<T>::const_reference const_reference;

        /*wrappers for three different at() functions*/
        template <class InputIterator>
        void assign(InputIterator first, InputIterator last)
        {
            //using a local lock_guard to lock mutex guarantees that the mutex will be unlocked on destruction and in the case of an exception being thrown
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.assign(first, last);
        }

        void assign(size_type n, const value_type& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.assign(n, val);
        }

        void assign(std::initializer_list<value_type> il)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.assign(il.begin(), il.end());
        }

        /*wrappers for back() functions*/
        reference back()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            return threadSafeList.back();
        }

        const reference back() const
        {
            return threadSafeList.back();
        }

        /*wrappers for begin() functions*/
        iterator begin()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            return threadSafeList.begin();
        }

        const iterator begin() const noexcept
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.begin();
        }


        /*wrapper for cbegin() function*/
        const iterator cbegin()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.cbegin();
        }

        /*wrapper for cend() function*/
        const iterator cend()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.cend();
        }

        /*wrapper for clear() function*/
        void clear()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.clear();
        }

        /*wrapper for crbegin() function*/
        const_reverse_iterator crbegin() const noexcept
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.crbegin();
        }

        /*wrapper for crend() function*/
        const_reverse_iterator crend() const noexcept
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.crend();
        }


        /*wrapper for emplace() function*/
        template <class... Args>
        void emplace(const iterator position, Args&&... args)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.emplace(position, args...);
        }		

        /*wrapper for emplace_back() function*/
        template <class... Args>
        void emplace_back(Args&&... args)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.emplace_back(args...);
        }
		
		/*wrapper for emplace_front() function*/
        template <class... Args>
        void emplace_front(const iterator position, Args&&... args)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.emplace_front(position, args...);
        }

        /*wrapper for empty() function*/
        bool empty() const noexcept
        {
            return threadSafeList.empty();
        }

        /*wrappers for end() functions*/
        iterator end()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            return threadSafeList.end();
        }

        const iterator end() const noexcept
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.end();
        }

        /*wrapper functions for erase()*/
        iterator erase(iterator position)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.erase(position);
        }

        iterator erase(iterator first, iterator last)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.erase(first, last);
        }

        /*wrapper functions for front()*/
        reference front()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            return threadSafeList.front();
        }

        const reference front() const
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.front();
        }

        /*wrapper function for get_allocator()*/
        value_type get_allocator() const noexcept
        {
            return threadSafeList.get_allocator();
        }

        /*wrapper functions for insert*/
        iterator insert(iterator position, const value_type& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.insert(position, val); 
        }

        iterator insert(iterator position, size_type n, const value_type& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.insert(position, n, val);
        }

        template <class InputIterator>
        iterator insert(iterator position, InputIterator first, InputIterator last)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.insert(position, first, last);
        }

        iterator insert(iterator position, value_type&& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.insert(position, val);
        }

        iterator insert(iterator position, std::initializer_list<value_type> il)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.insert(position, il.begin(), il.end());
        }

        /*wrapper function for max_size*/
        size_type max_size() const noexcept
        {
            return threadSafeList.max_size();
        }

        /*wrapper functions for operator =*/
        std::list<T>& operator= (const std::list<T>& x)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.swap(x);
        }

        std::list<T>& operator= (std::list<T>&& x)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList=std::move(x);
        }

        std::list<T>& operator= (std::initializer_list<value_type> il)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.assign(il.begin(), il.end());

            return *this; //is this safe to do?
        }

        /*wrapper functions for operator []*/
        reference operator[] (size_type n)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return std::ref(n);
        }

        const_reference operator[] (size_type n) const
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return std::cref(n);
        }
		
		/*wrapper function for merge()*/
		void merge (std::list<T>& x)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.merge(x);
		}
		template <class Compare>
		void merge (std::list<T>& x, Compare comp)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			merge(x, comp);
		}

        /*wrapper function for pop_back()*/
        void pop_back()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.pop_back();
        }
		
		/*wrapper function for pop_back()*/
        void pop_front()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.pop_front();
        }

        /*wrapper functions for push_back*/
        void push_back(const value_type& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.push_back(val);
        }

        void push_back(value_type&& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.push_back(val);
        }
		
		/*wrapper functions for push_front*/
        void push_front(const value_type& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.push_front(val);
        }
		
		void push_front(value_type&& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.push_front(val);
        }

        /*wrapper functions for rbegin()*/
        reverse_iterator rbegin() noexcept
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            return threadSafeList.rbegin();
        }

        const_reverse_iterator rbegin() const noexcept
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.rbegin();
        }

        /*wrapper functions for rend()*/
        reverse_iterator rend() noexcept
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            return threadSafeList.rend();
        }		
        const_reverse_iterator rend() const noexcept
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.rend();
        }
		
		/*wrapper functions for remove()*/
		void remove (const value_type& val)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.remove(val);
		}
		
		/*wrapper functions for remove_if()*/
		template <class Predicate>
		void remove_if (Predicate pred)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.remove_if(pred);
		}

        /*wrapper functions for resize()*/      
        void resize(size_type n)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.resize(n);
        }

        void resize(size_type n, const value_type& val)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.resize(n, val);
        }


        //add function for size
        size_type size() const //noexcept
        {
            //std::lock_guard<std::mutex> listLockGuard(list_mutex_);
            return threadSafeList.size();
        }
		
		/*wrapper function for sort()*/
		void sort()
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.sort();
		}
		template <class Compare>
		void sort (Compare comp)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.sort(comp);
		}
		
		/*wrapper function for splice()*/
		void splice (iterator position, std::list<T>& x)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.splice(position, x);
		}
	
		void splice (iterator position, std::list<T>& x, iterator i)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.splice(position, x, i);
		}

		void splice (iterator position, std::list<T>& x, iterator first, iterator last)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.splice(position, x, first, last);
		}

        /*wrapper function for swap()*/
        void swap(std::list<T>& x)
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            threadSafeList.swap(x);
        }
		
		/*wrapper function for unique()*/
		void unique()
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.unique();
		}
		template <class BinaryPredicate>
		void unique (BinaryPredicate binary_pred)
		{
			std::lock_guard<std::mutex> listLockGuard(list_mutex_);
			threadSafeList.unique(binary_pred);
		}

        void print()
        {
            std::lock_guard<std::mutex> listLockGuard(list_mutex_);

            for(const auto & element : threadSafeList)
            {
                std::cout << element << std::endl;
            }

            std::cout << std::endl;
        }
};

#endif

