#ifndef THREADSAFEVECTOR_H
#define THREADSAFEVECTOR_H

#include <iostream>
#include <vector>
#include <mutex>
#include <cstdlib>
#include <memory>
#include <iterator>
#include <algorithm>
#include <initializer_list>
#include <functional>
template <class T, class Alloc=std::allocator<T>>
class ThreadSafeVector
{
    private:
        std::vector<T> threadSafeVector;
        std::mutex vectorMutex;

    public:
        /*need to use typename here because std::allocator<T>::size_type, std::allocator<T>::value_type, std::vector<T>::iterator, 
        and std::vector<T>::const_reverse_iterator are 'dependent names' in that because we are working with a templated class, 
        these expressions may depend on types of type template parameters and values of non-template parameters*/

        typedef typename std::vector<T>::size_type size_type;

        typedef typename std::vector<T>::value_type value_type;

        typedef typename std::vector<T>::iterator iterator;

        typedef typename std::vector<T>::const_iterator const_iterator;

        typedef typename std::vector<T>::reverse_iterator reverse_iterator;

        typedef typename std::vector<T>::const_reverse_iterator const_reverse_iterator;

        typedef typename std::vector<T>::reference reference;

        typedef typename std::vector<T>::const_reference const_reference;

        /*wrappers for three different at() functions*/
        template <class InputIterator>
        void assign(InputIterator first, InputIterator last)
        {
            //using a local lock_guard to lock mutex guarantees that the mutex will be unlocked on destruction and in the case of an exception being thrown
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.assign(first, last);
        }

        void assign(size_type n, const value_type& val)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.assign(n, val);
        }

        void assign(std::initializer_list<value_type> il)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.assign(il.begin(), il.end());
        }

        /*wrappers for at() functions*/
        reference at(size_type n)
        {
            return threadSafeVector.at(n);
        }

        const_reference at(size_type n) const
        {
            return threadSafeVector.at(n);
        }   

        /*wrappers for back() functions*/
        reference back()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            return threadSafeVector.back();
        }

        const reference back() const
        {
            return threadSafeVector.back();
        }

        /*wrappers for begin() functions*/
        iterator begin()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            return threadSafeVector.begin();
        }

        const iterator begin() const noexcept
        {
            return threadSafeVector.begin();
        }

        /*wrapper for capacity() fucntion*/
        size_type capacity() const noexcept
        {
            return threadSafeVector.capacity();
        }

        /*wrapper for cbegin() function*/
        const iterator cbegin()
        {
            return threadSafeVector.cbegin();
        }

        /*wrapper for cend() function*/
        const iterator cend()
        {
            return threadSafeVector.cend();
        }

        /*wrapper for clear() function*/
        void clear()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.clear();
        }

        /*wrapper for crbegin() function*/
        const_reverse_iterator crbegin() const noexcept
        {
            return threadSafeVector.crbegin();
        }

        /*wrapper for crend() function*/
        const_reverse_iterator crend() const noexcept
        {
            return threadSafeVector.crend();
        }

        /*wrappers for data() functions*/
        value_type* data()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            return threadSafeVector.data();
        }

        const value_type* data() const noexcept
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            return threadSafeVector.data();
        }

        /*wrapper for emplace() function*/
        template <class... Args>
        void emplace(const iterator position, Args&&... args)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.emplace(position, args...);
        }

        /*wrapper for emplace_back() function*/
        template <class... Args>
        void emplace_back(Args&&... args)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.emplace_back(args...);
        }

        /*wrapper for empty() function*/
        bool empty() const noexcept
        {
            return threadSafeVector.empty();
        }

        /*wrappers for end() functions*/
        iterator end()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            return threadSafeVector.end();
        }

        const iterator end() const noexcept
        {
            return threadSafeVector.end();
        }

        /*wrapper functions for erase()*/
        iterator erase(const_iterator position)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.erase(position);
        }

        iterator erase(const_iterator first, const_iterator last)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.erase(first, last);
        }

        /*wrapper functions for front()*/
        reference front()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            return threadSafeVector.front();
        }

        const reference front() const
        {
            return threadSafeVector.front();
        }

        /*wrapper function for get_allocator()*/
        value_type get_allocator() const noexcept
        {
            return threadSafeVector.get_allocator();
        }

        /*wrapper functions for insert*/
        iterator insert(const_iterator position, const value_type& val)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.insert(position, val); 
        }

        iterator insert(const_iterator position, size_type n, const value_type& val)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.insert(position, n, val);
        }

        template <class InputIterator>
        iterator insert(const_iterator position, InputIterator first, InputIterator last)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.insert(position, first, last);
        }

        iterator insert(const_iterator position, value_type&& val)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.insert(position, val);
        }

        iterator insert(const_iterator position, std::initializer_list<value_type> il)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.insert(position, il.begin(), il.end());
        }

        /*wrapper function for max_size*/
        size_type max_size() const noexcept
        {
            return threadSafeVector.max_size();
        }

        /*wrapper functions for operator =*/
        std::vector<T>& operator= (const std::vector<T>& x)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.swap(x);
        }

        std::vector<T>& operator= (std::vector<T>&& x)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector=std::move(x);
        }

        std::vector<T>& operator= (std::initializer_list<value_type> il)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.assign(il.begin(), il.end());

            return *this; //is this safe to do?
        }

        /*wrapper functions for operator []*/
        reference operator[] (size_type n)
        {
            return std::ref(n);
        }

        const_reference operator[] (size_type n) const
        {
            return std::cref(n);
        }

        /*wrapper function for pop_back()*/
        void pop_back()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.pop_back();
        }

        /*wrapper functions for push_back*/
        void push_back(const value_type& val)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.push_back(val);
        }

        void push_back(value_type&& val)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.push_back(val);
        }

        /*wrapper functions for rbegin()*/
        reverse_iterator rbegin() noexcept
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            return threadSafeVector.rbegin();
        }

        const_reverse_iterator rbegin() const noexcept
        {
            return threadSafeVector.rbegin();
        }

        /*wrapper functions for rend()*/
        reverse_iterator rend() noexcept
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            return threadSafeVector.rend();
        }

        const_reverse_iterator rend() const noexcept
        {
            return threadSafeVector.rend();
        }

        /*wrapper function for reserve()*/
        void reserve(size_type n)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.reserve(n);
        }

        /*wrapper functions for resize()*/      
        void resize(size_type n)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.resize(n);
        }

        void resize(size_type n, const value_type& val)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.resize(n, val);
        }

        void shrink_to_fit()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.shrink_to_fit();
        }

        //add function for size
        size_type size() const noexcept
        {
            return threadSafeVector.size();
        }

        /*wrapper function for swap()*/
        void swap(std::vector<T>& x)
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            threadSafeVector.swap(x);
        }

        void print()
        {
            std::lock_guard<std::mutex> vectorLockGuard(vectorMutex);

            for(const auto & element : threadSafeVector)
            {
                std::cout << element << std::endl;
            }

            std::cout << std::endl;
        }
};

#endif
