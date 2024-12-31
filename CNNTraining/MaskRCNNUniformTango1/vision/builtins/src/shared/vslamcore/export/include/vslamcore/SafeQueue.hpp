//////////////////////////////////////////////////////////////////////////////////
// Implementation for thread safe queue.
//
// Copyright 2022-23 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////////

#ifndef SAFE_QUEUE_HPP
#define SAFE_QUEUE_HPP

#include <queue>
#include <deque>
#include <mutex>

namespace vision {
    namespace vslam {
        
        /**
         * @brief This class is a wrapper around the STL queue. All functions 
         *        APIs are similar to the STL queue, with a difference that all
         *        functions are protected using an internal mutex. So, this queue
         *        implementation is thread safe.
         */
        template <typename T>
        class SafeQueue
        {
        public:
            bool empty() {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                return internalQueue.empty();
            }
            
            void clear() {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                while(!internalQueue.empty()){
                    internalQueue.pop();
                }
            }

            size_t size() {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                return internalQueue.size();
            }
            
            decltype(auto) front() {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                return internalQueue.front();
            }
            
            decltype(auto) back() {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                return internalQueue.back();
            }
                
            void push(const T& _Val) {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                internalQueue.push(_Val);
            }
        
            void push(T&& _Val) {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                internalQueue.push(std::move(_Val));
            }
            
            template <class... TARGS>
            void emplace(TARGS&&... _Val) {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                internalQueue.emplace(std::forward<TARGS>(_Val)...);
            }
        
            void pop() {
                std::unique_lock<std::recursive_mutex> lock(this->internalMtx);
                internalQueue.pop();;
            }
        
            void lock() {
                internalMtx.lock();
            }
        
            void unlock() {
                internalMtx.unlock();
            }
        
            bool tryLock() {
                return internalMtx.try_lock();
            }
            
        private:
            std::recursive_mutex internalMtx;
            std::queue<T> internalQueue;
        };

    }// namespace vslam
}// namespace vision

#endif // SAFE_QUEUE_HPP
