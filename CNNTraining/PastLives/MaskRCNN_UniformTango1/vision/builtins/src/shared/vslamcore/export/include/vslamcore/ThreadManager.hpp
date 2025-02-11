//////////////////////////////////////////////////////////////////////////////////
// Implementation for thread safe queue.
//
// Copyright 2022 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////////

#ifndef THREAD_MANAGER_HPP
#define THREAD_MANAGER_HPP

#include <thread>
#include <vector>

namespace vision {
    namespace vslam {
        
        class ThreadManager {
        public:

            /**
             * @brief Function to create a thread using the provided function 
             *        and its arguments.
             *
             * @param[in] threadCount Number of threads to be created on the provided function
             * @param[in] fn Function reference
             * @param[in] args Variadic arguments for the function reference
             *
             * @return Return the index at which the thread is stored in the internal vector
             */
	        template <class Fn, class... Args>
	        int createThread(const int threadCount, Fn&& fn, Args&&... args) {
		        // For now threadCount=1
		        allThreads.emplace_back(std::forward<Fn>(fn), std::forward<Args>(args)...);
		        return static_cast<int>(allThreads.size()) - 1;
	        }
        
            /**
             * @brief Function to join a thread in the running thread.
             *
             * @param[in] idx Thread index in the internal vector
             *
             * @return Return true if thread index is valid otherwise false
             */
	        bool joinThread(const int idx) {
		        if (idx >= 0 && idx < static_cast<int>(allThreads.size())) {
			        allThreads[idx].join();
			        return true;
		        }
		        return false;
	        }

            bool detachThread(const int idx) {
		        if (idx >= 0 && idx < static_cast<int>(allThreads.size())) {
			        allThreads[idx].detach();
			        return true;
		        }
		        return false;
	        }

            bool empty() {
		        return allThreads.empty();
	        }

            /**
             * @brief destructor
             */
            ~ThreadManager(){
                allThreads.clear();
            }
        private:
	        std::vector<std::thread> allThreads;
        };

    }// namespace vslam
}// namespace vision

#endif // THREAD_MANAGER_HPP
