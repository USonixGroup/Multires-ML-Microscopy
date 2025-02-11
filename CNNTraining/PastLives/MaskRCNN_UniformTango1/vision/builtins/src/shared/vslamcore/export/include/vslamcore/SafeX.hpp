////////////////////////////////////////////////////////////////////////////////
//  SafeX.hpp
//
//  SafeX class header file. This class will be used to convert any data 
//  type into a thread protected type.
//
//  Copyright 2022-23 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#include <mutex>

#ifndef SAFEX_HPP
#define SAFEX_HPP

namespace vision {
    namespace vslam {

        template <typename T>
        class SafeX
        {
            T val;
            std::recursive_mutex valMtx;
            
            public:
            operator T() { 
                std::unique_lock<std::recursive_mutex> lock(this->valMtx);
                return this->val;
            }

            T& operator=(const T &a) {
                std::unique_lock<std::recursive_mutex> lock(this->valMtx);
                this->val = a;
                return this->val; 
            }
        };
    }
}

#endif