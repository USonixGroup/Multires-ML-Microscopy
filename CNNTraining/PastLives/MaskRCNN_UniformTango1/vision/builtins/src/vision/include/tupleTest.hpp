////////////////////////////////////////////////////////////////////////
// This file contains the header for built-in tupleTest function.
// Copyright 2020 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////

#ifndef TUPLETEST_HPP
#define TUPLETEST_HPP

#include "vision_defines.h"

#ifdef COMPILE_FOR_VISION_BUILTINS
    #include "mex.h"
    #include <fl/except/MsgIDException.hpp>
    #include <resources/vision/visionlib.hpp>
    #include <resources/vision/ocvShared.hpp>
    #include "matrix/unique_mxarray_ptr.hpp"
    // Find matched indices of input points for simulation
    template<typename T>
    void findMatches(boolean_T *matchedInds, const T* points1, const T* points2,
            const uint32_T* randInds, uint32_T numPoints, uint32_T pointsDim, 
            T scale, uint32_T numIters);
#endif

// Find matched indices of input points of type double
EXTERN_C LIBMWCVSTRT_API void findMatchesDouble(boolean_T *matchedInds, 
        const double* points1, const double* points2, const uint32_T* randInds, 
        uint32_T numPoints, uint32_T pointsDim, double scale, uint32_T numIters);

// Find matched indices of input points of type single
EXTERN_C LIBMWCVSTRT_API void findMatchesSingle(boolean_T *matchedInds, 
        const float* points1, const float* points2, const uint32_T* randInds, 
        uint32_T numPoints, uint32_T pointsDim, float scale, uint32_T numIters);

#endif
