/* Copyright 2019 The MathWorks, Inc. */

#ifndef _LABELRANGEDATAGROUNDREMOVALCORE_API_HPP
#define _LABELRANGEDATAGROUNDREMOVALCORE_API_HPP

#include "vision_defines.h"

/* LabelRangeData Ground Removal  */
EXTERN_C LIBMWCVSTRT_API
        void preprocessAndSegmentGround_single(const float* mRange, const int nRows, const int nCols, void* mLabels, 
        const float seed_threshold, const float angle_threshold, const float depth_threshold);

EXTERN_C LIBMWCVSTRT_API
        void preprocessAndSegmentGround_double(const double* mRange, const int nRows, const int nCols, void* mLabels,
        const double seed_threshold, const double angle_threshold, const double depth_threshold);

#endif