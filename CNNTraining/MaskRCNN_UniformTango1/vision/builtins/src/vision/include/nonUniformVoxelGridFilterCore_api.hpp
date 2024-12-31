/* Copyright 2018 The MathWorks, Inc. */

#ifndef _NONUNIFORMVOXELGRIDFILTERCORE_
#define _NONUNIFORMVOXELGRIDFILTERCORE_

#include "vision_defines.h"

/* Non-Uniform VoxelGridFilter */
EXTERN_C LIBMWCVSTRT_API
        uint32_T nonUniformVoxelGridFilterImpl_single(void** filterObj,
        const float* pData,
        uint32_T numPoints,
        double maxVoxelPoints,
        bool needNormals);
EXTERN_C LIBMWCVSTRT_API
        uint32_T nonUniformVoxelGridFilterImpl_double(void** filterObj,
        double* pData,
        uint32_T numPoints,
        double maxVoxelPoints,
        bool needNormals);
EXTERN_C LIBMWCVSTRT_API
        void nonUniformVoxelGridFilterImplAssignOutputs_single(void* filterObj,
        uint32_T* pIndices,
        float* pNormal,
        bool needNormals);
EXTERN_C LIBMWCVSTRT_API
        void nonUniformVoxelGridFilterImplAssignOutputs_double(void* filterObj,
        uint32_T* pIndices,
        double* pNormal,
        bool needNormals);
#endif