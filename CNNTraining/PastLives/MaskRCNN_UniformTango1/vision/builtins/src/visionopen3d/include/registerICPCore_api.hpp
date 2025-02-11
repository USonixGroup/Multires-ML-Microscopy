/* Copyright 2023 The MathWorks, Inc. */

#ifndef _REGISTERICPCORE_API_HPP
#define _REGISTERICPCORE_API_HPP

#include "vision_defines.h"
#include <stddef.h> // Scope for size_t for portable codegen

/* pcregistericp */

EXTERN_C LIBMWCVSTRT_API
double registerICP_single(void* mLocationsA, void* mLocationsB,
                          boolean_T hasNormalsA, void* mNormalsA,
                          boolean_T hasNormalsB, void* mNormalsB,
                          boolean_T useColors, void* mPtCloudAColors,
                          void* mPtCloudBColors, void* mVoxelSizes,
                          void* mMaxIterationVec, void* mInitTform, 
                          void* mDataTypeTform,
                          const int maxIteration,
                          void* mMethod,
                          const double relativeRotation,
                          const double relativeTranslation,
                          const double metric,
                          const size_t numPointsA,
                          const size_t numPointsB,
                          const boolean_T useInlierRatio,
                          void* mOutTransform);

EXTERN_C LIBMWCVSTRT_API
double registerICP_double(void* mLocationsA, void* mLocationsB,
                          const boolean_T hasNormalsA, void* mNormalsA,
                          const boolean_T hasNormalsB, void* mNormalsB,
                          boolean_T useColors, void* mPtCloudAColors,
                          void* mPtCloudBColors, void* mVoxelSizes,
                          void* mMaxIterationVec, void* mInitTform, 
                          void* mDataTypeTform,
                          const int maxIteration,
                          void* mMethod,
                          const double relativeRotation,
                          const double relativeTranslation,
                          const double metric,
                          const size_t numPointsA,
                          const size_t numPointsB,
                          const boolean_T useInlierRatio,
                          void* mOutTransform);

#endif