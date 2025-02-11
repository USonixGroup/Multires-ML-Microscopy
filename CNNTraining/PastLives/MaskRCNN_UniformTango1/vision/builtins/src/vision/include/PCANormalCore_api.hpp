/* Copyright 2018 The MathWorks, Inc. */

#ifndef _PCANORMALCORE_
#define _PCANORMALCORE_

#include "vision_defines.h"

/* PCA Normal */
EXTERN_C LIBMWCVSTRT_API 
        void PCANormalImpl_single(void* mPoints ,void* mIndices, void* mValid, uint32_T numPoints, uint32_T numNeighbors, void* mNormals);
EXTERN_C LIBMWCVSTRT_API 
        void PCANormalImpl_double(void* mPoints ,void* mIndices, void* mValid,uint32_T numPoints, uint32_T numNeighbors, void* mNormals);
#endif