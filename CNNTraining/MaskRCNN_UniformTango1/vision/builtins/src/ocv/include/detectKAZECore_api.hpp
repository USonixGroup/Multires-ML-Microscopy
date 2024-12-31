/* Copyright 2017 The MathWorks, Inc. */

#ifndef _DETECTKAZE_
#define _DETECTKAZE_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API int32_T detectKAZEComputeCM(uint8_T *inImg,
	int32_T nRows, int32_T nCols, 
    bool extended, bool upright,
    float threshold, 
    int numOctaves,
    int numScaleLevels, 
    int diffusivity,
	void **outKeypoints);

EXTERN_C LIBMWCVSTRT_API int32_T detectKAZEComputeRM(uint8_T *inImg,
	int32_T nRows, int32_T nCols, 
    bool extended, bool upright,
    float threshold, 
    int numOctaves,
    int numScaleLevels, 
    int diffusivity,
	void **outKeypoints);

EXTERN_C LIBMWCVSTRT_API void detectKAZEAssignOutputCM(
        void *ptrKeypoints,
        real32_T *outLoc, 
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl,
        uint8_T  *outMis);

EXTERN_C LIBMWCVSTRT_API void detectKAZEAssignOutputRM(
        void *ptrKeypoints,
        real32_T *outLoc, 
        real32_T *outOri,
        real32_T *outMet,
        real32_T *outScl,
        uint8_T  *outMis);
#endif
