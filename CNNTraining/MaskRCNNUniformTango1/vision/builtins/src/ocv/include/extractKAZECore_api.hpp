/* Copyright 2017 The MathWorks, Inc. */

#ifndef _EXTRACTKAZE_
#define _EXTRACTKAZE_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API int32_T extractKAZEComputeCM(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inOri,
    real32_T *inMet,
    real32_T *inScl,
    uint8_T  *inMis,
	int numKPts,
    bool extended, bool upright, float threshold,
    int numOctaves, int numScaleLevels, int diffusivity,
	void **outKeypoints, void **outFeatures);

EXTERN_C LIBMWCVSTRT_API int32_T extractKAZEComputeRM(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inOri,
    real32_T *inMet,
    real32_T *inScl,
    uint8_T  *inMis,
	int numKPts,
    bool extended, bool upright, float threshold,
    int numOctaves, int numScaleLevels, int diffusivity,
	void **outKeypoints, void **outFeatures);

EXTERN_C LIBMWCVSTRT_API void extractKAZEAssignOutputCM(
    void * ptrKeypoints, 
    void * ptrFeatures,
    real32_T *outLoc, 
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl,
    uint8_T  *outMis,
    real32_T *outFtrs);

EXTERN_C LIBMWCVSTRT_API void extractKAZEAssignOutputRM(
    void * ptrKeypoints, 
    void * ptrFeatures,
    real32_T *outLoc, 
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl,
    uint8_T  *outMis,
    real32_T *outFtrs);

#endif
