/* Copyright 2021 The MathWorks, Inc. */

#ifndef _EXTRACTSIFT_
#define _EXTRACTSIFT_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API int32_T extractSIFTComputeColumnMajor(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inScl,
    real32_T *inMet,
    real32_T *inOri,
    int32_T  *inOct,
    int32_T  *inLay,
	int numKPts,
	void **outKeypoints, void **outFeatures);

EXTERN_C LIBMWCVSTRT_API int32_T extractSIFTComputeRowMajor(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inScl,
    real32_T *inMet,
    real32_T *inOri,
    int32_T  *inOct,
    int32_T  *inLay,
	int numKPts,
	void **outKeypoints, void **outFeatures);

EXTERN_C LIBMWCVSTRT_API void extractSIFTAssignOutputColumnMajor(
    void * ptrKeypoints, 
    void * ptrFeatures,
    real32_T *outLoc, 
    real32_T *outScl,
    real32_T *outMet,
    real32_T *outOri,
    int32_T  *outOct,
    int32_T  *outLay,
    real32_T *outFtrs);

EXTERN_C LIBMWCVSTRT_API void extractSIFTAssignOutputRowMajor(
    void * ptrKeypoints, 
    void * ptrFeatures,
    real32_T *outLoc, 
    real32_T *outScl,
    real32_T *outMet,
    real32_T *outOri,
    int32_T  *outOct,
    int32_T  *outLay,
    real32_T *outFtrs);

#endif
