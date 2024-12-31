/* Copyright 2021 The MathWorks, Inc. */

#ifndef _DETECTSIFT_
#define _DETECTSIFT_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API
int32_T detectSIFTComputeColumnMajor(uint8_T *img, 
                           int nRows, int nCols,
                           float contrastThreshold, float edgeThreshold, 
                           int numLayersInOctave, float sigma,
                           void **outKeypoints);

EXTERN_C LIBMWCVSTRT_API
int32_T detectSIFTComputeRowMajor(uint8_T *img, 
                           int nRows, int nCols,
                           float contrastThreshold, float edgeThreshold, 
                           int numLayersInOctave, float sigma,
                           void **outKeypoints);

EXTERN_C LIBMWCVSTRT_API
void detectSIFTAssignOutputsColumnMajor(void *ptrKeypoints,
                               real32_T * location, real32_T * scale,
                               real32_T * metric, real32_T * orientation,
                               int32_T * octave, int32_T * layer);

EXTERN_C LIBMWCVSTRT_API
void detectSIFTAssignOutputsRowMajor(void *ptrKeypoints,
                               real32_T * location, real32_T * scale,
                               real32_T * metric, real32_T * orientation,
                               int32_T * octave, int32_T * layer);

#endif
