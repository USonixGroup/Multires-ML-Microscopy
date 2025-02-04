/* Copyright 2023-2024 The MathWorks, Inc. */
#ifndef STEREOVSLAM_CORE_API
#define STEREOVSLAM_CORE_API

#ifndef LIBMWVSLAMCORE_API
#    define LIBMWVSLAMCORE_API
#endif

#ifndef EXTERN_C
#  ifdef __cplusplus
#    define EXTERN_C extern "C"
#  else
#    define EXTERN_C extern
#  endif
#endif

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include "ParameterStruct.hpp"

EXTERN_C LIBMWVSLAMCORE_API
void* StereoVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const double baseline,
    const BaseParams* baseParams,
    const StereoParams* stereoParams,
    const char* vocabFile,
    const int threadLevel);

EXTERN_C LIBMWVSLAMCORE_API
void StereoVisualSLAM_addFrame(void* objPtr,
    const uint8_T* I1,
    const uint8_T* I2,
    const real32_T* disparity,
    const int nRows,
    const int nCols);
    
EXTERN_C LIBMWVSLAMCORE_API 
StereoParams StereoVisualSLAM_defaultStereoParams();

#endif // STEREOVSLAM_CORE_API