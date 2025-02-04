/* Copyright 2022-2024 The MathWorks, Inc. */
#ifndef RGBDVSLAM_CORE_API
#define RGBDVSLAM_CORE_API

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
void* RGBDVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const BaseParams* baseParams,
    const RGBDParams* rgbdParams,
    const char* vocabFile,
    const int threadLevel);

EXTERN_C LIBMWVSLAMCORE_API
void RGBDVisualSLAM_addFrame(void* objPtr,
    const uint8_T* imgColorData, // row-major nRows-by-nCols matrix
    const real32_T* imgDepthData, // row-major nRows-by-nCols matrix
    const int nRows,
    const int nCols);

#endif // RGBDVSLAM_CORE_API