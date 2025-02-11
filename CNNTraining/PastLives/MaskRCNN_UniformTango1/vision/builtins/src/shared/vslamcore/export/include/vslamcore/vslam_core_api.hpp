/* Copyright 2022-2024 The MathWorks, Inc. */
#ifndef VSLAM_CORE_API
#define VSLAM_CORE_API

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
void* MonoVisualSLAM_constructor(
    const double fx,
    const double fy,
    const double cx,
    const double cy,
    const BaseParams* baseParams,
    const IMUParams* imuParams,
    const char* vocabFile,
    const int threadLevel,
    const double* camToIMU); // row-major 4-by-4 transform matrix

EXTERN_C LIBMWVSLAMCORE_API 
void MonoVisualSLAM_addFrame(
    void* objPtr,
    const uint8_T* imgData, // row-major nRows-by-nCols matrix
    const int nRows,
    const int nCols,
    const double* imuG, // row-major imuRows-by-3 matrix
    const double* imuA, // row-major imuRows-by-3 matrix
    const int imuRows);

EXTERN_C LIBMWVSLAMCORE_API 
void MonoVisualSLAM_storeGravityRotationAndScale(void* objPtr,
    const double* gRot, // row-major 4-by-4 transform matrix
    const double poseScale);

#endif // VSLAM_CORE_API