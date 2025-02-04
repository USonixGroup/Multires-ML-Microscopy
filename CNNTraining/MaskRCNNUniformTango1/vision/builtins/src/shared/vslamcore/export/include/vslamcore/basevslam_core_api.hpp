/* Copyright 2024 The MathWorks, Inc. */
#ifndef BASEVSLAM_CORE_API
#define BASEVSLAM_CORE_API

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
bool BaseVisualSLAM_hasNewKeyFrame(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
void BaseVisualSLAM_getWorldPoints(void* objPtr,
                                   double* xyzPoints); // row-major N-by-3 matrix

EXTERN_C LIBMWVSLAMCORE_API 
void BaseVisualSLAM_getCameraPoses(void* objPtr,
                                   double* camPoses); // row-major N-by-4-by-4 matrix

EXTERN_C LIBMWVSLAMCORE_API 
void BaseVisualSLAM_getKeyFrameIndex(void* objPtr, int* keyFrameIndices);

EXTERN_C LIBMWVSLAMCORE_API
void BaseVisualSLAM_getViewIDs(void* objPtr, int* viewIDs);

EXTERN_C LIBMWVSLAMCORE_API 
int BaseVisualSLAM_getNumWorldPoints(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
int BaseVisualSLAM_getNumCameraPoses(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
bool BaseVisualSLAM_isInitialized(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
bool BaseVisualSLAM_isLoopRecentlyClosed(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
int BaseVisualSLAM_getNumTrackedPoints(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
bool BaseVisualSLAM_isDone(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
void BaseVisualSLAM_reset(void* objPtr);

EXTERN_C LIBMWVSLAMCORE_API 
int BaseVisualSLAM_getNumIMUMeasurements(void* objPtr, const int viewId);

EXTERN_C LIBMWVSLAMCORE_API 
void BaseVisualSLAM_getViewIMUMeasurements(void* objPtr, const int viewId, double* imuG, double* imuA);

EXTERN_C LIBMWVSLAMCORE_API 
BaseParams BaseVisualSLAM_defaultBaseParams();

EXTERN_C LIBMWVSLAMCORE_API 
IMUParams BaseVisualSLAM_defaultIMUParams();

#endif // BASEVSLAM_CORE_API