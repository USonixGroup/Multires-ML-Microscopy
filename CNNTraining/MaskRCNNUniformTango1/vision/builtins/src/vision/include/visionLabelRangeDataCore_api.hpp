
#ifndef _VISIONLABELRANGEDATACORE_
#define _VISIONLABELRANGEDATACORE_

#include "vision_defines.h"


EXTERN_C LIBMWCVSTRT_API void visionLabelRangeData_single(float* xyz, float* range, int nRows, int nCols, float threshold, float angle_threshold, uint32_T minClusterPoints, uint32_T maxClusterPoints, uint32_T* labels, float* numClusters);

EXTERN_C LIBMWCVSTRT_API void visionLabelRangeData_double(double* xyz, double* range, int nRows, int nCols, double threshold, double angle_threshold, uint32_T minClusterPoints, uint32_T maxClusterPoints, uint32_T* labels, double* numClusters);

#endif //_VISIONLABELRANGEDATACORE_