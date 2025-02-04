//////////////////////////////////////////////////////////////////////////////
//
//      searchOrganizedPointcloudwrapper
//      Copyright 2018-2019 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////

#ifdef __portable__
#include "libmwsearchOrganizedPointCloud_util.hpp"
#include "searchOrganizedPointCloud_published_c_api.hpp"
#include "searchOrganizedPointCloud_core.hpp"
#else
#include <searchOrganizedPointCloud/libmwsearchOrganizedPointCloud_util.hpp>
#include <searchOrganizedPointCloud/searchOrganizedPointCloud_published_c_api.hpp>
#include "vision/searchOrganizedPointCloud_core.hpp"
#endif

using namespace vision;

///////////////////////////////////////////////////////////////////////////
// knn search for different classes (float/double)
///////////////////////////////////////////////////////////////////////////
uint32_T searchOrganizedPointCloud_knnsearch_single(float* location, uint32_T height, uint32_T width, 
        float *point, double kValue, float *pointProjection, float *KRKRT, void** resultIndices, void** resultDistances)
{
    SearchOrganizedPointCloudImpl<float> organizedCloud(location, height, width);
    
    // create Indices container
    std::vector<uint32_T> *ptrIndices = new std::vector<uint32_T>();
    *resultIndices = (void *)ptrIndices;
    
    // create distances container
    std::vector<real32_T> *ptrDistances = new std::vector<real32_T>();
    *resultDistances = (void *)ptrDistances;
    
    std::vector<uint32_T> &refIndices = *ptrIndices;
    std::vector<real32_T> &refDistances = *ptrDistances;
    
    organizedCloud.knnSearch(point, kValue, pointProjection, KRKRT, refIndices, refDistances);
    
    return static_cast<uint32_T>(refIndices.size());
}
uint32_T searchOrganizedPointCloud_knnsearch_double(double* location, uint32_T height, uint32_T width, 
        double *point, double kValue, double *pointProjection, double *KRKRT, void** resultIndices, void** resultDistances)
{
    SearchOrganizedPointCloudImpl<double> organizedCloud(location, height, width);
    
    // create Indices container
    std::vector<uint32_T> *ptrIndices = new std::vector<uint32_T>();
    *resultIndices = (void *)ptrIndices;
    
    // create distances container
    std::vector<real64_T> *ptrDistances = new std::vector<real64_T>();
    *resultDistances = (void *)ptrDistances;
    
    std::vector<uint32_T> &refIndices = *ptrIndices;
    std::vector<real64_T> &refDistances = *ptrDistances;
    
    organizedCloud.knnSearch(point, kValue, pointProjection, KRKRT, refIndices, refDistances);
    
    return static_cast<uint32_T>(refIndices.size());
}
///////////////////////////////////////////////////////////////////////////
// radius search for different classes (float/double)
///////////////////////////////////////////////////////////////////////////
uint32_T searchOrganizedPointCloud_radiussearch_single(float* location, uint32_T height, uint32_T width, 
            float *point, double kValue, float *pointProjection, float *KRKRT, void** resultIndices, void** resultDistances)
{
    SearchOrganizedPointCloudImpl<float> organizedCloud(location, height, width);
    
    // create Indices container
    std::vector<uint32_T> *ptrIndices = new std::vector<uint32_T>();
    *resultIndices = (void *)ptrIndices;
    
    // create distances container
    std::vector<real32_T> *ptrDistances = new std::vector<real32_T>();
    *resultDistances = (void *)ptrDistances;
    
    std::vector<uint32_T> &refIndices = *ptrIndices;
    std::vector<real32_T> &refDistances = *ptrDistances;
    
    organizedCloud.radiusSearch(point, kValue, pointProjection, KRKRT, refIndices, refDistances);
    
    return static_cast<uint32_T>(refIndices.size());
}
uint32_T searchOrganizedPointCloud_radiussearch_double(double* location, uint32_T height, uint32_T width, 
        double *point, double kValue, double *pointProjection, double *KRKRT, void** resultIndices, void** resultDistances)
{
    SearchOrganizedPointCloudImpl<double> organizedCloud(location, height, width);
    
    // create Indices container
    std::vector<uint32_T> *ptrIndices = new std::vector<uint32_T>();
    *resultIndices = (void *)ptrIndices;
    
    // create distances container
    std::vector<real64_T> *ptrDistances = new std::vector<real64_T>();
    *resultDistances = (void *)ptrDistances;
    
    std::vector<uint32_T> &refIndices = *ptrIndices;
    std::vector<real64_T> &refDistances = *ptrDistances;
    
    organizedCloud.radiusSearch(point, kValue, pointProjection, KRKRT, refIndices, refDistances);
    
    return static_cast<uint32_T>(refIndices.size());
}

///////////////////////////////////////////////////////////////////////////
// Assigning outputs for different classes (float/double)
///////////////////////////////////////////////////////////////////////////
void searchOrganizedPointCloudAssignOutputs_single(void* ptrIndices, void* ptrDistances, uint32_T* indicesPtr, float* distancePtr)
{
    std::vector<uint32_T> &refIndices = *((std::vector<uint32_T> *)ptrIndices);
    std::vector<float> &refDistances = *((std::vector<float> *)ptrDistances);
    
    std::copy(refIndices.begin(), refIndices.end(), indicesPtr);
    std::copy(refDistances.begin(), refDistances.end(), distancePtr);
    
    delete((std::vector<uint32_T> *)ptrIndices);
    delete((std::vector<float> *)ptrDistances);
}

void searchOrganizedPointCloudAssignOutputs_double(void* ptrIndices, void* ptrDistances, uint32_T* indicesPtr, double* distancePtr)
{
    std::vector<uint32_T> &refIndices = *((std::vector<uint32_T> *)ptrIndices);
    std::vector<double> &refDistances = *((std::vector<double> *)ptrDistances);
    
    std::copy(refIndices.begin(), refIndices.end(), indicesPtr);
    std::copy(refDistances.begin(), refDistances.end(), distancePtr);
    
    delete((std::vector<uint32_T> *)ptrIndices);
    delete((std::vector<double> *)ptrDistances);
}


