///////////////////////////////////////////////////////////////////////////
//  Kdtree contains implementation of kdtree_published_c_api.
//  Copyright 2018-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////

#include "kdtree/vision_defines.h"
#include "kdtree/kdtree_published_c_api.hpp"
#include "kdtree/KdtreeImpl.hpp"
#include <cstring>

///////////////////////////////////////////////////////////////////////////
// constructor for different classes (float/double) for column major
///////////////////////////////////////////////////////////////////////////

void kdtreeConstruct_single(void** ptr2kdtreeObjPtr) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,0>*)new vision::
            KdtreeImpl<nanoflann::metric_L2, float ,0>;
    *ptr2kdtreeObjPtr = kdtreeObjPtr;
}

void kdtreeConstruct_double(void** ptr2kdtreeObjPtr) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,0>*)new vision::
            KdtreeImpl<nanoflann::metric_L2, double ,0>;
    *ptr2kdtreeObjPtr = kdtreeObjPtr;
}

///////////////////////////////////////////////////////////////////////////
// constructor for different classes (float/double) for row major
///////////////////////////////////////////////////////////////////////////

void kdtreeConstructRM_single(void** ptr2kdtreeObjPtr) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,1>*)new vision::
            KdtreeImpl<nanoflann::metric_L2, float ,1>;
    *ptr2kdtreeObjPtr = kdtreeObjPtr;
}

void kdtreeConstructRM_double(void** ptr2kdtreeObjPtr) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,1>*)new vision::
            KdtreeImpl<nanoflann::metric_L2, double ,1>;
    *ptr2kdtreeObjPtr = kdtreeObjPtr;
}

///////////////////////////////////////////////////////////////////////////
// needsReIndex function for different classes (float/double) for column major
///////////////////////////////////////////////////////////////////////////

boolean_T kdtreeNeedsReindex_single(void* ptrObj, void* pData) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,0>*)ptrObj;
    return kdtreeObjPtr->needsReindex(static_cast<float*>(pData));
}

boolean_T kdtreeNeedsReindex_double(void* ptrObj, void* pData) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,0>*)ptrObj;
    return kdtreeObjPtr->needsReindex(static_cast<double*>(pData));
}

///////////////////////////////////////////////////////////////////////////
// needsReIndex function for different classes (float/double) for row major
///////////////////////////////////////////////////////////////////////////

boolean_T kdtreeNeedsReindexRM_single(void* ptrObj, void* pData) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,1>*)ptrObj;
    return kdtreeObjPtr->needsReindex(static_cast<float*>(pData));
}

boolean_T kdtreeNeedsReindexRM_double(void* ptrObj, void* pData) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,1>*)ptrObj;
    return kdtreeObjPtr->needsReindex(static_cast<double*>(pData));
}

///////////////////////////////////////////////////////////////////////////
// index function for different classes (float/double) for column major
///////////////////////////////////////////////////////////////////////////

void kdtreeIndex_single(void* ptrObj,
                        void* pData,
                        uint32_t dataSize,
                        uint32_t dims,
                        double bucketSize) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,0>*)ptrObj;
    kdtreeObjPtr->index(static_cast<float*>(pData), dataSize, dims,static_cast<std::size_t>(bucketSize));
}

void kdtreeIndex_double(void* ptrObj,
                        void* pData,
                        uint32_t dataSize,
                        uint32_t dims,
                        double bucketSize) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,0>*)ptrObj;
    kdtreeObjPtr->index(static_cast<double*>(pData), dataSize, dims,
                        static_cast<std::size_t>(bucketSize));
}

///////////////////////////////////////////////////////////////////////////
// index function for different classes (float/double) for row major
///////////////////////////////////////////////////////////////////////////

void kdtreeIndexRM_single(void* ptrObj,
                          void* pData,
                          uint32_t dataSize,
                          uint32_t dims,
                          double bucketSize) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,1>*)ptrObj;
    kdtreeObjPtr->index(static_cast<float*>(pData), dataSize, dims,
                        static_cast<std::size_t>(bucketSize));
}

void kdtreeIndexRM_double(void* ptrObj,
                          void* pData,
                          uint32_t dataSize,
                          uint32_t dims,
                          double bucketSize) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,1>*)ptrObj;
    kdtreeObjPtr->index(static_cast<double*>(pData), dataSize, dims,
                        static_cast<std::size_t>(bucketSize));
}

///////////////////////////////////////////////////////////////////////////////////////
// box search for different classes (float/double) for column major
///////////////////////////////////////////////////////////////////////////////////////

int32_T kdtreeBoxSearch_single(void* ptrObj, void* roi, void** indices) {
    vision::KdtreeImpl<nanoflann::metric_BBOX_IOU_METRIC, float, true>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_BBOX_IOU_METRIC, float, true>*)ptrObj;
    return kdtreeObjPtr->boxSearch(roi, indices);
}

int32_T kdtreeBoxSearch_double(void* ptrObj, void* roi, void** indices) {
    vision::KdtreeImpl<nanoflann::metric_BBOX_IOU_METRIC, double, true>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_BBOX_IOU_METRIC, double, true>*)ptrObj;
    return kdtreeObjPtr->boxSearch(roi, indices);
}

///////////////////////////////////////////////////////////////////////////////////////
// box search for different classes (float/double) for row major
///////////////////////////////////////////////////////////////////////////////////////

int32_T kdtreeBoxSearchRM_single(void* ptrObj, void* roi, void** indices) {
    vision::KdtreeImpl<nanoflann::metric_BBOX_IOU_METRIC, float, false>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_BBOX_IOU_METRIC, float, false>*)ptrObj;
    return kdtreeObjPtr->boxSearch(roi, indices);
}

int32_T kdtreeBoxSearchRM_double(void* ptrObj, void* roi, void** indices) {
    vision::KdtreeImpl<nanoflann::metric_BBOX_IOU_METRIC, double, false>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_BBOX_IOU_METRIC, double, false>*)ptrObj;
    return kdtreeObjPtr->boxSearch(roi, indices);
}

///////////////////////////////////////////////////////////////////////////////////////
// Assign Box Search Outputs
///////////////////////////////////////////////////////////////////////////////////////

void kdtreeBoxSearchSetOutputs(void* ptrIndices, uint32_t* location) {
    std::vector<std::size_t>& refIndices = *((std::vector<std::size_t>*)ptrIndices);
    size_t m = refIndices.size();
    for (size_t i = 0; i < m; i++) {
        location[i] = static_cast<uint32_t>(refIndices[i] + 1); // Convert to 1 based indexing
    }
    delete ((std::vector<std::size_t>*)ptrIndices);
}

///////////////////////////////////////////////////////////////////////////////////////
// KNN search for different classes (float/double) for column major
///////////////////////////////////////////////////////////////////////////////////////
int32_T kdtreeKNNSearch_single(void* ptrObj,
                               void* queryData,
                               uint32_t numQueries,
                               uint32_t numQueryDims,
                               uint32_t knn,
                               float paramEps,                              
                               void* indices,
                               void* dists,
                               void* valid,
                               int32_T grainSize,
                               uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,0>*)ptrObj;
    return kdtreeObjPtr->knnSearch(static_cast<float*>(queryData), numQueries, numQueryDims, knn,
                                   paramEps, indices, dists,
                                   valid, grainSize, tbbQueryThreshold);
}

int32_T kdtreeKNNSearch_double(void* ptrObj,
                               void* queryData,
                               uint32_t numQueries,
                               uint32_t numQueryDims,
                               uint32_t knn,
                               double paramEps,
                               void* indices,
                               void* dists,
                               void* valid,
                               int32_T grainSize,
                               uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,0>*)ptrObj;
    return kdtreeObjPtr->knnSearch(static_cast<double*>(queryData), numQueries, numQueryDims, knn,
                                   paramEps, indices, dists,
                                   valid, grainSize, tbbQueryThreshold);
}

///////////////////////////////////////////////////////////////////////////////////////
// KNN search for different classes (float/double) for row major
///////////////////////////////////////////////////////////////////////////////////////
int32_T kdtreeKNNSearchRM_single(void* ptrObj,
                                 void* queryData,
                                 uint32_t numQueries,
                                 uint32_t numQueryDims,
                                 uint32_t knn,
                                 float paramEps,
                                 void* indices,
                                 void* dists,
                                 void* valid,
                                 int32_T grainSize,
                                 uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,1>*)ptrObj;
    uint32_t* indices_T = new uint32_t[numQueries * knn];
    float* dists_T = new float[numQueries * knn];
    int32_T numReturn =
        kdtreeObjPtr->knnSearch(static_cast<float*>(queryData), numQueries, numQueryDims, knn,
                                paramEps, (void*)indices_T,
                                (void*)dists_T, valid, grainSize, tbbQueryThreshold);
    copyToArray_RowMajor<uint32_t>(indices_T, static_cast<uint32_t*>(indices), knn, numQueries);
    copyToArray_RowMajor<float>(dists_T, static_cast<float*>(dists), knn, numQueries);
    delete[] indices_T;
    delete[] dists_T;
    return numReturn;
}

int32_T kdtreeKNNSearchRM_double(void* ptrObj,
                                 void* queryData,
                                 uint32_t numQueries,
                                 uint32_t numQueryDims,
                                 uint32_t knn,
                                 double paramEps,
                                 void* indices,
                                 void* dists,
                                 void* valid,
                                 int32_T grainSize,
                                 uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,1>*)ptrObj;
    uint32_t* indices_T = new uint32_t[numQueries * knn];
    double* dists_T = new double[numQueries * knn];
    int32_T numReturn =
        kdtreeObjPtr->knnSearch(static_cast<double*>(queryData), numQueries, numQueryDims, knn,
                                paramEps, (void*)indices_T,
                                (void*)dists_T, valid, grainSize, tbbQueryThreshold);
    copyToArray_RowMajor<uint32_t>(indices_T, static_cast<uint32_t*>(indices), knn, numQueries);
    copyToArray_RowMajor<double>(dists_T, static_cast<double*>(dists), knn, numQueries);
    delete[] indices_T;
    delete[] dists_T;
    return numReturn;
}

///////////////////////////////////////////////////////////////////////////////////////
// Hybrid search for different classes (float/double) for column major
///////////////////////////////////////////////////////////////////////////////////////
int32_T kdtreeHybridSearch_single(void* ptrObj,
                               void* queryData,
                               uint32_t numQueries,
                               uint32_t numQueryDims,
                               uint32_t knn,
                               float radius,
                               float paramEps,                              
                               void* indices,
                               void* dists,
                               void* valid,
                               int32_T grainSize,
                               uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,0>*)ptrObj;
    return kdtreeObjPtr->hybridSearch(static_cast<float*>(queryData), numQueries, numQueryDims, knn,
                                   radius, paramEps, indices, dists,
                                   valid, grainSize, tbbQueryThreshold);
}

int32_T kdtreeHybridSearch_double(void* ptrObj,
                               void* queryData,
                               uint32_t numQueries,
                               uint32_t numQueryDims,
                               uint32_t knn,
                               double radius,
                               double paramEps,
                               void* indices,
                               void* dists,
                               void* valid,
                               int32_T grainSize,
                               uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,0>*)ptrObj;
    return kdtreeObjPtr->hybridSearch(static_cast<double*>(queryData), numQueries, numQueryDims, knn,
                                   radius, paramEps, indices, dists,
                                   valid, grainSize, tbbQueryThreshold);
}

///////////////////////////////////////////////////////////////////////////////////////
// Hybrid search for different classes (float/double) for row major
///////////////////////////////////////////////////////////////////////////////////////
int32_T kdtreeHybridSearchRM_single(void* ptrObj,
                                 void* queryData,
                                 uint32_t numQueries,
                                 uint32_t numQueryDims,
                                 uint32_t knn,
                                 float radius,
                                 float paramEps,
                                 void* indices,
                                 void* dists,
                                 void* valid,
                                 int32_T grainSize,
                                 uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,1>*)ptrObj;
    uint32_t* indices_T = new uint32_t[numQueries * knn];
    float* dists_T = new float[numQueries * knn];
    int32_T numReturn =
        kdtreeObjPtr->hybridSearch(static_cast<float*>(queryData), numQueries, numQueryDims, knn,
                                radius,  paramEps, (void*)indices_T,
                                (void*)dists_T, valid, grainSize, tbbQueryThreshold);
    copyToArray_RowMajor<uint32_t>(indices_T, static_cast<uint32_t*>(indices), knn, numQueries);
    copyToArray_RowMajor<float>(dists_T, static_cast<float*>(dists), knn, numQueries);
    delete[] indices_T;
    delete[] dists_T;
    return numReturn;
}

int32_T kdtreeHybridSearchRM_double(void* ptrObj,
                                 void* queryData,
                                 uint32_t numQueries,
                                 uint32_t numQueryDims,
                                 uint32_t knn,
                                 double radius,
                                 double paramEps,
                                 void* indices,
                                 void* dists,
                                 void* valid,
                                 int32_T grainSize,
                                 uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,1>*)ptrObj;
    uint32_t* indices_T = new uint32_t[numQueries * knn];
    double* dists_T = new double[numQueries * knn];
    int32_T numReturn =
        kdtreeObjPtr->hybridSearch(static_cast<double*>(queryData), numQueries, numQueryDims, knn,
                                radius, paramEps, (void*)indices_T,
                                (void*)dists_T, valid, grainSize, tbbQueryThreshold);
    copyToArray_RowMajor<uint32_t>(indices_T, static_cast<uint32_t*>(indices), knn, numQueries);
    copyToArray_RowMajor<double>(dists_T, static_cast<double*>(dists), knn, numQueries);
    delete[] indices_T;
    delete[] dists_T;
    return numReturn;
}

///////////////////////////////////////////////////////////////////////////////////////
// radius search for different classes (float/double) for column major
///////////////////////////////////////////////////////////////////////////////////////
int32_T kdtreeRadiusSearch_single(void* ptrObj,
                                  void* queryData,
                                  uint32_t numQueries,
                                  uint32_t numQueryDims,
                                  float radius,
                                  float paramEps,
                                  void** indices,
                                  void** dists,
                                  uint32_t* valid,
                                  int32_T grainSize,
                                  uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,0>*)ptrObj;

    return kdtreeObjPtr->radiusSearch(static_cast<float*>(queryData), numQueries, numQueryDims,
                                      radius, paramEps, indices,
                                      dists, valid, grainSize, tbbQueryThreshold);
}

int32_T kdtreeRadiusSearch_double(void* ptrObj,
                                  void* queryData,
                                  uint32_t numQueries,
                                  uint32_t numQueryDims,
                                  double radius,
                                  double paramEps,
                                  void** indices,
                                  void** dists,
                                  uint32_t* valid,
                                  int32_T grainSize,
                                  uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,0>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,0>*)ptrObj;

    return kdtreeObjPtr->radiusSearch(static_cast<double*>(queryData), numQueries, numQueryDims,
                                      radius, paramEps, indices, 
                                      dists, valid, grainSize, tbbQueryThreshold);
}

///////////////////////////////////////////////////////////////////////////////////////
// radius search for different classes (float/double) for row major
///////////////////////////////////////////////////////////////////////////////////////
int32_T kdtreeRadiusSearchRM_single(void* ptrObj,
                                    void* queryData,
                                    uint32_t numQueries,
                                    uint32_t numQueryDims,
                                    float radius,
                                    float paramEps,
                                    void** indices,
                                    void** dists,
                                    uint32_t* valid,
                                    int32_T grainSize,
                                    uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, float ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, float ,1>*)ptrObj;

    return kdtreeObjPtr->radiusSearch(static_cast<float*>(queryData), numQueries, numQueryDims,
                                      radius, paramEps, indices, 
                                      dists, valid, grainSize, tbbQueryThreshold);
}

int32_T kdtreeRadiusSearchRM_double(void* ptrObj,
                                    void* queryData,
                                    uint32_t numQueries,
                                    uint32_t numQueryDims,
                                    double radius,
                                    double paramEps,
                                    void** indices,
                                    void** dists,
                                    uint32_t* valid,
                                    int32_T grainSize,
                                    uint32_t tbbQueryThreshold) {
    vision::KdtreeImpl<nanoflann::metric_L2, double ,1>* kdtreeObjPtr =
        (vision::KdtreeImpl<nanoflann::metric_L2, double ,1>*)ptrObj;

    return kdtreeObjPtr->radiusSearch(static_cast<double*>(queryData), numQueries, numQueryDims,
                                      radius, paramEps, indices, 
                                      dists, valid, grainSize, tbbQueryThreshold);
}

///////////////////////////////////////////////////////////////////////////////////////
// Assign Radius Search Outputs
///////////////////////////////////////////////////////////////////////////////////////

void kdtreeRadiusSearchSetOutputs_single(void* ptrIndicesIn,
                                         void* ptrDistsIn,
                                         uint32_t* ptrIndicesOut,
                                         float* ptrDistsOut) {
    std::vector<std::vector<uint32_t>>& refIndices =
        *((std::vector<std::vector<uint32_t>>*)ptrIndicesIn);
    std::vector<std::vector<float>>& refDists = *((std::vector<std::vector<float>>*)ptrDistsIn);

    size_t m = refIndices.size();
    uint32_t* indicesPtr = ptrIndicesOut;
    float* distsPtr = ptrDistsOut;

    for (size_t i = 0; i < m; i++) {
        std::copy(refIndices[i].begin(), refIndices[i].end(), indicesPtr);
        std::copy(refDists[i].begin(), refDists[i].end(), distsPtr);

        indicesPtr = indicesPtr + refIndices[i].size();
        distsPtr = distsPtr + refIndices[i].size();

        refIndices[i].clear();
        refDists[i].clear();
    }

    delete ((std::vector<std::vector<uint32_t>>*)ptrIndicesIn);
    delete ((std::vector<std::vector<float>>*)ptrDistsIn);
}


void kdtreeRadiusSearchSetOutputs_double(void* ptrIndicesIn,
                                         void* ptrDistsIn,
                                         uint32_t* ptrIndicesOut,
                                         double* ptrDistsOut) {
    std::vector<std::vector<uint32_t>>& refIndices =
        *((std::vector<std::vector<uint32_t>>*)ptrIndicesIn);
    std::vector<std::vector<double>>& refDists = *((std::vector<std::vector<double>>*)ptrDistsIn);

    size_t m = refIndices.size();
    uint32_t* indicesPtr = ptrIndicesOut;
    double* distsPtr = ptrDistsOut;

    for (size_t i = 0; i < m; i++) {
        std::copy(refIndices[i].begin(), refIndices[i].end(), indicesPtr);
        std::copy(refDists[i].begin(), refDists[i].end(), distsPtr);

        indicesPtr = indicesPtr + refIndices[i].size();
        distsPtr = distsPtr + refIndices[i].size();

        refIndices[i].clear();
        refDists[i].clear();
    }

    delete ((std::vector<std::vector<uint32_t>>*)ptrIndicesIn);
    delete ((std::vector<std::vector<double>>*)ptrDistsIn);
}

///////////////////////////////////////////////////////////////////////////
// Delete for different classes (float/double) for column major
///////////////////////////////////////////////////////////////////////////

void kdtreeDeleteObj_single(void* ptrObj) {
    if (ptrObj != NULL) {
        delete ((vision::KdtreeImpl<nanoflann::metric_L2, float ,0>*)ptrObj);
        ptrObj = NULL;
    }
}

void kdtreeDeleteObj_double(void* ptrObj) {
    if (ptrObj != NULL) {
        delete ((vision::KdtreeImpl<nanoflann::metric_L2, double ,0>*)ptrObj);
        ptrObj = NULL;
    }
}

///////////////////////////////////////////////////////////////////////////
// Delete for different classes (float/double) for row major
///////////////////////////////////////////////////////////////////////////

void kdtreeDeleteObjRM_single(void* ptrObj) {
    if (ptrObj != NULL) {
        delete ((vision::KdtreeImpl<nanoflann::metric_L2, float ,1>*)ptrObj);
        ptrObj = NULL;
    }
}

void kdtreeDeleteObjRM_double(void* ptrObj) {
    if (ptrObj != NULL) {
        delete ((vision::KdtreeImpl<nanoflann::metric_L2, double ,1>*)ptrObj);
        ptrObj = NULL;
    }
}

///////////////////////////////////////////////////////////////////////////
// Get Location Pointer for different classes (float/double)
///////////////////////////////////////////////////////////////////////////
void kdtreeGetLocationDataPointer_single(void* locationData,
                                         uint32_t dataSize,
                                         uint32_t dims,
                                         void** ptr2locationDataPtr) {
    float* locationDataPtr = new float[dataSize * dims];
    if (locationDataPtr != NULL) {
        memcpy(locationDataPtr, locationData, dataSize * dims * sizeof(float));
    }
    *ptr2locationDataPtr = locationDataPtr;
}

void kdtreeGetLocationDataPointer_double(void* locationData,
                                         uint32_t dataSize,
                                         uint32_t dims,
                                         void** ptr2locationDataPtr) {
    double* locationDataPtr = new double[dataSize * dims];
    if (locationDataPtr != NULL) {
        memcpy(locationDataPtr, locationData, dataSize * dims * sizeof(double));
    }
    *ptr2locationDataPtr = locationDataPtr;
}

///////////////////////////////////////////////////////////////////////////
// Delete Location Pointer for different classes (float/double)
///////////////////////////////////////////////////////////////////////////
void kdtreeDeleteLocationDataPointer_single(void* locationPtr) {
    if (locationPtr != NULL) {
        delete[](static_cast<float*>(locationPtr));
        locationPtr = NULL;
    }
}

void kdtreeDeleteLocationDataPointer_double(void* locationPtr) {
    if (locationPtr != NULL) {
        delete[](static_cast<double*>(locationPtr));
        locationPtr = NULL;
    }
}
