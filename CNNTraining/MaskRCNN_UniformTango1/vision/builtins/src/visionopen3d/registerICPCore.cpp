//////////////////////////////////////////////////////////////////////////////
//
// APIs for interfaces used in pcregistericp codegeneration
// Copyright 2023 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
// vision_builtins does not need this source file

#ifndef COMPILE_FOR_VISION_BUILTINS

#include "registerICPCore_api.hpp"
#include "registerICPUtils.hpp"

///////////////////////////////////////////////////////////////////////////
//    registration wrapper for single datatype
///////////////////////////////////////////////////////////////////////////
double registerICP_single(void* mLocationsA, void* mLocationsB,
                          const boolean_T hasNormalsA, void* mNormalsA,
                          const boolean_T hasNormalsB, void* mNormalsB,
                          boolean_T useColors, void* mPtCloudAColors,
                          void* mPtCloudBColors, void* mVoxelSizes,
                          void* mMaxIterationVector, void* mInitTform,
                          void* mDataTypeTform,
                          const int maxIteration,
                          void* mMethod,
                          const double relativeRotation,
                          const double relativeTranslation,
                          const double metric,
                          const size_t numPointsA,
                          const size_t numPointsB,
                          const boolean_T useInlierRatio,
                          void* mOutTransform){

    double* outTransform = static_cast<double*>(mOutTransform);

    const float* locationsA = static_cast<const float*>(mLocationsA);
    const float* locationsB = static_cast<const float*>(mLocationsB);

    o3dGeometry::PointCloud moving = readPointCloud<float>(locationsA, numPointsA);
    o3dGeometry::PointCloud fixed = readPointCloud<float>(locationsB, numPointsB);

    const float* normalsA = static_cast<const float*>(mNormalsA);
    const float* normalsB = static_cast<const float*>(mNormalsB);

    if(hasNormalsA){
        moving.normals_ = readNormals<float>(normalsA, numPointsA);
    }

    if(hasNormalsB){
        fixed.normals_ = readNormals<float>(normalsB, numPointsB);
    }

    const double* colorsA = static_cast<const double*>(mPtCloudAColors);
    const double* colorsB = static_cast<const double*>(mPtCloudBColors);
    const double* voxelSizeData = static_cast<const double*>(mVoxelSizes);
    const double* maxIterationVector = static_cast<const double*>(mMaxIterationVector); 

    std::vector<double> voxelSizes(voxelSizeData, voxelSizeData+3);
    std::vector<int> iterations(maxIterationVector, maxIterationVector+3);
    if(useColors) {
        moving.colors_ = readNormalizedColors(colorsA, numPointsA);
        fixed.colors_ = readNormalizedColors(colorsB, numPointsB);    
    }

    const std::string dataTypeTform = static_cast<const char*>(mDataTypeTform);
    Eigen::Matrix4d initTform;
    if (dataTypeTform == "single"){

        const float* initialTform = static_cast<const float*>(mInitTform);
        initTform = readInitTform<float>(initialTform);
    }
    else
    {
        const double* initialTform = static_cast<const double*>(mInitTform);
        initTform = readInitTform<double>(initialTform);
    }

    const std::string method = static_cast<const char*>(mMethod);

    auto regResult = processICPImpl(moving, fixed,
                                    metric, initTform, 
                                    maxIteration, relativeRotation,
                                    relativeTranslation, useInlierRatio, method,
                                    voxelSizes, iterations);
    return packageOutputs(regResult, outTransform);
}

///////////////////////////////////////////////////////////////////////////
//    registration wrapper for double datatype
///////////////////////////////////////////////////////////////////////////
double registerICP_double(void* mLocationsA, void* mLocationsB,
                          const boolean_T hasNormalsA, void* mNormalsA,
                          const boolean_T hasNormalsB, void* mNormalsB,
                          boolean_T useColors, void* mPtCloudAColors,
                          void* mPtCloudBColors, void* mVoxelSizes,
                          void* mMaxIterationVector, void* mInitTform,
                          void* mDataTypeTform,
                          const int maxIteration,
                          void* mMethod,
                          const double relativeRotation,
                          const double relativeTranslation,
                          const double metric,
                          const size_t numPointsA,
                          const size_t numPointsB,
                          const boolean_T useInlierRatio,
                          void* mOutTransform){

    double* outTransform = static_cast<double*>(mOutTransform);

    const double* locationsA = static_cast<const double*>(mLocationsA);
    const double* locationsB = static_cast<const double*>(mLocationsB);

    o3dGeometry::PointCloud moving = readPointCloud<double>(locationsA, numPointsA);
    o3dGeometry::PointCloud fixed = readPointCloud<double>(locationsB, numPointsB);

    const double* normalsA = static_cast<const double*>(mNormalsA);
    const double* normalsB = static_cast<const double*>(mNormalsB);

    if(hasNormalsA){
        moving.normals_ = readNormals<double>(normalsA, numPointsA);
    }

    if(hasNormalsB){
        fixed.normals_ = readNormals<double>(normalsB, numPointsB);
    }

    const double* colorsA = static_cast<const double*>(mPtCloudAColors);
    const double* colorsB = static_cast<const double*>(mPtCloudBColors);
    const double* voxelSizeData = static_cast<const double*>(mVoxelSizes);
    const double* maxIterationVector = static_cast<const double*>(mMaxIterationVector);

    std::vector<double> voxelSizes(voxelSizeData, voxelSizeData+3);
    std::vector<int> iterations(maxIterationVector, maxIterationVector+3);
    if(useColors) {
        moving.colors_ = readNormalizedColors(colorsA, numPointsA);
        fixed.colors_ = readNormalizedColors(colorsB, numPointsB);
       }

    const std::string dataTypeTform = static_cast<const char*>(mDataTypeTform);
    Eigen::Matrix4d initTform;
    if (dataTypeTform == "single"){

        const float* initialTform = static_cast<const float*>(mInitTform);
        initTform = readInitTform<float>(initialTform);
    }
    else
    {
        const double* initialTform = static_cast<const double*>(mInitTform);
        initTform = readInitTform<double>(initialTform);
    }

    const std::string method = static_cast<const char*>(mMethod);

    auto regResult = processICPImpl(moving, fixed,
                                    metric, initTform, 
                                    maxIteration, relativeRotation,
                                    relativeTranslation, useInlierRatio, method,
                                    voxelSizes, iterations);
    return packageOutputs(regResult, outTransform);
}

#endif
