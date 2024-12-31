//=========================================================================
//
//    Templatized class implementation for nonUniformVoxelGridFilter
//    Copyright 2018 The MathWorks, Inc.
//
//=========================================================================
#ifndef _NONUNIFORMVOXELGRIDFILTER_H_
#define _NONUNIFORMVOXELGRIDFILTER_H_

// system includes
#include <assert.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <limits>
#include <cstdlib>
#include <cstring>

#ifndef COMPILE_FOR_VISION_BUILTINS
#include "normalVector_core.hpp"
#else
// export includes
#include <mfl_scalar/query.hpp>
#include <vision/normalVector_core.hpp>
#endif

#if defined(NDEBUG)
#define VISION_ASSERT(EXPR) (void)0
#else
#define VISION_ASSERT(EXPR) assert(EXPR)
#endif

#if defined(NDEBUG)
#define VISION_ASSERT_MSG(EXPR, MSG) (void)0
#else
#define VISION_ASSERT_MSG(EXPR, MSG)            \
do {                                            \
        if (! (EXPR)) {                         \
                printf("%s \n",MSG);            \
                assert((EXPR));                 \
        }                                       \
} while (false)
#endif
    
//=========================================================================
//  execute function of class contains core implementation for nonUniformVoxelGridFilter
//  In addition to sub-sampling, this filter also generates normal vectors
//  if needNormals is true.
//
//  inputs:
//      pData					:		Mx3 or MxNx3 matrix (single or double)
//      numPoints               :       no of pData points
//      maxVoxelPoints		    :       the maximum number of points in voxel
//   outputs:
//      indices						:		Nx1 indices of filtered points (uint32)
//      filteredNormal				:       Nx3 filtered normals if the input normal is empty
//
// Note: this filter removes points with NaN, Inf value.
//=========================================================================
    template <typename valueType>
            class NonUniformVoxelGridAlgImpl
    {
            public:
                uint32_T execute(const valueType* pData,
                        uint32_T numPoints,
                        double maxVoxelPoints,
                        bool needNormals = false
                        )
                {
                    mData = pData;
                    mNumPoints = numPoints;
                    
                    mMaxVoxelPoints = maxVoxelPoints;
                    
                    mNeedNormals = needNormals;
                    
                    VISION_ASSERT_MSG(!((!mNeedNormals && mMaxVoxelPoints < 2) || (mNeedNormals && mMaxVoxelPoints < 3)),
                            "Invalid Input, please check the given maximum number of Voxel Points");
                    
                    // Get the range of data first
                    std::vector<valueType> minPoint(3, 0), maxPoint(3, 0);
                    for (uint32_T j = 0; j < 3; j++) {
                        minPoint[j] = std::numeric_limits<valueType>::max();
                        maxPoint[j] = -std::numeric_limits<valueType>::max();
                    }
                    
                    for (uint32_T i = 0; i < mNumPoints; i++) {
                        valueType x = mData[i];
                        valueType y = mData[i + mNumPoints];
                        valueType z = mData[i + mNumPoints * 2];
                        // Skip invalid values
#ifdef COMPILE_FOR_VISION_BUILTINS
                        if (!mfl_scalar::IsFinite(x) || !mfl_scalar::IsFinite(y) || !mfl_scalar::IsFinite(z))
                            continue;
#else
                        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                            continue;
#endif
                        minPoint[0] = std::min(minPoint[0], x);
                        minPoint[1] = std::min(minPoint[1], y);
                        minPoint[2] = std::min(minPoint[2], z);
                        maxPoint[0] = std::max(maxPoint[0], x);
                        maxPoint[1] = std::max(maxPoint[1], y);
                        maxPoint[2] = std::max(maxPoint[2], z);
                    }
                    
                    // Allocate memory for input indices and reserve memory for output indices
                    std::vector<uint32_T> indices;
                    indices.reserve(mNumPoints);
                    // Estimate the memory needed
                    mOutIndices.reserve(static_cast<int>(mNumPoints / (mMaxVoxelPoints-1)));
                    
                    // index is zero-based
                    for (uint32_T i = 0; i < mNumPoints; i++) {
                        indices.push_back(i);
                    }
                    
                    // Reserve memory for output normals
                    if (mNeedNormals) {
                        // Normal is filled with row-major format in this buffer
                        mNormals.reserve(static_cast<int>(mNumPoints * 3 / (mMaxVoxelPoints-1)));
                    }
                    
                    // Recursively partition the space until each voxel has the maximum points
                    partition(0, mNumPoints, minPoint, maxPoint, indices);
                    
                    size_t nOutSize = mOutIndices.size();
                    
                    return (uint32_T)nOutSize;
                }
                // caller must need to allocate/deallocate memory for pIndices and pNormal
                void getOutputs(uint32_T* pIndices, valueType* pNormal = nullptr)
                {
                    size_t nOutSize = mOutIndices.size();
                    
                    for (size_t i = 0; i < nOutSize; i++) {
                        pIndices[i] = mOutIndices[i] + 1;
                    }
                    if (mNeedNormals) {
                        for (size_t i = 0; i < nOutSize; i++) {
                            pNormal[i] = mNormals[i * 3];
                            pNormal[i + nOutSize] = mNormals[i * 3 + 1];
                            pNormal[i + 2 * nOutSize] = mNormals[i * 3 + 2];
                        }
                    }
                }
            private:
                struct CompareDim
                {
                    uint32_T dim;
                    const valueType * data;
                    uint32_T num;
                    
                    CompareDim(const valueType * pData, uint32_T numPoints, uint32_T cutDim)
                    {
                        data = pData;
                        num = numPoints;
                        dim = cutDim;
                    }
                    
                    // Compare values at a specific dimension: x, or y, or z
                    bool operator () (const uint32_T& p0, const uint32_T& p1)
                    {
                        if (dim == 0)
                            return (data[p0] < data[p1]);
                        else if (dim == 1)
                            return (data[p0+num] < data[p1+num]);
                        else if (dim == 2)
                            return (data[p0+2*num] < data[p1+2*num]);
                        return false;
                    }
                };
                
                ///////////////////////////////////////////////////////////////////////////////
                // Recursively partition the data in the range of [first, last)
                // The current voxel is given by minPoint and maxPoint.
                // Only indices of points are sorted.
                ///////////////////////////////////////////////////////////////////////////////
                void partition(uint32_T first, uint32_T last, std::vector<valueType>& minPoint,
                        std::vector<valueType>& maxPoint, std::vector<uint32_T>& indices)
                {
                    uint32_T count = last - first;
                    // Sample a point and compute the normal using PCA if necessary
                    if (count <= this->mMaxVoxelPoints) {
                        samplePartition(first, last, indices);
                        return;
                    }
                    
                    // Determine the partition axis
                    uint32_T cutDim = 0;
                    valueType diff, maxDiff = 0;
                    for (uint32_T j = 0; j < 3; j++) {
                        diff = maxPoint[j] - minPoint[j];
                        if (diff > maxDiff) {
                            maxDiff = diff;
                            cutDim = j;
                        }
                    }
                    
                    // Cut the space in half
                    uint32_T rightCount = count / 2;
                    uint32_T leftCount = count - rightCount;
                    
                    // Sort the indices in the range
                    std::nth_element(indices.begin() + first, indices.begin() + first + leftCount,	indices.begin() + last, CompareDim(mData, mNumPoints, cutDim));
                    
                    uint32_T cutIndex = indices[first + leftCount];
                    valueType cutVal = mData[cutIndex + cutDim * mNumPoints];
                    
                    // Update bounds for the left voxel
                    std::vector<valueType> leftMaxValues(maxPoint);
                    leftMaxValues[cutDim] = cutVal;
                    // Update bounds for the right voxel
                    std::vector<valueType> rightMinValues(minPoint);
                    rightMinValues[cutDim] = cutVal;
                    
                    // Recurse
                    partition(first, first + leftCount, minPoint, leftMaxValues, indices);
                    partition(first + leftCount, last, rightMinValues, maxPoint, indices);
                }
                
                ///////////////////////////////////////////////////////////////////////////////
                // Sample a point between indices[first] and indices[last]
                ///////////////////////////////////////////////////////////////////////////////
                void samplePartition(uint32_T first, uint32_T last, std::vector<uint32_T>& indices)
                {
                    std::vector<uint32_T> neighborIndices;
                    neighborIndices.reserve(last-first);
                    
                    // Get the points in this voxel, exclude the invalid point
                    for (uint32_T i = first; i < last; i++) {
                        uint32_T index = indices[i];
                        valueType x = mData[index];
                        valueType y = mData[index + mNumPoints];
                        valueType z = mData[index + mNumPoints * 2];
#ifdef COMPILE_FOR_VISION_BUILTINS
                        if (!mfl_scalar::IsFinite(x) || !mfl_scalar::IsFinite(y) || !mfl_scalar::IsFinite(z))
                            continue;
#else
                        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                            continue;
#endif
                        neighborIndices.push_back(index);
                    }
                    
                    uint32_T numNeighbors = static_cast<uint32_T>(neighborIndices.size());
                    if ((!mNeedNormals && numNeighbors == 0) || (mNeedNormals && numNeighbors < 3))
                        return;
                    
                    // Randomly select a point
                    int randIndex = std::rand() % numNeighbors;
                    mOutIndices.push_back(neighborIndices[randIndex]);
                    
                    if (mNeedNormals) {
                        std::vector<valueType> neighbors(numNeighbors * 3);
                        valueType normalVector[3];
                        
                        for (uint32_T i = 0; i < numNeighbors; i++) {
                            uint32_T index = neighborIndices[i];
                            neighbors[i] = mData[index];
                            neighbors[i + numNeighbors] = mData[index + mNumPoints];
                            neighbors[i + 2 * numNeighbors] = mData[index + mNumPoints * 2];
                        }
                        
                        vision::estimateNormalVectorWithPCA<valueType>(&(neighbors[0]), &(neighbors[0]), normalVector, numNeighbors);
                        
                        mNormals.push_back(normalVector[0]);
                        mNormals.push_back(normalVector[1]);
                        mNormals.push_back(normalVector[2]);
                    }
                }
                
                const valueType * mData;
                
                uint32_T mNumPoints;
                double mMaxVoxelPoints;
                
                std::vector<uint32_T> mOutIndices;
                bool mNeedNormals;
                std::vector<valueType> mNormals;
    };
    
// instantiate templates <DataType>
    template class  NonUniformVoxelGridAlgImpl<float>;
    template class  NonUniformVoxelGridAlgImpl<double>;
#endif
