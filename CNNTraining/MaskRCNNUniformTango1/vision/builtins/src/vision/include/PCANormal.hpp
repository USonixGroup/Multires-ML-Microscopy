//=========================================================================
//
//              Compute normals with PCA
//             Copyright 2018 The MathWorks, Inc.
//
//=========================================================================
#ifndef PCANORMAL_HPP
#define PCANORMAL_HPP

#ifndef COMPILE_FOR_VISION_BUILTINS
#include "normalVector_core.hpp"
#else
#include <vision/normalVector_core.hpp>
#endif
#include <vector>
#include <limits>

//=========================================================================
// PCANormalImpl(mPoints ,mIndices, mValid, M, numNeighbors, mNormals)
//
// Inputs:
//   mPoints		: Mx3 matrix (single or double)
//   mIndices		: [NUM of Neighbors]xM uint32 matrix, indices of neighbor points in mPoints
//   mValid      	: Mx1 vector indicating the actual number of neighbors in indices
// Output:
//   mNormals       : Mx3 matrix (single or double), normals for each points
//
// Caller must allocate/deallocate memory for mNormals
//=========================================================================
template<typename DataType>
        void PCANormalImpl(DataType* mPoints , uint32_T* mIndices, uint32_T* mValid, uint32_T M, uint32_T numNeighbors, DataType* mNormals)
{
    DataType* xyzPoints = static_cast<DataType*>(mPoints);
    uint32_T * indices = static_cast<uint32_T*>(mIndices);
    uint32_T * valid = static_cast<uint32_T*>(mValid);
    
    DataType * normals = static_cast<DataType*>(mNormals);
    
    // Create buffer for neighboring points
    std::vector<DataType> neighbors(numNeighbors * 3);
    
    DataType normalVector[3]; // normal vector
    
    for (uint32_T i = 0; i < M; i++) {
        if (valid[i] < numNeighbors) {
            // Not enough neighbor points to apply PCA
            for (uint32_T d = 0; d < 3; d++)
                normals[d * M + i] = std::numeric_limits<DataType>::quiet_NaN();
            continue;
        }
        
        for (uint32_T k = 0; k < numNeighbors; k++) {
            // index is one-based
            uint32_T index = indices[i * numNeighbors + k] - 1;
            for (uint32_T d = 0; d < 3; d++)
                neighbors[k + d * numNeighbors] = xyzPoints[index + d * M];
        }
        
        vision::estimateNormalVectorWithPCA<DataType>(&(neighbors[0]), &(neighbors[0]), normalVector, numNeighbors);
        
        for (uint32_T d = 0; d < 3; d++)
            normals[d * M + i] = normalVector[d];
    }
    
}
#endif