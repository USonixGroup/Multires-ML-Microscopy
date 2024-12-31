///////////////////////////////////////////////////////////////////////////
//
//    Function to estimate normal vector with PCA
//    Copyright 2018 The MathWorks, Inc.
//
///////////////////////////////////////////////////////////////////////////
#ifndef NORMALVECTOR_CORE_HPP
#define NORMALVECTOR_CORE_HPP

#include "realSymmetricEigenSolver_core.hpp"

namespace vision
{
    //========================================================================================
    // Estimate normal vector with PCA
    //
    // estimateNormalVectorWithPCA(xyzPoints, normPoints, normalVector, numPoints)
    //
    // Parameters:
    //   xyzPoints		: input X-Y-Z column major Nx3 matrix
    //   normPoints		: buffer for storing the normalized points, a Nx3 matrix
    //   normalVector	: buffer for normal vector, 1x3 vector
    //   numPoints      : number of points
    //
    // Caller must allocate/deallocate memory for normPoints and normalVector
    //========================================================================================
    template <typename T>
    void estimateNormalVectorWithPCA(T* xyzPoints, T* normPoints, T* normalVector,
            uint32_T numPoints, const uint32_T dims = 3)
    {
        T meanValue[3];
        T covariance[9];
        T eigenValue[3];
        T eigenVector[9];
        
        // Center the input data
        for (uint32_T d = 0; d < dims; d++)
            meanValue[d] = 0;
        
        for (uint32_T d = 0; d < dims; d++) {
            uint32_T j = d * numPoints;
            for (uint32_T k = 0; k < numPoints; k++)
                meanValue[d] += xyzPoints[j + k];
        }
        
        for (uint32_T d = 0; d < dims; d++)
            meanValue[d] /= numPoints;
        
        for (uint32_T d = 0; d < dims; d++) {
            uint32_T j = d * numPoints;
            for (uint32_T k = 0; k < numPoints; k++)
                normPoints[j + k] = xyzPoints[j + k] - meanValue[d];
        }
        
        // Compute the covariance matrix
        for (uint32_T d = 0; d < dims; d++) {
            uint32_T m = d * numPoints;
            uint32_T n = 0;
            for (uint32_T j = 0; j <= d; j++) {
                T val = 0;
                n = j * numPoints;
                for (uint32_T k = 0; k < numPoints; k++)
                    val += normPoints[m + k] * normPoints[n + k];
                covariance[j * 3 + d] = val;
            }
        }
        
        // Make sure this is symmetric
        for (uint32_T d = 0; d < dims; d++) {
            for (uint32_T n = d + 1; n < 3; n++)
                covariance[n * dims + d] = covariance[d * dims + n];
        }
        
        // Find the eigen values and vectors
        vision::eig3(covariance, eigenValue, eigenVector);
        
        // Find the eigen vector associated with the minimum eigen value
        T minEigenValue = eigenValue[0];
        uint32_T idx = 0;
        for (uint32_T k = 1; k < 3; k++) {
            if (eigenValue[k] < minEigenValue) {
                minEigenValue = eigenValue[k];
                idx = k;
            }
        }
        
        for (uint32_T d = 0; d < 3; d++)
            normalVector[d] = eigenVector[idx * 3 + d];
        
    }
}

#endif