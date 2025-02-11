///////////////////////////////////////////////////////////////////////////
// This file contains the source code to perform tuple test.
// Copyright 2020 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////

#include "tupleTest.hpp"
#include <numeric>
#include <cstdlib>
#include <cmath>
#include <vector>

// Compute Norm
template <typename T>
inline T computeNorm(const std::vector<T>& arr) {
    return std::sqrt(std::inner_product(arr.begin(), arr.end(), arr.begin(), T(0.0)));
}

// Find correspondences b/w points using spatial relation
template<typename T>
void findMatches(boolean_T *matchedInds, const T* points1, const T* points2,
    const uint32_T* randInds, uint32_T numPoints, uint32_T pointsDim, T scale, uint32_T numIters){

    std::vector<T> p1;
    p1.resize(pointsDim);
    std::vector<T> p2;
    p2.resize(pointsDim);
    std::vector<T> p3;
    p3.resize(pointsDim);
    std::vector<T> q1;
    q1.resize(pointsDim);
    std::vector<T> q2;
    q2.resize(pointsDim);
    std::vector<T> q3;
    q3.resize(pointsDim);

    T normp1, normp2, normp3, normq1, normq2, normq3;
    uint32_T randPtIdx1, randPtIdx2, randPtIdx3;

    // @TODO: Performance optimization using tbb::parallel_for
    for (uint32_T i = 0; i < numIters; i++) {
        // select 3 indices from randInds.
        randPtIdx1 = randInds[3*i+0];
        randPtIdx2 = randInds[3*i+1];
        randPtIdx3 = randInds[3*i+2];
        // Calculate the euclidean distance b/w slected correspondences in points1 and points2.
        for(uint32_T j = 0; j< pointsDim; j++){
            p1[j] = points1[randPtIdx1 + j * numPoints] - points1[randPtIdx2 + j * numPoints];
            p2[j] = points1[randPtIdx2 + j * numPoints] - points1[randPtIdx3 + j * numPoints];
            p3[j] = points1[randPtIdx3 + j * numPoints] - points1[randPtIdx1 + j * numPoints];

            q1[j] = points2[randPtIdx1 + j * numPoints] - points2[randPtIdx2 + j * numPoints];
            q2[j] = points2[randPtIdx2 + j * numPoints] - points2[randPtIdx3 + j * numPoints];
            q3[j] = points2[randPtIdx3 + j * numPoints] - points2[randPtIdx1 + j * numPoints];
        }

        normp1 = computeNorm(p1);
        normp2 = computeNorm(p2);
        normp3 = computeNorm(p3);

        normq1 = computeNorm(q1);
        normq2 = computeNorm(q2);
        normq3 = computeNorm(q3);

        // Correspondence triplets are considered as valid, if the ratio of distance b/w them
        // is within a given range(scale to 1/scale).
        if ((normp1 * scale < normq1) && (normq1 < normp1 / scale) && (normp2 * scale < normq2) &&
            (normq2 < normp2 / scale) && (normp3 * scale < normq3) && (normq3 < normp3 / scale)) {
            matchedInds[randPtIdx1] = true;
            matchedInds[randPtIdx2] = true;
            matchedInds[randPtIdx3] = true;
        }
    }
}

// Find matched indices for input points of type double
void findMatchesDouble(boolean_T *matchedInds, const double* points1,
    const double* points2, const uint32_T* randInds, uint32_T numPoints,
    uint32_T pointsDim, double scale, uint32_T numIters){
    findMatches(matchedInds, points1, points2, randInds, numPoints, pointsDim, scale, numIters);
}

// Find matched indices for input points of type single
void findMatchesSingle(boolean_T *matchedInds, const float* points1,
    const float* points2, const uint32_T* randInds, uint32_T numPoints,
    uint32_T pointsDim, float scale, uint32_T numIters){
    findMatches(matchedInds, points1, points2, randInds, numPoints, pointsDim, scale, numIters);
}
