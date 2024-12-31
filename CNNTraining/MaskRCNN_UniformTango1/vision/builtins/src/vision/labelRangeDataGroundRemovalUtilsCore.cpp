//////////////////////////////////////////////////////////////////////////////
//
// APIs for interfaces for labelRangeDataGroundRemovalUtils that are used in codegeneration
// Copyright 2019 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////

#include "labelRangeDataGroundRemovalUtilsCore_api.hpp"
#include "labelRangeDataGroundRemovalUtils.hpp"

void preprocessAndSegmentGround_single(const float* mRange, const int nRows, const int nCols, void* mLabels, 
        const float seed_threshold, const float angle_threshold, const float depth_threshold)
{
    float* repairedRange = (float*)malloc(sizeof(float) * nRows * nCols * 3);
    
    memcpy(repairedRange, mRange, sizeof(float) * nRows * nCols * 2);
    preprocessRangeData<float>(mRange, nRows, nCols, repairedRange, depth_threshold);
    
    segmentGround<float>(repairedRange, nRows, nCols, static_cast<bool*>(mLabels),
            seed_threshold, angle_threshold);
    
    free(repairedRange);
    
}
void preprocessAndSegmentGround_double(const double* mRange, const int nRows, const int nCols, void* mLabels,
        const double seed_threshold, const double angle_threshold, const double depth_threshold)
{
    double* repairedRange = (double*)malloc(sizeof(double) * nRows * nCols * 3);
    
    memcpy(repairedRange, mRange, sizeof(double) * nRows * nCols * 2);
    preprocessRangeData<double>(mRange, nRows, nCols, repairedRange, depth_threshold);
    
    segmentGround<double>(repairedRange, nRows, nCols, static_cast<bool*>(mLabels),
            seed_threshold, angle_threshold);
    
    free(repairedRange);
    
}