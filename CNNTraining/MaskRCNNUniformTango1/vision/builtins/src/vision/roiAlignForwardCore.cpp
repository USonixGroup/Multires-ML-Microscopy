//////////////////////////////////////////////////////////////////////////////
//
//   APIs for roiAlignForward
// Copyright 2024 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS

#include "roiAlignForwardCore_api.hpp"
#include "roiAlignForward.hpp"

void roiAlignForward(float const* X, float const* roi, const uint32_T pooledHeight,
                     const uint32_T pooledWidth, const int32_T samplingRatioX,
                     const int32_T samplingRatioY, const uint32_T xDim, const uint32_T yDim,
                     const uint32_T numChannels, const uint32_T numROI,
                     float* Z)
{
    float* output = Z;
    const float* src = X;

    pooledDlDataImpl(roi, output, src, samplingRatioX, samplingRatioY, numChannels, numROI, pooledHeight, pooledWidth, xDim, yDim);
}


#endif