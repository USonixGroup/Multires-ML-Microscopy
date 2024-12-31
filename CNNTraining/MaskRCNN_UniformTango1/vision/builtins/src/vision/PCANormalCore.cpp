//////////////////////////////////////////////////////////////////////////////
//
//   APIs for PCANormal
// Copyright 2018 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS

#include "PCANormalCore_api.hpp"
#include "PCANormal.hpp"

void PCANormalImpl_single(void* mPoints ,void* mIndices, void* mValid, uint32_T numPoints, uint32_T numNeighbors, void* mNormals)
{
    PCANormalImpl<float>(static_cast<float*>(mPoints) , static_cast<uint32_T*>(mIndices),
                                    static_cast<uint32_T*>(mValid), numPoints, numNeighbors, static_cast<float*>(mNormals));
}

void PCANormalImpl_double(void* mPoints ,void* mIndices, void* mValid,uint32_T numPoints, uint32_T numNeighbors, void* mNormals)
{
    PCANormalImpl<double>(static_cast<double*>(mPoints) , static_cast<uint32_T*>(mIndices),
                                    static_cast<uint32_T*>(mValid), numPoints, numNeighbors, static_cast<double*>(mNormals));
}
#endif