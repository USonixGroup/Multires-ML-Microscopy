//////////////////////////////////////////////////////////////////////////////
//
// APIs for interfaces for scanContextDescriptorUtils that are used in codegeneration
// Copyright 2020 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
// vision_builtins does not need this source file

#ifndef COMPILE_FOR_VISION_BUILTINS

#include "scanContextDescriptorUtilsCore_api.hpp"
#include "scanContextDescriptorUtils.hpp"


void scanContextImageDescriptor_single(void* mPoints, const size_t numPoints, const size_t numRadialBins,
        const size_t numAzimuthalBins, const size_t minPointsPerBin, void* mSensorOrigin, void* mRadialRange,
        void* mDescriptor)
{
    const float* points = static_cast<const float*>(mPoints);
    const float* sensorOrigin = static_cast<const float*>(mSensorOrigin);
    float* radialRange = static_cast<float*>(mRadialRange);
    float* descriptor = static_cast<float*>(mDescriptor);
    vision::pc::scanContextImageDescriptor<float>(points, numPoints, numRadialBins, numAzimuthalBins,
            minPointsPerBin, sensorOrigin, radialRange, descriptor);
}

void scanContextImageDescriptor_double(void* mPoints, const size_t numPoints, const size_t numRadialBins,
        const size_t numAzimuthalBins, const size_t minPointsPerBin, void* mSensorOrigin, void* mRadialRange,
        void* mDescriptor)
{
    const double* points = static_cast<const double*>(mPoints);
    const double* sensorOrigin = static_cast<const double*>(mSensorOrigin);
    double* radialRange = static_cast<double*>(mRadialRange);
    double* descriptor = static_cast<double*>(mDescriptor);
    vision::pc::scanContextImageDescriptor<double>(points, numPoints, numRadialBins, numAzimuthalBins,
            minPointsPerBin, sensorOrigin, radialRange, descriptor);
}
#endif
