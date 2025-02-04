/* Copyright 2020-2023 The MathWorks, Inc. */

#ifndef _SCANCONTEXTDESCRIPTORUTILSCORE_API_HPP
#define _SCANCONTEXTDESCRIPTORUTILSCORE_API_HPP

#include "vision_defines.h"
#include "stddef.h"

/* ScanContextImageDescriptor */

EXTERN_C LIBMWCVSTRT_API
       void scanContextImageDescriptor_single(void* mPoints, const size_t numPoints, const size_t numRadialBins,
        const size_t numAzimuthalBins, const size_t minPointsPerBin, void* mSensorOrigin, void* mRadialRange,
        void* mDescriptor);

EXTERN_C LIBMWCVSTRT_API
        void scanContextImageDescriptor_double(void* mPoints, const size_t numPoints, const size_t numRadialBins,
        const size_t numAzimuthalBins, const size_t minPointsPerBin, void* mSensorOrigin, void* mRadialRange,
        void* mDescriptor);

#endif