/* Copyright 2024 The MathWorks, Inc. */

#ifndef _ROIALIGNFORWARDCORE_
#define _ROIALIGNFORWARDCORE_

#include "vision_defines.h"

/* ROIALIGNFORWARD */
EXTERN_C LIBMWCVSTRT_API
void roiAlignForward(float const* X, float const* roi, uint32_T pooledHeight, uint32_T pooledWidth,
                     int samplingRatioX, int samplingRatioY, uint32_T xDim,
                     uint32_T yDim, uint32_T numChannels, uint32_T numROI, float* Z);
#endif