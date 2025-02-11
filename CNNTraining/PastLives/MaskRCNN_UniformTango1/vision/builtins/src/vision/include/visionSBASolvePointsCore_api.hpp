/* Copyright 2021 The MathWorks, Inc. */

#ifndef _VISIONSBASOLVEPOINTSCORE_
#define _VISIONSBASOLVEPOINTSCORE_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API
    void visionSBASolvePoints(void*, void*, void*, void*,
                             const int*, const int*, const size_t,
                             const size_t, void*, void*);
#endif