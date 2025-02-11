/* Copyright 2021 The MathWorks, Inc. */

#ifndef _VISIONSBASCHURCOMPLEMENTCORE_
#define _VISIONSBASCHURCOMPLEMENTCORE_

#include "vision_defines.h"
#include <stdint.h>

EXTERN_C LIBMWCVSTRT_API
    void visionSBASchurComplement(void*, void*, void*, void*,
                                 void*, const int32_t*, const int32_t*,
                                 const size_t, const size_t, 
                                 void*, void*, void*);
#endif