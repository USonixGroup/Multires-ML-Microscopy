/* Copyright 2024 The MathWorks, Inc. */
#ifndef VISION_RTW_TYPES
#define VISION_RTW_TYPES

/* Static version of rtwtypes.h that redirects to tmwtypes.h for building
 * this module.
 * 
 * rtwtypes.h is a header generated during codegen. Because this module 
 * generates headers for codegen, it needs to use rtwtypes.h instead of 
 * twmtypes.h. See this wiki for details:
 *
 * http://inside.mathworks.com/wiki/MATLAB_Coder_Codegen_Best_Practices
 *
 * However, because this module is also shared between sim and codegen, it
 * also needs to build with tmwtypes.h in the MATLAB context. Hence the
 * inclusion of tmwtypes.h here.
 */

#include "tmwtypes.h"

#endif
