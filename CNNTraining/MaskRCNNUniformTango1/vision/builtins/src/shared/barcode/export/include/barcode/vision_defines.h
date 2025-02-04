/* Copyright 2021 The MathWorks, Inc. */
#ifndef vision_defines_h
#define vision_defines_h

/* All symbols in this module are intentionally exported. */

#ifndef __arm__
#include "version.h"
#endif

#if defined(BUILDING_LIBMWREADBARCODE)
    #define LIBMWREADBARCODE_API DLL_EXPORT_SYM
#else

#ifdef __arm__
    #define LIBMWREADBARCODE_API
#else
    #define LIBMWREADBARCODE_API DLL_IMPORT_SYM
#endif

#endif

#ifndef EXTERN_C
#  ifdef __cplusplus
#    define EXTERN_C extern "C"
#  else
#    define EXTERN_C extern
#  endif
#endif

#ifdef MATLAB_MEX_FILE
    #include "tmwtypes.h" /* mwSize is defined here */
#else
   #include "stddef.h"
   typedef size_t mwSize;  /* unsigned pointer-width integer */

   #include "rtwtypes.h"
#endif

#endif
