/* Copyright 2018-2024 The MathWorks, Inc. */
#ifndef vision_defines_kdtree_h
#define vision_defines_kdtree_h

/* All symbols in this module are intentionally exported. */
#ifdef COMPILE_FOR_VISION_BUILTINS
#ifndef __arm__
#include "version.h"
#endif

#if defined(BUILDING_LIBMWKDTREE)
#define LIBMWKDTREE_API DLL_EXPORT_SYM
#else

#ifdef __arm__
#define LIBMWKDTREE_API
#else
#define LIBMWKDTREE_API DLL_IMPORT_SYM
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

#include "rtwtypes.h"
#endif

#else

#ifndef EXTERN_C
#  ifdef __cplusplus
#    define EXTERN_C extern "C"
#  else
#    define EXTERN_C extern
#  endif
#endif

#include "stddef.h"

#include "rtwtypes.h"

/* All symbols in this module are intentionally exported. */
#ifndef LIBMWCVSTRT_API
#ifdef _MSC_VER
#define LIBMWCVSTRT_API __declspec(dllexport)
#else
#define LIBMWCVSTRT_API
#endif
#endif

#endif

#endif