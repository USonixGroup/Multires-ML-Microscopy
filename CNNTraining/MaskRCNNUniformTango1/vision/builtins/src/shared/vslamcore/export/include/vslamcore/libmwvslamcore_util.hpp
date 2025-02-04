/* Copyright 2023 The MathWorks, Inc. */

#ifndef _VSLAMCG_UTIL_
#define _VSLAMCG_UTIL_

#ifndef VSLAMROSCODEGEN 
#include "version.h"
#endif

#if defined(BUILDING_LIBMWVSLAMCORE)
  #define LIBMWVSLAMCORE_API DLL_EXPORT_SYM
#else
#ifdef VSLAMROSCODEGEN 
  #define LIBMWVSLAMCORE_API
#else
  #define LIBMWVSLAMCORE_API DLL_IMPORT_SYM
#endif
#endif

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#endif 
