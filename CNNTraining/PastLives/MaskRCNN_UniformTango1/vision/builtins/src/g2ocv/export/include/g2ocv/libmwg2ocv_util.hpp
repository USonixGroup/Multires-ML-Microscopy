/* Copyright 2022 The MathWorks, Inc. */
#ifndef libmwg2ocv_util_hpp
#define libmwg2ocv_util_hpp

#include "version.h"
#if defined(BUILDING_G2OCV)
#define LIBMWG2OCV_API DLL_EXPORT_SYM
#else
#define LIBMWG2OCV_API DLL_IMPORT_SYM
#endif

#endif //libmwg2ocv_util_hpp