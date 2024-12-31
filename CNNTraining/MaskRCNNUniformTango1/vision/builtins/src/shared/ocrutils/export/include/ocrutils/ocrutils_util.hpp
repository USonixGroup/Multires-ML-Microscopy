/* Copyright 2013-2024 The MathWorks, Inc. */
#ifndef _OCRUTILS_UTIL_
#define _OCRUTILS_UTIL_

#ifdef COMPILE_FOR_VISION_BUILTINS
#ifndef __arm__
#include "version.h"
#endif

#if defined(BUILDING_OCRUTILS)
	#define OCRUTILS_API DLL_EXPORT_SYM
#else
	#define OCRUTILS_API DLL_IMPORT_SYM
#endif
#endif

#include "tesseract/baseapi.h"

#ifndef OCRUTILS_API
#    define OCRUTILS_API
#endif

OCRUTILS_API
void resetGlobalParameters(tesseract::TessBaseAPI *);

#endif //ocrutils_util_hpp