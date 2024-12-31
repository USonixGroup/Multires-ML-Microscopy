/* Copyright 2013-2024 The MathWorks, Inc. */
////////////////////////////////////////////////////////////////////////////////
// This file contains utility functions to convert data to and from
// Leptonica's Pix format to MATLAB's mxArray format. Currently, only unit8
// datatypes and logical image types are supported.
////////////////////////////////////////////////////////////////////////////////

#ifndef _PIXUTILS_H_
#define _PIXUTILS_H_

#include "leptonica/allheaders.h"

#ifdef COMPILE_FOR_VISION_BUILTINS
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include "ocrutils_util.hpp"

#ifdef DEBUG_PIX 
////////////////////////////////////////////////////////////////////////////////
// Copy data from Pix to logical MATLAB data.
//   1) We need to convert from row major to column major.
////////////////////////////////////////////////////////////////////////////////
void copyPixToBoolean(const Pix * src, bool * dest);

////////////////////////////////////////////////////////////////////////////////
// Copy data from Pix to uint8 MATLAB data.
//   1) We need to convert from row major to column major.
////////////////////////////////////////////////////////////////////////////////
void copyPixToUint8(const Pix * src, uint8_T * dest);
#endif

////////////////////////////////////////////////////////////////////////////////
// Copy uint8 data from MATLAB into Leptonica's Pix data structure.
//   1) We need to convert column major data into row major data.
//   2) We use leptonica's SET_DATA_BYTE helper to copy data into Pix
////////////////////////////////////////////////////////////////////////////////
OCRUTILS_API
void copyMxUint8DataToPix(const uint8_T * src, Pix * pix,
                          const int32_T width, const int32_T height);

////////////////////////////////////////////////////////////////////////////////
// Copy logical data from MATLAB to leptonica's Pix format 
//    1) We need to convert from column-major to row-major
//    2) We need to bit pack boolean data into 1 bit-per-pixel format used by
//       leptonica.
//    3) We use leptonica's SET/CLEAR_DATA_BIT to set binary values
////////////////////////////////////////////////////////////////////////////////
OCRUTILS_API
void copyMxLogicalsToPix(const boolean_T * src, Pix * pix,
                         const int32_T width, const int32_T height);

#endif /* _PIXUTILS_H_ */
