/* Copyright 2013-2024 The MathWorks, Inc. */
////////////////////////////////////////////////////////////////////////////////
// This file contains utility functions to convert data to and from
// Leptonica's Pix format to MATLAB's mxArray format. Currently, only unit8
// datatypes and logical image types are supported.
////////////////////////////////////////////////////////////////////////////////

// REVIEW: moved pix_utils.cpp from ocrutils/export to ocrutils/
#include "ocrutils/pix_utils.hpp"

#include "leptonica/allheaders.h"

#ifdef COMPILE_FOR_VISION_BUILTINS
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#ifdef DEBUG_PIX 
////////////////////////////////////////////////////////////////////////////////
// Copy data from Pix to logical MATLAB data.
//   1) We need to convert from row major to column major.
////////////////////////////////////////////////////////////////////////////////
void copyPixToBoolean(const Pix * src, bool * dest)
{
    uint32_T width  = src->w;
    uint32_T height = src->h;
    uint32_T wordsPerLine = src->wpl;  // 32-bit words per line
    uint32_T * pixData    = src->data; // pix data pointer is uint32 type
    for (uint32_T j = 0; j < height; ++j)
    {           
        for (uint32_T i = 0; i < width; ++i)
        {
            // row major to column major
            // use leptonica's GET_DATA_BIT helper to read packed bits
#ifdef _WIN32
            // fix compiler warning c4800
            // on windows GET_DATA_BIT is a function whereas on linux
            // it is an inlined macro
            dest[j + i*height] = static_cast<bool>(GET_DATA_BIT(pixData,i) != 0);
#else
            dest[j + i*height] = static_cast<bool>(GET_DATA_BIT(pixData,i));
#endif
        }
        pixData += wordsPerLine;
    }      

}

////////////////////////////////////////////////////////////////////////////////
// Copy data from Pix to uint8 MATLAB data.
//   1) We need to convert from row major to column major.
////////////////////////////////////////////////////////////////////////////////
void copyPixToUint8(const Pix * src, uint8_T * dest)
{
    uint32_T width  = src->w;
    uint32_T height = src->h;

    uint32_T bytesPerLine = 4 * src->wpl;
   
    uint8_T * pixData = (uint8_T *)(src->data); // pix data pointer is uint32 type

    // row-major to column-major copy
    for (uint32_T j = 0; j < height; ++j)
    {
        for (uint32_T i = 0; i < width; ++i)
        {
            dest[j + i*height] = pixData[i];
        }
        pixData += bytesPerLine; // 32-bit alignment for each row in Pix
    }
    
}
#endif

////////////////////////////////////////////////////////////////////////////////
// Copy uint8 data from MATLAB into Leptonica's Pix data structure.
//   1) We need to convert column major data into row major data.
//   2) We use leptonica's SET_DATA_BYTE helper to copy data into Pix
////////////////////////////////////////////////////////////////////////////////
void copyMxUint8DataToPix(const uint8_T * src, Pix * pix,
                     const int32_T width, const int32_T height)
{
    uint32_T * line = pixGetData(pix);

    // row-major to column-major copy
    for (int32_T j = 0; j < height; ++j)
    {
        for (int32_T i = 0; i < width; ++i)
        {
            SET_DATA_BYTE(line,i,src[i*height + j]);
        }            
        line += pixGetWpl(pix); // go to next line
    }
}

////////////////////////////////////////////////////////////////////////////////
// Copy logical data from MATLAB to leptonica's Pix format 
//    1) We need to convert from column-major to row-major
//    2) We need to bit pack boolean data into 1 bit-per-pixel format used by
//       leptonica.
//    3) We use leptonica's SET/CLEAR_DATA_BIT to set binary values
////////////////////////////////////////////////////////////////////////////////
void copyMxLogicalsToPix(const boolean_T * src, Pix * pix,
                     const int32_T width, const int32_T height)
{
    uint32_T * line = pixGetData(pix);
    
    // copy column major data into row major data
    for (int32_T j = 0; j < height; ++j)
    {           
        for (int32_T i = 0; i < width; ++i)
        {            
            if (src[j+ i*height])
            {
                SET_DATA_BIT(line,i);
            }
            else
            {
                CLEAR_DATA_BIT(line,i);
            }
        }
        line += pixGetWpl(pix); // go to next line
    }
}

