/* Copyright 2024 The MathWorks, Inc. */

/* Header file for insert ellipse codegen apis */
#ifndef _INSERTELLIPSE_
#define _INSERTELLIPSE_

#include "vision_defines.h"

EXTERN_C LIBMWCVSTRT_API
void insertEllipseCg_double(const double *img, const int32_T nRows, const int32_T nCols,  
        int32_T * positionOut, int32_T * axes, double * angle, 
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        double *imgOut);

EXTERN_C LIBMWCVSTRT_API
void insertEllipseCg_single(const float *img, const int32_T nRows, const int32_T nCols, 
        int32_T * positionOut, int32_T * axes, double * angle, 
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        float *imgOut);

EXTERN_C LIBMWCVSTRT_API
void insertEllipseCg_int16(const int16_T *img, const int32_T nRows, const int32_T nCols,  
        int32_T * positionOut, int32_T * axes, double * angle, 
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        int16_T *imgOut);

EXTERN_C LIBMWCVSTRT_API
void insertEllipseCg_uint8(const uint8_T *img, const int32_T nRows, const int32_T nCols,  
        int32_T * positionOut, int32_T * axes, double * angle, 
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        uint8_T *imgOut);

EXTERN_C LIBMWCVSTRT_API
void insertEllipseCg_uint16(const uint16_T *img, const int32_T nRows, const int32_T nCols,  
        int32_T * positionOut, int32_T * axes, double * angle, 
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        uint16_T *imgOut);
#endif
