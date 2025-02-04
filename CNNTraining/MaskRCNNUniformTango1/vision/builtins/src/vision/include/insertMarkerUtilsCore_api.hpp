/* Copyright 2019 The MathWorks, Inc. */

#ifndef _INSERTMARKERUTILSCORE_API_HPP
#define _INSERTMARKERUTILSCORE_API_HPP

#include "vision_defines.h"

/* Declarations of Insert Marker APIs */

EXTERN_C LIBMWCVSTRT_API
        void constructDrawBaseObjectMarker(void **ptr2dbObjPtr);

EXTERN_C LIBMWCVSTRT_API
        boolean_T initialiseDrawbaseMarker(void *ptrObj, const int i);

EXTERN_C LIBMWCVSTRT_API
        void instantiateDrawBaseMarker_single(void *ptrObj, void *rgbOut, void *rgbIn, void *pts, 
        void *fillColor, double mOpacity, int32_T markerShape, int32_T markerSize, int_T numRow, 
        int_T numCol, int_T numColor, int_T numDimens, int32_T dimens1, int32_T dimens2, 
        int_T numFillColor, void* pixCount, int objCounter);

EXTERN_C LIBMWCVSTRT_API
        void instantiateDrawBaseMarker_double(void *ptrObj, void *rgbOut, void *rgbIn, void *pts, 
        void *fillColor, double mOpacity, int32_T markerShape, int32_T markerSize, int_T numRow,
        int_T numCol, int_T numColor, int_T numDimens, int32_T dimens1, int32_T dimens2, 
        int_T numFillColor, void* pixCount, int objCounter);

EXTERN_C LIBMWCVSTRT_API
        void instantiateDrawBaseMarker_uint8(void *ptrObj, void *rgbOut, void *rgbIn, void *pts, 
        void *fillColor, double mOpacity, int32_T markerShape, int32_T markerSize, int_T numRow,
        int_T numCol, int_T numColor, int_T numDimens, int32_T dimens1, int32_T dimens2, 
        int_T numFillColor, void* pixCount, int objCounter);

EXTERN_C LIBMWCVSTRT_API
        void instantiateDrawBaseMarker_uint16(void *ptrObj, void *rgbOut, void *rgbIn, void *pts, 
        void *fillColor, double mOpacity, int32_T markerShape, int32_T markerSize, int_T numRow,
        int_T numCol, int_T numColor, int_T numDimens, int32_T dimens1, int32_T dimens2, 
        int_T numFillColor, void* pixCount, int objCounter);

EXTERN_C LIBMWCVSTRT_API
        void instantiateDrawBaseMarker_int16(void *ptrObj, void *rgbOut, void *rgbIn, void *pts, 
        void *fillColor, double mOpacity, int32_T markerShape, int32_T markerSize, int_T numRow,
        int_T numCol, int_T numColor, int_T numDimens, int32_T dimens1, int32_T dimens2, 
        int_T numFillColor, void* pixCount, int objCounter);

EXTERN_C LIBMWCVSTRT_API
        void deallocateMemoryMarker(void *ptrObj);

#endif