// //////////////////////////////////////////////////////////////////////////////
// //
// // APIs for interfaces for insertMarkerUtils that are used for codegeneration
// // Copyright 2019-2022 The MathWorks, Inc.
// //
// //////////////////////////////////////////////////////////////////////////////

#ifndef COMPILE_FOR_VISION_BUILTINS

#include "insertMarkerUtilsCore_api.hpp"
#include "insertShapeAndMarkerUtils.hpp"


void constructDrawBaseObjectMarker(void** ptr2dbObjPtr) {
    DrawBase** drawBaseP2P;
    drawBaseP2P = new DrawBase*[2];
    *ptr2dbObjPtr = drawBaseP2P;
}

boolean_T initialiseDrawbaseMarker(void* ptrObj, const int i) {
    DrawBase** drawBaseP2P = (DrawBase**)ptrObj;
    boolean_T isInitialise = false;
    if (i != 0) {
        drawBaseP2P[i] = NULL;
        isInitialise = true;
    }
    return isInitialise;
}

void instantiateDrawBaseMarker_single(void* ptrObj,
                                      void* rgbOut,
                                      void* rgbIn,
                                      void* pts,
                                      void* fillColor,
                                      double mOpacity,
                                      int32_T markerShape,
                                      int32_T markerSize,
                                      int_T numRow,
                                      int_T numCol,
                                      int_T numColor,
                                      int_T numDimens,
                                      int32_T dimens1,
                                      int32_T dimens2,
                                      int_T numFillColor,
                                      void* pixCount,
                                      int objCounter) {
    DrawBase** drawBaseP2P = (DrawBase**)ptrObj;

    // Copy the inputImage to outImage
    int_T numPixels = numCol * numRow * 3;
    memcpy(rgbOut, rgbIn, numPixels * sizeof(real32_T));

    int_T dimens[2] = {(int_T)dimens1, (int_T)dimens2};
    real32_T opacity = (real32_T)(mOpacity);

    // Instantiate the drawBase Object for Marker
    drawBaseP2P[objCounter] =
        InstantiateMarker<real32_T, real32_T, int32_T, int32_T, real32_T, real32_T>(
            static_cast<real32_T*>(rgbOut), static_cast<int32_T*>(pts), &opacity,
            static_cast<real32_T*>(fillColor), markerShape, &markerSize, numRow, numCol, numColor,
            numFillColor, numDimens, dimens, static_cast<uint8_T*>(pixCount));

    // Draws the Marker
    drawBaseP2P[objCounter]->ProcessStep();
}

void instantiateDrawBaseMarker_double(void* ptrObj,
                                      void* rgbOut,
                                      void* rgbIn,
                                      void* pts,
                                      void* fillColor,
                                      double mOpacity,
                                      int32_T markerShape,
                                      int32_T markerSize,
                                      int_T numRow,
                                      int_T numCol,
                                      int_T numColor,
                                      int_T numDimens,
                                      int32_T dimens1,
                                      int32_T dimens2,
                                      int_T numFillColor,
                                      void* pixCount,
                                      int objCounter) {
    DrawBase** drawBaseP2P = (DrawBase**)ptrObj;

    // Copy the inputImage to outImage
    int_T numPixels = numCol * numRow * 3;
    memcpy(rgbOut, rgbIn, numPixels * sizeof(real_T));

    int_T dimens[2] = {(int_T)dimens1, (int_T)dimens2};
    real_T opacity = (real_T)(mOpacity);

    // Instantiate the drawBase Object for Marker
    drawBaseP2P[objCounter] = InstantiateMarker<real_T, real_T, int32_T, int32_T, real_T, real_T>(
        static_cast<real_T*>(rgbOut), static_cast<int32_T*>(pts), &opacity,
        static_cast<real_T*>(fillColor), markerShape, &markerSize, numRow, numCol, numColor,
        numFillColor, numDimens, dimens, static_cast<uint8_T*>(pixCount));
    // Draws the Marker
    drawBaseP2P[objCounter]->ProcessStep();
}

void instantiateDrawBaseMarker_uint8(void* ptrObj,
                                     void* rgbOut,
                                     void* rgbIn,
                                     void* pts,
                                     void* fillColor,
                                     double mOpacity,
                                     int32_T markerShape,
                                     int32_T markerSize,
                                     int_T numRow,
                                     int_T numCol,
                                     int_T numColor,
                                     int_T numDimens,
                                     int32_T dimens1,
                                     int32_T dimens2,
                                     int_T numFillColor,
                                     void* pixCount,
                                     int objCounter) {
    DrawBase** drawBaseP2P = (DrawBase**)ptrObj;

    // Copy the inputImage to outImage
    int_T numPixels = numCol * numRow * 3;
    memcpy(rgbOut, rgbIn, numPixels * sizeof(uint8_T));

    int_T dimens[2] = {(int_T)dimens1, (int_T)dimens2};
    real32_T opacity = (real32_T)(mOpacity);

    // Instantiate the drawBase Object for Marker
    drawBaseP2P[objCounter] =
        InstantiateMarker<uint8_T, uint8_T, int32_T, int32_T, real32_T, real32_T>(
            static_cast<uint8_T*>(rgbOut), static_cast<int32_T*>(pts), &opacity,
            static_cast<uint8_T*>(fillColor), markerShape, &markerSize, numRow, numCol, numColor,
            numFillColor, numDimens, dimens, static_cast<uint8_T*>(pixCount));
    // Draws the Marker
    drawBaseP2P[objCounter]->ProcessStep();
}

void instantiateDrawBaseMarker_uint16(void* ptrObj,
                                      void* rgbOut,
                                      void* rgbIn,
                                      void* pts,
                                      void* fillColor,
                                      double mOpacity,
                                      int32_T markerShape,
                                      int32_T markerSize,
                                      int_T numRow,
                                      int_T numCol,
                                      int_T numColor,
                                      int_T numDimens,
                                      int32_T dimens1,
                                      int32_T dimens2,
                                      int_T numFillColor,
                                      void* pixCount,
                                      int objCounter) {
    DrawBase** drawBaseP2P = (DrawBase**)ptrObj;

    // Copy the inputImage to outImage
    int_T numPixels = numCol * numRow * 3;
    memcpy(rgbOut, rgbIn, numPixels * sizeof(uint16_T));

    int_T dimens[2] = {(int_T)dimens1, (int_T)dimens2};
    real32_T opacity = (real32_T)(mOpacity);

    // Instantiate the drawBase Object for Marker
    drawBaseP2P[objCounter] =
        InstantiateMarker<uint16_T, uint16_T, int32_T, int32_T, real32_T, real32_T>(
            static_cast<uint16_T*>(rgbOut), static_cast<int32_T*>(pts), &opacity,
            static_cast<uint16_T*>(fillColor), markerShape, &markerSize, numRow, numCol, numColor,
            numFillColor, numDimens, dimens, static_cast<uint8_T*>(pixCount));
    // Draws the Marker
    drawBaseP2P[objCounter]->ProcessStep();
}

void instantiateDrawBaseMarker_int16(void* ptrObj,
                                     void* rgbOut,
                                     void* rgbIn,
                                     void* pts,
                                     void* fillColor,
                                     double mOpacity,
                                     int32_T markerShape,
                                     int32_T markerSize,
                                     int_T numRow,
                                     int_T numCol,
                                     int_T numColor,
                                     int_T numDimens,
                                     int32_T dimens1,
                                     int32_T dimens2,
                                     int_T numFillColor,
                                     void* pixCount,
                                     int objCounter) {
    DrawBase** drawBaseP2P = (DrawBase**)ptrObj;

    // Copy the inputImage to outImage
    int_T numPixels = numCol * numRow * 3;
    memcpy(rgbOut, rgbIn, numPixels * sizeof(int16_T));

    int_T dimens[2] = {(int_T)dimens1, (int_T)dimens2};
    real32_T opacity = (real32_T)(mOpacity);

    // Instantiate the drawBase Object for Marker
    drawBaseP2P[objCounter] =
        InstantiateMarker<int16_T, int16_T, int32_T, int32_T, real32_T, real32_T>(
            static_cast<int16_T*>(rgbOut), static_cast<int32_T*>(pts), &opacity,
            static_cast<int16_T*>(fillColor), markerShape, &markerSize, numRow, numCol, numColor,
            numFillColor, numDimens, dimens, static_cast<uint8_T*>(pixCount));
    // Draws the Marker
    drawBaseP2P[objCounter]->ProcessStep();
}

void deallocateMemoryMarker(void* ptrObj) {
    DrawBase** drawBaseP2P = (DrawBase**)ptrObj;
    if (drawBaseP2P) {
        if (drawBaseP2P[0]) {
            delete drawBaseP2P[0];
        }
        if (drawBaseP2P[1]) {
            delete drawBaseP2P[1];
        }
        delete[] drawBaseP2P;
    }
}

#endif
