/* Copyright 2024 The MathWorks, Inc. */

#ifndef _READARUCOMARKERCORE_API_HPP
#define _READARUCOMARKERCORE_API_HPP

#include "vision_defines.h"

typedef struct{

    int adaptiveThreshWinSizeMin;
    int adaptiveThreshWinSizeMax;
    int adaptiveThreshWinSizeStep;
    double adaptiveThreshConstant;
    double minMarkerPerimeterRate;
    double maxMarkerPerimeterRate;
    double polygonalApproxAccuracyRate;
    double minCornerDistanceRate;
    int minDistanceToBorder;
    double minMarkerDistanceRate;
    int perspectiveRemovePixelPerCell;
    double perspectiveRemoveIgnoredMarginPerCell;
    int markerBorderBits;
    double minOtsuStdDev;
    double maxErroneousBitsInBorderRate;
    double errorCorrectionRate;
    bool detectInvertedMarker;
    bool useAruco3Detection;
    int minSideLengthCanonicalImg;
    float minMarkerLengthRatioOriginalImg;
    double cornerRefinementMethod;
    int cornerRefinementWinSize;
    int cornerRefinementMaxIterations;
    double cornerRefinementMinAccuracy;
} ArucoDetectorParams;

EXTERN_C LIBMWCVSTRT_API
uint32_T readArucoImplCore(void* mImgData,
                         const bool isRGB,
                         const int nRows, const int nCols,
                         void* mFamiliesData,
                         const int numFormats, 
                         const int* formatLengths,
                         const ArucoDetectorParams  mParams,
                         const bool doEstimatePose,
                         const double markerSize,
                         void* mCamMatrixPtr,
                         void* mDistCoeffsPtr, void** resultObj,
                         int* markerfamlen, int* rejectionLen,
                         int* poseLen);
EXTERN_C LIBMWCVSTRT_API
void assignOutputs(void* mIds, void* mLocs, void* mFamLength,
                   void* mFamilyNames, void* mRejections, void* mRotMat,
                   void* mTransVec, void* resultObj,
                   const bool doEstimatePose);
EXTERN_C LIBMWCVSTRT_API
void deleteResultPtr(void *ptrObj);

#endif
