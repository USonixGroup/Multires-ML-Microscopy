/* Copyright 2024 The MathWorks, Inc. */

/* Codegen implementation for inserting ellipse shape using opencv */
#ifndef COMPILE_FOR_VISION_BUILTINS
#include "insertEllipseCore_api.hpp"
#include "opencv2/opencv.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "cgCommon.hpp"
#include "insertEllipseAlg.hpp"

using namespace cv;

    template <typename ImageDataType>
void insertEllipse(const ImageDataType *img, const int32_T nRows, const int32_T nCols, 
        int32_T * positionOut, int32_T *axes, double *angle,
        const boolean_T fillShape, const real_T lineWidth,
        double *lineColor, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        ImageDataType *imgOut)
{
    int thickness = fillShape ? -1 : (int32_T)lineWidth;
    int lineType            = smoothEdges ? cv::LINE_AA : cv::LINE_8;
    constexpr double startAngle   = 0;
    constexpr double endAngle     = 360;
    
    bool isRGB = true;
    cv::Ptr<cv::Mat> inImage = new cv::Mat;
    if(isRowMajor)
        cArrayToMat_RowMaj<ImageDataType>(img, nRows, nCols, isRGB, *inImage);
    else
        cArrayToMat<ImageDataType>(img, nRows, nCols, isRGB, *inImage);

    drawMultipleEllipses(*inImage, positionOut, axes, angle, numRows, startAngle, endAngle, lineColor, opacity, thickness, lineType, shift);
    if(isRowMajor)
        cArrayFromMat_RowMaj<ImageDataType>(imgOut, *inImage);
    else
        cArrayFromMat<ImageDataType>(imgOut, *inImage);
}

///////////////////////////////////////////////////////////////////////////
void insertEllipseCg_double(const double *img, const int32_T nRows, const int32_T nCols,  
        int32_T * positionOut, int32_T *axes, double *angle,
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        double *imgOut)
{
    insertEllipse<real_T>(img, nRows, nCols, positionOut, axes, angle, fillShape, lineWidth,
            color, opacity, smoothEdges, shift, numRows, isRowMajor,
            imgOut);
}

///////////////////////////////////////////////////////////////////////////
void insertEllipseCg_single(const float *img, const int32_T nRows, const int32_T nCols,  
        int32_T * positionOut, int32_T *axes, double *angle,
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        float *imgOut)
{
    insertEllipse<real32_T>(img, nRows, nCols, positionOut, axes, angle,  fillShape, lineWidth,
            color, opacity, smoothEdges, shift, numRows, isRowMajor,
            imgOut);
}

///////////////////////////////////////////////////////////////////////////
void insertEllipseCg_int16(const int16_T *img, const int32_T nRows, const int32_T nCols, 
        int32_T * positionOut, int32_T *axes, double *angle,
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        int16_T *imgOut)
{
    insertEllipse<int16_T>(img, nRows, nCols, positionOut, axes, angle, fillShape, lineWidth,
            color, opacity, smoothEdges, shift, numRows, isRowMajor,
            imgOut);
}

///////////////////////////////////////////////////////////////////////////
void insertEllipseCg_uint8(const uint8_T *img, const int32_T nRows, const int32_T nCols,  
        int32_T * positionOut, int32_T *axes, double *angle,
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        uint8_T *imgOut)
{
    insertEllipse<uint8_T>(img, nRows, nCols, positionOut, axes, angle,  fillShape, lineWidth,
            color, opacity, smoothEdges, shift, numRows, isRowMajor,
            imgOut);
}

///////////////////////////////////////////////////////////////////////////
void insertEllipseCg_uint16(const uint16_T *img, const int32_T nRows, const int32_T nCols,  
        int32_T * positionOut, int32_T *axes, double *angle,
        const boolean_T fillShape, const real_T lineWidth,
        double *color, const double opacity, 
        const boolean_T smoothEdges, const int shift, const int numRows, const boolean_T isRowMajor, 
        uint16_T *imgOut)
{
    insertEllipse<uint16_T>(img, nRows, nCols, positionOut, axes, angle, fillShape, lineWidth,
            color, opacity, smoothEdges, shift, numRows, isRowMajor,
            imgOut);
}
#endif
