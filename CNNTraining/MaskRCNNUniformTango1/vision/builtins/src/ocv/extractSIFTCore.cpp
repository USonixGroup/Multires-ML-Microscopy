//////////////////////////////////////////////////////////////////////////////
// OpenCV SIFT extractor wrapper
//
// Copyright 2021 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS

#include "extractSIFTCore_api.hpp"
#include "xfeatures2d_sift_mw.hpp"

#include "opencv2/opencv.hpp"
#include "cgCommon.hpp"

using namespace std;

//////////////////////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////////////////////

void fieldsToSIFTKeyPoints(
    real32_T *inLoc, 
    real32_T *inScl,
    real32_T *inMet,
    real32_T *inOri,
    int32_T  *inOct,
    int32_T  *inLay,
    bool isColumnMajor, int numKPts,
    vector<cv::KeyPoint> & keypoints){
    keypoints.reserve(numKPts);
    for (int i = 0; i < numKPts; i ++){
        float x, y;
        // convert to C/C++ 0-indexing
        if (isColumnMajor){  // column major
            x = float(inLoc[i]) - 1.0f;
            y = float(inLoc[numKPts + i]) - 1.0f;
        }
        else{       // row major
            x = float(inLoc[i*2]) - 1.0f;
            y = float(inLoc[i*2 + 1]) - 1.0f;
        }
        float orientation = inOri[i]; // let extractor assign actual values
        
        // size is the diameter of the keypoint region
        // Scale is equal to the radius of the keypoint region
        float size = inScl[i] * 2.0f;
        
        float metric = inMet[i];
        int   oct = inOct[i] + (inLay[i] << 8);
        cv::KeyPoint kpt(x, y, size, orientation, metric, oct);
        keypoints.push_back(kpt);
    }
}

void siftKeyPointsToStructFields(
    void *ptrKeypoints,
    real32_T *outLoc, 
    real32_T *outScl,
    real32_T *outMet,
    real32_T *outOri,
    int32_T  *outOct,
    int32_T  *outLay,
    bool isColumnMajor)
{
    vector<cv::KeyPoint> &keypoints = ((vector<cv::KeyPoint> *)ptrKeypoints)[0];
    size_t m = keypoints.size();

    for(size_t i = 0; i < m; i++ ) {
        
        cv::KeyPoint& kp = keypoints[i];
        
        // convert C 0-index to MATLAB 1-index
        if (isColumnMajor){  // column major
            outLoc[i]   = kp.pt.x + 1.0f;
            outLoc[m+i] = kp.pt.y + 1.0f;
        }else{      // row major
            *outLoc++   = kp.pt.x + 1.0f;
            *outLoc++   = kp.pt.y + 1.0f;
        }
        
        outOri[i] = kp.angle *(float)(CV_PI/180.0); // angle to radian, should be >= 0 according to OpenCV implementation.
        outMet[i] = kp.response;  // metric (float)
        
        // size is the diameter of the keypoint region
        // Scale is equal to the radius of the keypoint region
        outScl[i] = kp.size * 0.5f;
        
        outOct[i] = static_cast<int32_T>(( kp.octave & 255 ));
        outLay[i] = static_cast<int32_T>(( (kp.octave >> 8) & 255 ));
    }
    
    delete((vector<cv::KeyPoint> *)ptrKeypoints);
}

void siftFeaturesMatToVector(
        void *ptrDescriptors,
        real32_T *outFeatures,
        bool isColumnMajor)
{
    cv::Mat descriptors = *((cv::Mat *)ptrDescriptors);
    if (isColumnMajor){
        cArrayFromMat<real32_T>(outFeatures, descriptors);
    }
    else
    {
        cArrayFromMat_RowMaj<real32_T>(outFeatures, descriptors);
    }
    delete((cv::Mat *)ptrDescriptors);
}


//////////////////////////////////////////////////////////////////////////////
// Invoke OpenCV SIFT extractor
//////////////////////////////////////////////////////////////////////////////

int32_T extractSIFTCompute(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inScl,
    real32_T *inMet,
    real32_T *inOri,
    int32_T  *inOct,
    int32_T  *inLay,
    int numKPts,
    void **outKeypoints, void **outFeatures, bool isColumnMajor)
{
    
    // Use OpenCV smart pointer to manage image 
    cv::Ptr<cv::Mat> inImage = new cv::Mat;
    bool isRGB = false; // only grayscale image allowed
    
    if (isColumnMajor){
        cArrayToMat<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }else{
        cArrayToMat_RowMaj<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }
    
    // Create KeyPoint array from the given fields
    vector<cv::KeyPoint> *ptrKeypoints = (vector<cv::KeyPoint> *)new vector<cv::KeyPoint>();
    *outKeypoints = ptrKeypoints;
    vector<cv::KeyPoint> &refKeypoints = *ptrKeypoints;
    
    fieldsToSIFTKeyPoints(
            inLoc, inScl, inMet, inOri, inOct, inLay, isColumnMajor, numKPts, refKeypoints);
    // Define the feature pointer
    cv::Mat * ptrFeatures = new cv::Mat;
    *outFeatures = ptrFeatures;
    cv::Mat & refFeatures = *ptrFeatures;
    
    // invoke the OpenCV SIFT
    try
    {
        cv::Ptr<MWSIFT> sift = cv::makePtr<MWSIFT>();
        sift->compute(*inImage, refKeypoints, refFeatures);
    }
    catch (...)
    {
        CV_Error(cv::Error::StsNotImplemented, "OpenCV was built without SIFT support");
    }

    return ((int32_T)(refKeypoints.size())); //actual_numel
}


int32_T extractSIFTComputeColumnMajor(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inScl,
    real32_T *inMet,
    real32_T *inOri,
    int32_T  *inOct,
    int32_T  *inLay,
    int numKPts,
    void **outKeypoints, void **outFeatures){
    return extractSIFTCompute(inImg, nRows, nCols,
            inLoc, inScl, inMet, inOri, inOct, inLay, numKPts,
            outKeypoints, outFeatures, true);
}

int32_T extractSIFTComputeRowMajor(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inScl,
    real32_T *inMet,
    real32_T *inOri,
    int32_T  *inOct,
    int32_T  *inLay,
    int numKPts,
    void **outKeypoints, void **outFeatures){
    return extractSIFTCompute(inImg, nRows, nCols,
            inLoc, inScl, inMet, inOri, inOct, inLay, numKPts,
            outKeypoints, outFeatures, false);
}

//////////////////////////////////////////////////////////////////////////////
// Assign KeyPoint to Fields and Descriptor Mat to Array
//////////////////////////////////////////////////////////////////////////////

void extractSIFTAssignOutputColumnMajor(
    void * ptrKeypoints, 
    void * ptrFeatures,
    real32_T *outLoc, 
    real32_T *outScl,
    real32_T *outMet,
    real32_T *outOri,
    int32_T  *outOct,
    int32_T  *outLay,
    real32_T *outFtrs)
{
    siftKeyPointsToStructFields(
            ptrKeypoints, outLoc, outScl, outMet, outOri, outOct, outLay, true);
    siftFeaturesMatToVector(
            ptrFeatures, outFtrs, true);
}

void extractSIFTAssignOutputRowMajor(
    void * ptrKeypoints, 
    void * ptrFeatures,
    real32_T *outLoc, 
    real32_T *outScl,
    real32_T *outMet,
    real32_T *outOri,
    int32_T  *outOct,
    int32_T  *outLay,
    real32_T *outFtrs)
{
    siftKeyPointsToStructFields(
            ptrKeypoints, outLoc, outScl, outMet, outOri, outOct, outLay, false);
    siftFeaturesMatToVector(
            ptrFeatures, outFtrs, false);
}

#endif