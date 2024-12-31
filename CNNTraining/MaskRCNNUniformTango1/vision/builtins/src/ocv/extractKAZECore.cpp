//////////////////////////////////////////////////////////////////////////////
// OpenCV KAZE extractor wrapper
//
// Copyright 2017 The MathWorks, Inc.
//
//////////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS
#include "opencv2/opencv.hpp"

#include "extractKAZECore_api.hpp"
#include "cgCommon.hpp"

using namespace std;

//////////////////////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////////////////////

void fieldsToKAZEKeyPoints(
    real32_T *inLoc, 
    real32_T *inOri,
    real32_T *inMet,
    real32_T *inScl,
    uint8_T  *inMis,
    bool isCM, int numKPts,
    vector<cv::KeyPoint> & keypoints){
    keypoints.reserve(numKPts);
    for (int i = 0; i < numKPts; i ++){
        float x, y;
        // convert to C/C++ 0-indexing
        if (isCM){  // column major
            x = float(inLoc[i]) - 1.0f;
            y = float(inLoc[numKPts + i]) - 1.0f;
        }
        else{       // row major
            x = float(inLoc[i*2]) - 1.0f;
            y = float(inLoc[i*2 + 1]) - 1.0f;
        }
        float orientation = inOri[i]; // let extractor assign actual values
        float scale = inScl[i];
        float metric = inMet[i];
        int   misc = inMis[i];
        cv::KeyPoint kpt(x, y, scale, orientation, metric, 0, misc);
        keypoints.push_back(kpt);
    }
}

void kazeKeyPointsToStructFields(
    void *ptrKeypoints,
    real32_T *outLoc, 
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl,
    uint8_T  *outMis,
    bool isCM)
{
    vector<cv::KeyPoint> &keypoints = ((vector<cv::KeyPoint> *)ptrKeypoints)[0];
    size_t m = keypoints.size();

    for(size_t i = 0; i < m; i++ ) {
        
        cv::KeyPoint& kp = keypoints[i];
        
        // convert C 0-index to MATLAB 1-index
        if (isCM){  // column major
            outLoc[i]   = kp.pt.x + 1.0f;
            outLoc[m+i] = kp.pt.y + 1.0f;
        }else{      // row major
            *outLoc++   = kp.pt.x + 1.0f;
            *outLoc++   = kp.pt.y + 1.0f;
        }
        
        outOri[i] = kp.angle *(float)(CV_PI/180.0); // angle to radian, should be >= 0 according to OpenCV implementation.
        outMet[i] = kp.response;  // metric (float)
        outScl[i] = kp.size;
        outMis[i] = (uint8_T)kp.class_id;  // e.g. Layer ID for KAZE
    }
    
    delete((vector<cv::KeyPoint> *)ptrKeypoints);
}

void kazeFeaturesMatToVector(
        void *ptrDescriptors,
        real32_T *outFeatures,
        bool isCM)
{
    cv::Mat descriptors = *((cv::Mat *)ptrDescriptors);
    if (isCM){
        cArrayFromMat<real32_T>(outFeatures, descriptors);
    }
    else
    {
        cArrayFromMat_RowMaj<real32_T>(outFeatures, descriptors);
    }
    delete((cv::Mat *)ptrDescriptors);
}


//////////////////////////////////////////////////////////////////////////////
// Invoke OpenCV KAZE extractor
//////////////////////////////////////////////////////////////////////////////

int32_T extractKAZECompute(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inOri,
    real32_T *inMet,
    real32_T *inScl,
    uint8_T  *inMis,
    int numKPts,
    bool extended, bool upright, float threshold,
    int numOctaves, int numScaleLevels, int diffusivity,
    void **outKeypoints, void **outFeatures, bool isCM)
{
    
    // Use OpenCV smart pointer to manage image 
    cv::Ptr<cv::Mat> inImage = new cv::Mat;
    bool isRGB = false; // only grayscale image allowed
    
    if (isCM){
        cArrayToMat<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }else{
        cArrayToMat_RowMaj<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }
    KAZE::DiffusivityType Diffusivity = static_cast<KAZE::DiffusivityType>(diffusivity);
    // Create KeyPoint array from the given fields
    vector<cv::KeyPoint> *ptrKeypoints = (vector<cv::KeyPoint> *)new vector<cv::KeyPoint>();
    *outKeypoints = ptrKeypoints;
    vector<cv::KeyPoint> &refKeypoints = *ptrKeypoints;
    fieldsToKAZEKeyPoints(
            inLoc, inOri, inMet, inScl, inMis, isCM, numKPts, refKeypoints);
    // Define the feature pointer
    cv::Mat * ptrFeatures = new cv::Mat;
    *outFeatures = ptrFeatures;
    cv::Mat & refFeatures = *ptrFeatures;
    
    // invoke the OpenCV KAZE
    try
    {
        cv::Ptr<cv::KAZE> kaze = cv::KAZE::create(
                extended, upright, threshold, 
                numOctaves, numScaleLevels, Diffusivity);
        kaze->compute(*inImage, refKeypoints, refFeatures);
    }
    catch (...)
    {
        CV_Error(cv::Error::StsNotImplemented, "OpenCV was built without KAZE support");
    }

    return ((int32_T)(refKeypoints.size())); //actual_numel
}


int32_T extractKAZEComputeCM(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inOri,
    real32_T *inMet,
    real32_T *inScl,
    uint8_T  *inMis,
    int numKPts,
    bool extended, bool upright, float threshold,
    int numOctaves, int numScaleLevels, int diffusivity,
    void **outKeypoints, void **outFeatures){
    return extractKAZECompute(inImg, nRows, nCols,
            inLoc, inOri, inMet, inScl, inMis, numKPts,
            extended, upright, threshold,
            numOctaves, numScaleLevels, diffusivity,
            outKeypoints, outFeatures, true);
}

int32_T extractKAZEComputeRM(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    real32_T *inLoc, 
    real32_T *inOri,
    real32_T *inMet,
    real32_T *inScl,
    uint8_T  *inMis,
    int numKPts,
    bool extended, bool upright, float threshold,
    int numOctaves, int numScaleLevels, int diffusivity,
    void **outKeypoints, void **outFeatures){
    return extractKAZECompute(inImg, nRows, nCols,
            inLoc, inOri, inMet, inScl, inMis, numKPts,
            extended, upright, threshold,
            numOctaves, numScaleLevels, diffusivity,
            outKeypoints, outFeatures, false);
}

//////////////////////////////////////////////////////////////////////////////
// Assign KeyPoint to Fields and Descriptor Mat to Array
//////////////////////////////////////////////////////////////////////////////

void extractKAZEAssignOutputCM(
    void * ptrKeypoints, 
    void * ptrFeatures,
    real32_T *outLoc, 
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl,
    uint8_T  *outMis,
    real32_T *outFtrs)
{
    kazeKeyPointsToStructFields(
            ptrKeypoints, outLoc, outOri, outMet, outScl, outMis, true);
    kazeFeaturesMatToVector(
            ptrFeatures, outFtrs, true);
}

void extractKAZEAssignOutputRM(
    void * ptrKeypoints, 
    void * ptrFeatures,
    real32_T *outLoc, 
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl,
    uint8_T  *outMis,
    real32_T *outFtrs)
{
    kazeKeyPointsToStructFields(
            ptrKeypoints, outLoc, outOri, outMet, outScl, outMis, false);
    kazeFeaturesMatToVector(
            ptrFeatures, outFtrs, false);
}

#endif