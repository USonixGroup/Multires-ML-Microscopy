//////////////////////////////////////////////////////////////////////////////
// OpenCV KAZE detector wrapper for code generation
//
// Copyright 2017 The MathWorks, Inc.
//  
//////////////////////////////////////////////////////////////////////////////

#ifndef COMPILE_FOR_VISION_BUILTINS
// vision_builtins does not need this source file

#include "detectKAZECore_api.hpp"

#include "opencv2/opencv.hpp"
#include "cgCommon.hpp"

using namespace std;

//////////////////////////////////////////////////////////////////////////////
// Invoke OpenCV cvDetectKAZE
//////////////////////////////////////////////////////////////////////////////
int32_T detectKAZECompute(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, bool extended, bool upright,
    float threshold, int numOctaves, int numScaleLevels, int diffusivity,
    void **outKeypoints, bool isCM){
    
    // Use OpenCV smart pointer to manage image 
    cv::Ptr<cv::Mat> inImage = new cv::Mat;
    bool isRGB = false; // only grayscale image allowed
    
    if (isCM){
        cArrayToMat<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }else{
        cArrayToMat_RowMaj<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }
    KAZE::DiffusivityType Diffusivity = static_cast<KAZE::DiffusivityType>(diffusivity);

    // keypoints
    vector<cv::KeyPoint> *ptrKeypoints = (vector<cv::KeyPoint> *)new vector<cv::KeyPoint>();
    *outKeypoints = ptrKeypoints;
    vector<cv::KeyPoint> &refKeypoints = *ptrKeypoints;

    try
    {
        cv::Ptr<cv::KAZE> kaze = cv::KAZE::create(
                extended, upright, threshold, 
                numOctaves, numScaleLevels, Diffusivity);
        kaze->detect(*inImage, refKeypoints, cv::Mat());
    }
    catch (...)
    {
        CV_Error(cv::Error::StsNotImplemented, "OpenCV was built without KAZE support");
    }

    return ((int32_T)(refKeypoints.size())); //actual_numel
}

int32_T detectKAZEComputeCM(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    bool extended, bool upright,
    float threshold, 
    int numOctaves,
    int numScaleLevels, 
    int diffusivity,
	void **outKeypoints)
{
    return detectKAZECompute(inImg, nRows, nCols, extended, upright,
            threshold, numOctaves, numScaleLevels, diffusivity,
            outKeypoints, true);
}

int32_T detectKAZEComputeRM(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    bool extended, bool upright,
    float threshold, 
    int numOctaves,
    int numScaleLevels, 
    int diffusivity,
	void **outKeypoints)
{
    return detectKAZECompute(inImg, nRows, nCols, extended, upright,
            threshold, numOctaves, numScaleLevels, diffusivity,
            outKeypoints, false);
}

//////////////////////////////////////////////////////////////////////////////
// Assign KeyPoint to Fields
//////////////////////////////////////////////////////////////////////////////
void kazeKeyPointToFields(
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
            outLoc[i]   = kp.pt.x + 1;
            outLoc[m+i] = kp.pt.y + 1;
        }else{      // row major
            *outLoc++   = kp.pt.x + 1;
            *outLoc++   = kp.pt.y + 1;
        }
        
        outOri[i] = kp.angle *(float)(CV_PI/180.0); // angle to radian
        outMet[i] = kp.response;  // metric (float)
        outScl[i] = kp.size;
        outMis[i] = (uint8_T)kp.class_id;  // e.g. Layer ID for KAZE
    }
    
    delete((vector<cv::KeyPoint> *)ptrKeypoints);
}

void detectKAZEAssignOutputCM(
    void *ptrKeypoints,
    real32_T *outLoc, 
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl,
    uint8_T  *outMis)
{
    kazeKeyPointToFields(ptrKeypoints, 
            outLoc, outOri, outMet, outScl, outMis, true);
}

void detectKAZEAssignOutputRM(
    void *ptrKeypoints,
    real32_T *outLoc, 
    real32_T *outOri,
    real32_T *outMet,
    real32_T *outScl,
    uint8_T  *outMis)
{
    kazeKeyPointToFields(ptrKeypoints, 
            outLoc, outOri, outMet, outScl, outMis, false);	
}

#endif