//////////////////////////////////////////////////////////////////////////////
// OpenCV SIFT detector wrapper 
//
// Copyright 2021 The MathWorks, Inc.
//  
//////////////////////////////////////////////////////////////////////////////

#ifndef COMPILE_FOR_VISION_BUILTINS
// vision_builtins does not need this source file

#include "detectSIFTCore_api.hpp"
#include "xfeatures2d_sift_mw.hpp"

#include "opencv2/opencv.hpp"
#include "cgCommon.hpp"

////////////////////////////////////////////////////////////////////////////////
// copy SIFT keyPoints to struct
////////////////////////////////////////////////////////////////////////////////
void siftKeyPointToStruct(std::vector<cv::KeyPoint> &keypoints,
                          real32_T * location, real32_T * scale, 
                          real32_T * metric, real32_T * orientation,
                          int32_T * octave, int32_T * layer)
{
    size_t m = keypoints.size();

    for(size_t i = 0; i < m; i++ ) {
        const cv::KeyPoint& kp = keypoints[i];
        
        location[i]    = kp.pt.x+1;              // Convert to 1 based indexing
        location[m+i]  = kp.pt.y+1;
        
        // Scale is equal to the radius of the keypoint region
        // in.size gives the diameter of the keypoint region
        scale[i]       = kp.size*0.5f;
        
        metric[i]      = kp.response; 
        orientation[i] = 0.0f;                   // detector does not compute angle
        octave[i]      = kp.octave & 255;        // octave (int)
        layer[i]       = (kp.octave >> 8) & 255; // layer (int)
    }
}

void siftKeyPointToStructRowMajor(std::vector<cv::KeyPoint> &keypoints,
  real32_T * location, real32_T * scale, 
  real32_T * metric, real32_T * orientation,
  int32_T * octave, int32_T * layer)
{
	size_t m = keypoints.size();

	for (size_t i = 0; i < m; i++) {
		const cv::KeyPoint& kp = keypoints[i];
		/* location = Mx2 */
		*location++     = kp.pt.x + 1;             // Convert to 1 based indexing
		*location++     = kp.pt.y + 1;
        
		// Scale is equal to the radius of the keypoint region
        // in.size gives the diameter of the keypoint region
        scale[i]        = kp.size*0.5f;
        
        metric[i]       = kp.response; 
        orientation[i]  = 0.0f;                    // detector does not compute angle
        octave[i]       = kp.octave & 255;         // octave (int)
        layer[i]        = (kp.octave >> 8) & 255;   // layer (int)
	}
}

////////////////////////////////////////////////////////////////////////////////
// call SIFT::detect, assign keypoints to outKeypoints, and return the number
// detected KeyPoints.
////////////////////////////////////////////////////////////////////////////////
int32_T detectSIFTCompute(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    float contrastThreshold, 
    float edgeThreshold, 
    int numLayersInOctave, 
    float sigma,
	void **outKeypoints, bool isColumnMajor){
    
    // Use OpenCV smart pointer to manage image 
    cv::Ptr<cv::Mat> inImage = new cv::Mat;
    bool isRGB = false; // only grayscale image allowed
    
    if (isColumnMajor){
        cArrayToMat<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }else{
        cArrayToMat_RowMaj<uint8_T>(inImg, nRows, nCols, isRGB, *inImage);
    }
    
    // keypoints
    vector<cv::KeyPoint> *ptrKeypoints = (vector<cv::KeyPoint> *)new vector<cv::KeyPoint>();
    *outKeypoints = ptrKeypoints;
    vector<cv::KeyPoint> &refKeypoints = *ptrKeypoints;

    try
    {
        cv::Ptr<MWSIFT> sift = cv::makePtr<MWSIFT>(0, numLayersInOctave,
                                    contrastThreshold,edgeThreshold,sigma);
        sift->detect(*inImage, refKeypoints);
    }
    catch (...)
    {
        CV_Error(cv::Error::StsNotImplemented, "OpenCV was built without SIFT support");
    }

    return ((int32_T)(refKeypoints.size())); //actual_numel
}

int32_T detectSIFTComputeColumnMajor(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    float contrastThreshold, 
    float edgeThreshold, 
    int numLayersInOctave, 
    float sigma,
	void **outKeypoints)
{
    return detectSIFTCompute(inImg, nRows, nCols, contrastThreshold,
            edgeThreshold, numLayersInOctave, sigma, outKeypoints, true);
}

int32_T detectSIFTComputeRowMajor(
    uint8_T *inImg, 
    int32_T nRows, int32_T nCols, 
    float contrastThreshold, 
    float edgeThreshold, 
    int numLayersInOctave, 
    float sigma,
	void **outKeypoints)
{
    return detectSIFTCompute(inImg, nRows, nCols, contrastThreshold,
            edgeThreshold, numLayersInOctave, sigma, outKeypoints, false);
}

////////////////////////////////////////////////////////////////////////////////
// Copy keypoints to struct and delete keypoint data 
////////////////////////////////////////////////////////////////////////////////
void detectSIFTAssignOutputsColumnMajor(void *ptrKeypoints,
                              real32_T * location, real32_T * scale, 
                              real32_T * metric, real32_T * orientation,
                              int32_T * octave, int32_T * layer)
{
    
    // Populate the outputs
    siftKeyPointToStruct(*((std::vector<cv::KeyPoint> *)ptrKeypoints),
                          location, scale, metric, orientation, octave, layer);
    
    delete((std::vector<cv::KeyPoint> *)ptrKeypoints);
}

void detectSIFTAssignOutputsRowMajor(void *ptrKeypoints,
	real32_T * location, real32_T * scale, 
    real32_T * metric, real32_T * orientation,
    int32_T * octave, int32_T * layer)
{

	// Populate the outputs
	siftKeyPointToStructRowMajor(*((std::vector<cv::KeyPoint> *)ptrKeypoints),
		location, scale, metric, orientation, octave, layer);

	delete((std::vector<cv::KeyPoint> *)ptrKeypoints);
}
#endif