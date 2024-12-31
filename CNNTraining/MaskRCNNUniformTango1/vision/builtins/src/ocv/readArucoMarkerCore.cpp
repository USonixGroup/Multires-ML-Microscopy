//////////////////////////////////////////////////////////////////////////////
// OpenCV ArUco Markers Detector Codegen Wrapper
//
// Copyright 2024 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////

#ifndef COMPILE_FOR_VISION_BUILTINS
// vision_builtins does not need this source file
#include "readArucoMarkerCore_api.hpp"
#include "readArucoImpl.hpp"
#include "cgCommon.hpp"

//////////////////////////////////////////////////////////////////////////////
// Utility to parse a list of ArUco marker families.
//////////////////////////////////////////////////////////////////////////////
std::vector<std::string> parseMarkerFamilies(const std::string mFamilies,
                                             const int numMarkers,
                                             const int* markerLengths){

    std::vector<std::string> markerFamilies;
    // Use one family
    if (numMarkers == 1) {
        int len = markerLengths[0];
        std::string tmpFormat("");
        for (int j = 0; j < len; j++)
            tmpFormat = tmpFormat + mFamilies[j];
        markerFamilies.push_back(tmpFormat);
    }else {
        // Use a set of families
        int start = 0;        
        for (int i = 0; i < numMarkers; i++) {
            // extract length of each individual family
            int len = markerLengths[i];
            std::string tmpFormat("");
            for (int j = 0; j < len; j++)
                // add each family to string
                tmpFormat = tmpFormat + mFamilies[start+j];

            markerFamilies.push_back(tmpFormat);
            start = start + len;           
        }
    }
    return markerFamilies;
}

//////////////////////////////////////////////////////////////////////////////
// Utility to parse readAruco input parameters.
//////////////////////////////////////////////////////////////////////////////
cv::aruco::DetectorParameters parseArucoDetectorParams(const ArucoDetectorParams mParams) {

    cv::aruco::DetectorParameters params = cv::aruco::DetectorParameters();

    // Adaptive thresholding parameters
    params.adaptiveThreshWinSizeMin    = mParams.adaptiveThreshWinSizeMin;
    params.adaptiveThreshWinSizeMax    = mParams.adaptiveThreshWinSizeMax;
    params.adaptiveThreshWinSizeStep   = mParams.adaptiveThreshWinSizeStep;
    params.adaptiveThreshConstant      = mParams.adaptiveThreshConstant;

    // Contour filtering parameters
    params.minMarkerPerimeterRate      = mParams.minMarkerPerimeterRate;
    params.maxMarkerPerimeterRate      = mParams.maxMarkerPerimeterRate;
    params.polygonalApproxAccuracyRate = mParams.polygonalApproxAccuracyRate;
    params.minCornerDistanceRate       = mParams.minCornerDistanceRate;
    params.minDistanceToBorder         = mParams.minDistanceToBorder;
    params.minMarkerDistanceRate       = mParams.minMarkerDistanceRate;

    // Perspective distortion removal parameters
    params.perspectiveRemovePixelPerCell         = mParams.perspectiveRemovePixelPerCell;
    params.perspectiveRemoveIgnoredMarginPerCell = mParams.perspectiveRemoveIgnoredMarginPerCell;

    // Bit extraction parameters
    params.markerBorderBits = mParams.markerBorderBits;
    params.minOtsuStdDev    = mParams.minOtsuStdDev;

    // Error correction parameters
    params.maxErroneousBitsInBorderRate = mParams.maxErroneousBitsInBorderRate;
    params.errorCorrectionRate          = mParams.errorCorrectionRate;

    // Global parameter: Used in all steps
    params.detectInvertedMarker = mParams.detectInvertedMarker;

    // Aruco 3 functionality parameters
    params.useAruco3Detection              = mParams.useAruco3Detection;
    params.minSideLengthCanonicalImg       = mParams.minSideLengthCanonicalImg;
    params.minMarkerLengthRatioOriginalImg = mParams.minMarkerLengthRatioOriginalImg;

    // Corner refinement parameters
    params.cornerRefinementMethod        = static_cast<const cv::aruco::CornerRefineMethod>(mParams.cornerRefinementMethod);
    params.cornerRefinementWinSize       = mParams.cornerRefinementWinSize;
    params.cornerRefinementMaxIterations = mParams.cornerRefinementMaxIterations;
    params.cornerRefinementMinAccuracy   = mParams.cornerRefinementMinAccuracy;

    return params;
}

///////////////////////////////////////////////////////////////////
// Function initializes marker locs and rejections
///////////////////////////////////////////////////////////////////
void initializeLocs(double* &locations,  std::vector<std::vector<cv::Point2f>> &locs,
                    const size_t detectionSize, const size_t offset) {

    for (size_t i = 0; i < detectionSize; ++i) {

        cv::Point2f topLeft     = locs[i][0];
        cv::Point2f topRight    = locs[i][1];
        cv::Point2f bottomRight = locs[i][2];
        cv::Point2f bottomLeft  = locs[i][3];

        // Translate pixel locations to 1-based indexing.
        locations[8*i+offset]   = static_cast<double>(topLeft.x+1);
        locations[8*i+4+offset] = static_cast<double>(topLeft.y+1);

        locations[8*i+1+offset] = static_cast<double>(topRight.x+1);
        locations[8*i+5+offset] = static_cast<double>(topRight.y+1);

        locations[8*i+2+offset] = static_cast<double>(bottomRight.x+1);
        locations[8*i+6+offset] = static_cast<double>(bottomRight.y+1);

        locations[8*i+3+offset] = static_cast<double>(bottomLeft.x+1);
        locations[8*i+7+offset] = static_cast<double>(bottomLeft.y+1);
    }

}

//////////////////////////////////////////////////////////////////////////////
// Implementation of readAruco.
//////////////////////////////////////////////////////////////////////////////
uint32_T readArucoImplCore(void* mImgData,
                          const bool isRGB,
                          const int nRows, const int nCols,
                          void* mFamiliesData,
                          const int numMarkers,
                          const int* markerLengths,
                          const ArucoDetectorParams mParams,
                          const bool doEstimatePose,
                          const double markerSize,
                          void* mCamMatrixPtr,
                          void* mDistCoeffsPtr, void** detectionsObj,
                          int* markerfamlen, int* rejectionLen,
                          int* poseLen)
{
    uint8_T* imgData = static_cast<uint8_T*>(mImgData);
    const std::string mFamilies = static_cast<const char*>(mFamiliesData);
    double* camMatrixPtr = static_cast<double*>(mCamMatrixPtr);
    double* distCoeffsPtr = static_cast<double*>(mDistCoeffsPtr);    

    using namespace cv;
    cv::Mat * image;
    image = new cv::Mat;
    cArrayToMat<uint8_T>(imgData, nRows, nCols, isRGB, *image);
    cv::Ptr<cv::Mat> out = cv::Ptr<cv::Mat>(image);

    auto families = parseMarkerFamilies(mFamilies, numMarkers, markerLengths);

    auto params = parseArucoDetectorParams(mParams);

    cv::Mat camMatrix, distCoeffs;
    if(doEstimatePose){
        camMatrix = cv::Mat(3, 3, CV_64F, camMatrixPtr);
        distCoeffs = cv::Mat(1, 5, CV_64F, distCoeffsPtr);
    }

    vision::aruco::ArucoReader **readerObj = new vision::aruco::ArucoReader* [1];

    for (int i = 0; i < 1; i++) {
        readerObj[i] = new vision::aruco::ArucoReader(out, families, params, markerSize,
                                                 camMatrix, distCoeffs);
    }   
    *detectionsObj = readerObj;

    readerObj[0]->readMarkerIds();
    const std::vector<int> markerIds =  readerObj[0]->getMarkerIds();    
     const uint32_T idsSize = static_cast<uint32_T>(markerIds.size());
   
    std::vector<std::string> detectedFamilies = readerObj[0]->getDetectedFamilies();
    for (uint32_T i = 0; i < idsSize; ++i)
        *markerfamlen = *markerfamlen + static_cast<int>(detectedFamilies[i].size());

    std::vector<std::vector<std::vector<cv::Point2f>>> locsRejections = readerObj[0]->getRejectedCandidates();
    int rejectionsize = static_cast<int>(locsRejections.size());

    for(int i = 0; i< rejectionsize ; i++)
        *rejectionLen = *rejectionLen +  static_cast<int>(locsRejections[i].size());

    if(doEstimatePose) {
        readerObj[0]->estimateMarkerPoses();
        std::vector<cv::Matx33d> inMatrices = readerObj[0]->getRmatrices();
        *poseLen =  static_cast<int>(inMatrices.size());
    }

    return idsSize;
}


////////////////////////////////////////////////////////////////////////////////
// Function to initialize all the required outputs - message, locations and
// detected marker
////////////////////////////////////////////////////////////////////////////////

void assignOutputs(void* mIds, void* mLocs, void* mFamLength,
                   void* mFamilyNames, void* mRejections, void* mRotMatrices,
                   void* mTransVector, void* detectionsObj,
                   const bool doEstimatePose) {

    char* familyNames = static_cast<char*>(mFamilyNames);
    double* locations = static_cast<double*>(mLocs);
    double* rejections = static_cast<double*>(mRejections);
    double* ids = static_cast<double*>(mIds);

    double* rotMatrices = static_cast<double*>(mRotMatrices);
    double* transVector = static_cast<double*>(mTransVector);
    int32_T* lengths = static_cast<int32_T*>(mFamLength);

    vision::aruco::ArucoReader **readerObj = (vision::aruco::ArucoReader **)detectionsObj;
   
    const std::vector<int> markerIds = readerObj[0]->getMarkerIds();    
    const size_t idsSize = markerIds.size();
    for (size_t i = 0; i < idsSize; ++i)
        ids[i] = static_cast<double>(markerIds[i]);

    std::vector<std::vector<cv::Point2f>> locs = readerObj[0]->getMarkerCorners();
    initializeLocs(locations, locs, idsSize, 0);   

    std::vector<std::string> families = readerObj[0]->getDetectedFamilies();
    int32_T sumLen = 0;
    for (size_t i = 0; i < idsSize; ++i){
        std::string fName = families.at(i);
        int32_T len = static_cast<int32_T>(fName.size());
        lengths[i] = len;
        for (int32_T j = 0; j < len; ++j)
            familyNames[sumLen+j] = fName[j];
        sumLen = sumLen + len;
    }

    std::vector<std::vector<std::vector<cv::Point2f>>> locsRejections = readerObj[0]->getRejectedCandidates();
    int rejectionsize = static_cast<int>(locsRejections.size());
   
    size_t offset = 0;
    for(int i = 0; i < rejectionsize; i++){        
        std::vector<std::vector<cv::Point2f>> temp = locsRejections.at(i);
        size_t len = temp.size();     
        initializeLocs(rejections, temp, len, offset);
        offset = offset + 8*len;
    }

    if(doEstimatePose) {

        std::vector<cv::Matx33d> inMatrices = readerObj[0]->getRmatrices();
        int32_T poselen = static_cast<int32_T>(inMatrices.size());

        for (int32_T i = 0; i < poselen; ++i) {

            rotMatrices[9*i]   = inMatrices.at(i)(0,0);
            rotMatrices[9*i+1] = inMatrices.at(i)(1,0);
            rotMatrices[9*i+2] = inMatrices.at(i)(2,0);

            rotMatrices[9*i+3] = inMatrices.at(i)(0,1);
            rotMatrices[9*i+4] = inMatrices.at(i)(1,1);
            rotMatrices[9*i+5] = inMatrices.at(i)(2,1);

            rotMatrices[9*i+6] = inMatrices.at(i)(0,2);
            rotMatrices[9*i+7] = inMatrices.at(i)(1,2);
            rotMatrices[9*i+8] = inMatrices.at(i)(2,2);
        }

        std::vector<cv::Vec3d> inVectors = readerObj[0]->getTvectors();
        for (int32_T i = 0; i < poselen; ++i) {

            transVector[3*i]   = inVectors[i][0];
            transVector[3*i+1] = inVectors[i][1];
            transVector[3*i+2] = inVectors[i][2];            
        }

    }

}
////////////////////////////////////////////////////////////////////////////////
// Function to delete the object pointer memory
////////////////////////////////////////////////////////////////////////////////
void deleteResultPtr(void *ptrObj)
{
    vision::aruco::ArucoReader **readerObj = (vision::aruco::ArucoReader **)ptrObj;
    if (readerObj)
    {
        if (readerObj[0])
            delete readerObj[0];

        delete[] readerObj;
    }
}

#endif