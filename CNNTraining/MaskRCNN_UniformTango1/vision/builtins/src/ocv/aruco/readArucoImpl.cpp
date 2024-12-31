//////////////////////////////////////////////////////////////////////////////
// OpenCV ArUco Markers Detector Implementation 
//
// Copyright 2023 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////

#include "aruco/readArucoImpl.hpp"

namespace vision {
namespace aruco  {

//////////////////////////////////////////////////////////////////////////////
// Method to read ArUco markers
//////////////////////////////////////////////////////////////////////////////
void ArucoReader::readMarkerIds(){
    
    size_t numFamilies = this->MarkerFamilies.size();
    this->RejectedCandidates.reserve(numFamilies);
    for(size_t i = 0; i < numFamilies; ++i){

        // Define the ArUco detector.
        auto dictionary = getPredefinedDictionary(this->MarkerFamilies.at(i));
        cv::aruco::ArucoDetector detector(dictionary, this->DetectorParams);
        
        // Detect ArUco markers in the input image.
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        detector.detectMarkers(*this->Image, markerCorners, markerIds, rejectedCandidates);

        // Store the results.
        std::copy(markerIds.begin(), markerIds.end(), std::back_inserter(this->MarkerIds)); 
        std::copy(markerCorners.begin(), markerCorners.end(), std::back_inserter(this->MarkerCorners));
        this->RejectedCandidates.push_back(rejectedCandidates);

        size_t numMarkers = markerIds.size();
        for(size_t j = 0; j < numMarkers; j++) {
            this->DetectedFamilies.push_back(this->MarkerFamilies.at(i));
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
// Method to estimate ArUco marker poses
//////////////////////////////////////////////////////////////////////////////
void ArucoReader::estimateMarkerPoses(){
    
    // Set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    float a = static_cast<float>(this->MarkerSize);
    objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-a/2,  a/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f( a/2,  a/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f( a/2, -a/2, 0);
    objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-a/2, -a/2, 0);
    
    // Estimate pose of each marker.
    size_t numMarkers = this->MarkerCorners.size();
    this->Tvectors.reserve(numMarkers);
    this->Rmatrices.reserve(numMarkers);
        
    // Calculate pose for each marker
    for (size_t i = 0; i < numMarkers; ++i) {
        
        // Estimate pose
        cv::Vec3d rvec, tvec;
        solvePnP(objPoints, this->MarkerCorners.at(i), this->CameraMatrix, 
                 this->DistCoeffs, rvec, tvec);

        // Convert rotation vector to rotation matrix.
        cv::Matx33d rmat;
        cv::Rodrigues(rvec, rmat);
        
        this->Tvectors.push_back(tvec);
        this->Rmatrices.push_back(rmat);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Getters for outputs.
//////////////////////////////////////////////////////////////////////////////
std::vector<int> ArucoReader::getMarkerIds(){
    return this->MarkerIds;
}

//////////////////////////////////////////////////////////////////////////////
std::vector<std::string> ArucoReader::getDetectedFamilies(){
    return this->DetectedFamilies;
}

//////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<cv::Point2f>> ArucoReader::getMarkerCorners(){
    return this->MarkerCorners;
}

//////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<std::vector<cv::Point2f>>> ArucoReader::getRejectedCandidates(){
    return this->RejectedCandidates;
}

//////////////////////////////////////////////////////////////////////////////
std::vector<cv::Vec3d> ArucoReader::getTvectors(){
    return this->Tvectors;
}

//////////////////////////////////////////////////////////////////////////////
std::vector<cv::Matx33d> ArucoReader::getRmatrices(){
    return this->Rmatrices;
}

} // namespace aruco
} // namespace vision