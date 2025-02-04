//////////////////////////////////////////////////////////////////////////////
// OpenCV ArUco Markers Detector Implementation 
//
// Copyright 2023 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////

#include "arucoDictionary.hpp"

namespace vision {
namespace aruco  {

class ArucoReader{
    
        // Inputs
        cv::Ptr<cv::Mat> Image;
        std::vector<std::string> MarkerFamilies;
        cv::aruco::DetectorParameters DetectorParams;
        double MarkerSize;
        cv::Mat CameraMatrix;
        cv::Mat DistCoeffs;

        // Outputs
        std::vector<int> MarkerIds;
        std::vector<std::string> DetectedFamilies;
        std::vector<std::vector<cv::Point2f>> MarkerCorners;
        std::vector<std::vector<std::vector<cv::Point2f>>> RejectedCandidates;
        std::vector<cv::Vec3d> Tvectors;
        std::vector<cv::Matx33d> Rmatrices;

    public:

        // Constructor
        ArucoReader(cv::Ptr<cv::Mat> I, std::vector<std::string> families,
         cv::aruco::DetectorParameters params, double size, cv::Mat camMatrix, 
         cv::Mat distCoeffs) : Image(I), MarkerFamilies(families), 
         DetectorParams(params), MarkerSize(size), CameraMatrix(camMatrix), 
         DistCoeffs(distCoeffs) {}

        // Method to read ArUco markers
        void readMarkerIds();

        // Method to estimate ArUco marker poses
        void estimateMarkerPoses();

        // Getters for the outputs.
        std::vector<int> getMarkerIds();
        std::vector<std::string> getDetectedFamilies();
        std::vector<std::vector<cv::Point2f>> getMarkerCorners();
        std::vector<std::vector<std::vector<cv::Point2f>>> getRejectedCandidates();
        std::vector<cv::Vec3d> getTvectors();
        std::vector<cv::Matx33d> getRmatrices();
};
} // namespace aruco
} // namespace vision