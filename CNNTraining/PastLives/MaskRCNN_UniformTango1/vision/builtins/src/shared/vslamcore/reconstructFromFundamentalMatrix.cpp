////////////////////////////////////////////////////////////////////////////////
// Perform 3-D reconstruction from a fundamental matrix
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/reconstructFromFundamentalMatrix.hpp"
#else
    #include "reconstructFromFundamentalMatrix.hpp"
#endif

namespace vision {
    namespace vslam {
        void reconstructFromFundamentalMatrix(
            const cv::Matx33d& tformF,
            const std::vector<cv::Point2f>& imagePoints1,
            const std::vector<cv::Point2f>& imagePoints2,
            const cv::Matx33d& intrinsics,
            std::vector<std::pair<int, int>>& matchedPairs,
            std::vector<cv::Vec3d>& xyzPoints,
            bool& isMapInitialized,
            double& medianDepth,
            cv::Matx33d& relPose_R,
            cv::Vec3d& relPose_t,
            const ConfigurationMono& config) {
 
            isMapInitialized = false;
            medianDepth = -DBL_MAX;

            // Transform fundamental matrix to essential matrix
            cv::Matx33d tformE = intrinsics.t() * tformF * intrinsics;

            // Compute R, t, and the triangulated points
            cv::Matx33d extrinsics_R;
            cv::Vec3d extrinsics_t;
            cv::Mat triangulatedPoints;
            cv::Mat_<bool> mask;
            constexpr double distanceThresh{ 57.2987}; // With 1 degree parallax, the distance is 1/sind(1) 
            const int numValidPoints = cv::recoverPose(tformE, imagePoints1, imagePoints2, intrinsics, extrinsics_R, extrinsics_t, distanceThresh, mask, triangulatedPoints);

            if (numValidPoints < config.baseParams.minNumWorldPoints)
                return;

            // Convert points from 4-D to 3-D
            hom2cart<double>(triangulatedPoints);

            // Compute reprojection errors
            std::vector<double> errorsInCamera1 = computeReprojectionErrors(
                triangulatedPoints, imagePoints1, cv::Matx33d::eye(),
                cv::Vec3d(0, 0, 0), intrinsics);

            std::vector<double> errorsInCamera2 = computeReprojectionErrors(
                triangulatedPoints, imagePoints2, extrinsics_R, extrinsics_t,
                intrinsics);

            // Convert from extrinsics to camera pose
            extr2pose(extrinsics_R, extrinsics_t, relPose_R, relPose_t);

            // Filter 3-D points by reprojection error, parallax, and depth
            filterTriangulatedPoints(relPose_t, triangulatedPoints, numValidPoints, mask, errorsInCamera1, errorsInCamera2, config,
                         matchedPairs, xyzPoints, isMapInitialized, medianDepth);
        }
    } // namespace vslam
} // namespace vision