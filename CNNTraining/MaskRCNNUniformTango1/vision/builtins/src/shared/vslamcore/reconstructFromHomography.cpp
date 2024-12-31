////////////////////////////////////////////////////////////////////////////////
// Perform 3-D reconstruction from a homography matrix
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
#include "vslamcore/reconstructFromHomography.hpp"
#else
#include "reconstructFromHomography.hpp"
#endif

namespace vision {
    namespace vslam {
        void reconstructFromHomography(
            const cv::Matx33d& tformH,
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

            // Decompose the homography matrix
            std::vector<cv::Mat> Rs, ts;
            int numSol = cv::decomposeHomographyMat(tformH, intrinsics, Rs, ts, cv::noArray());

            // Determine which of the 4 possible solutions is physically realizable.
            // A physically realizable solution is the one which puts reconstructed 3D
            // points in front of both cameras. There could be two solutions if the R
            // and t are extracted from homography matrix
            cv::Mat_<double> pointsInCamera2 = cv::Mat_<double>::zeros(3, imagePoints1.size());
            cv::Mat pointsInCamera1; // triangulatePoints requires an output that can have imagePoints1.type()
            cv::Matx34d camProjMat1, camProjMat2; // camera projection matrix

            cv::hconcat(intrinsics, cv::Vec3d::zeros(), camProjMat1);

            cv::Mat_<bool> isPositiveDepth, isValidPoints;
            cv::Mat validPoints3d;
            int maxNumPositiveDepth{ 0 }, max2NumPositiveDepth{ 0 };// Track the number of 3-D points in front of both cameras
            int maxNumPositiveDepthIdx{ 0 };
            cv::Vec3d t_normalized;

            for (int i = 0; i != numSol; ++i) {
                // Get projection matrix of the second camera
                cv::normalize(ts[i], t_normalized);

                cv::hconcat(intrinsics * Rs[i], intrinsics * t_normalized, camProjMat2); // K * [R, t]

                triangulatePoints(camProjMat1, camProjMat2, imagePoints1, imagePoints2, pointsInCamera1);

                // Convert points from 4-D float to 3-D double
                hom2cart<double>(pointsInCamera1);

                for (int j = 0; j < pointsInCamera1.cols; ++j)
                    pointsInCamera2.col(j) = Rs[i].t() * (pointsInCamera1.col(j) - t_normalized);

                isPositiveDepth = (pointsInCamera1.row(2) > 0) & (pointsInCamera2.row(2) > 0); // Positive Z

                int currNumPositives = cv::countNonZero(isPositiveDepth);

                if (currNumPositives > maxNumPositiveDepth) {
                    maxNumPositiveDepth = currNumPositives;
                    maxNumPositiveDepthIdx = i;
                    cv::swap(isValidPoints, isPositiveDepth);
                    cv::swap(validPoints3d, pointsInCamera1);
                }
                else if (currNumPositives > max2NumPositiveDepth) {
                    max2NumPositiveDepth = currNumPositives;
                }
            }

            if (maxNumPositiveDepth < config.baseParams.minNumWorldPoints ||
                max2NumPositiveDepth > maxNumPositiveDepth*config.monoParams.maxRatioHomography) {
                return;
            }

            // Get the best rotation and translation
            cv::Matx33d extrinsics_R = Rs[maxNumPositiveDepthIdx];
            cv::Vec3d extrinsics_t;
            cv::normalize(ts[maxNumPositiveDepthIdx], extrinsics_t);

            // Compute reprojection errors
            std::vector<double> errorsInCamera1 = computeReprojectionErrors(
                validPoints3d, imagePoints1, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0),
                intrinsics);

            std::vector<double> errorsInCamera2 = computeReprojectionErrors(
                validPoints3d, imagePoints2, extrinsics_R, extrinsics_t,
                intrinsics);

            // Convert from extrinsics to camera pose
            extr2pose(extrinsics_R, extrinsics_t, relPose_R, relPose_t);

            // Filter 3-D points by reprojection error, parallax, and depth
            filterTriangulatedPoints(relPose_t, validPoints3d, maxNumPositiveDepth, isValidPoints, errorsInCamera1, errorsInCamera2, config,
                matchedPairs, xyzPoints, isMapInitialized, medianDepth);
        }
    } // namespace vslam
} // namespace vision