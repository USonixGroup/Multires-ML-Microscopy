////////////////////////////////////////////////////////////////////////////////
// Perform 3-D reconstruction from a fundamental matrix
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef RECONSTRUCTFROMFMATRIX_HPP
#define RECONSTRUCTFROMFMATRIX_HPP

#include <vector>
#include <algorithm>

#include "computeReprojectionErrors.hpp"
#include "geometry.hpp"
#include "converter.hpp"
#include "filterTriangulatedPoints.hpp"
#include "Configuration.hpp"

namespace vision {
    namespace vslam {

        /**
        * @brief Perform 3-D reconstruction from a fundamental matrix between two image views.
        *
        * @param[in] tformF a fundamental matrix, represented as a cv::Matx33d.
        * @param[in] imagePoints1 matched feature points in image 1, represented as a vector of cv::Point2f.
        * @param[in] imagePoints2 matched feature points in image 2, represented as a vector of cv::Point2f.
        * @param[in] intrinsics intrinsic parameters of the camera, represented as a cv::Matx33d
        * @param[in/out] matchedPairs indices of matched feature points which are updated after removing outliers by
        *           checking reprojection error and parallax. It has the same length as xyzPoints after update.
        * @param[out] xyzPoints, reconstructed 3-D world points, represented as a cv::Mat.
        * @param[out] isMapInitialized, a boolean value indicating if map initialization is successful.
        * @param[out] medianDepth a double value representing the median depth of the 3-D world points.
        *        If isMapInitialized == FALSE, medianDepth = -DBL_MAX.
        * @param[out] relPose_R relative rotation between the two views, represented as a cv::Matx33d.
        * @param[out] relPose_t relative translation between the two views, represented as a cv::Vec3d.
        * @param[in]  config, a vision::vslam::ConfigurationMono struct.
        */

        void reconstructFromFundamentalMatrix(
            const cv::Matx33d& tformF,
            const std::vector<cv::Point2f>& inlierPoints1,
            const std::vector<cv::Point2f>& inlierPoints2,
            const cv::Matx33d& intrinsics,
            std::vector<std::pair<int, int>>& matchedPairs,
            std::vector<cv::Vec3d>& xyzPoints,
            bool& isMapInitialized,
            double& medianDepth,
            cv::Matx33d& relPose_R,
            cv::Vec3d& relPose_t,
            const ConfigurationMono& config);
    }// namespace vslam
}// namespace vision
#endif //RECONSTRUCTFROMFMATRIX_HPP