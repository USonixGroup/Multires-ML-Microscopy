////////////////////////////////////////////////////////////////////////////////
// Compute reprojection errors for 3-D world points.
// 
// Copyright 2022-2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef COMPUTEREPROJECTIONERRORS_HPP
#define COMPUTEREPROJECTIONERRORS_HPP

#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"

namespace vision {
    namespace vslam {
        /**
        * @brief Compute reprojection errors for 3-D world points.
        *
        * @param[in] points3d 3-D world points, specified as an M-by-3 cv::Mat of double type.
        * @param[in] imagePoints 2-D image points, specified as an M-element vector of cv::Point2f.
        * @param[in] extrinsics_R rotation matrix from world coordinate to camera coordinate, specified as a 3-by-3 cv::Mat of double type.
        * @param[in] extrinsics_t translation vector from world coordinate to camera coordinate, specified as a cv::Vec3d.
        * @param[in] intrinsics intrinsic parameters of the camera, specified as a 3-by-3 cv::Mat of double type.
        * @return An M-element vector containing the reprojection error of each world point.
        */

        std::vector<double> computeReprojectionErrors(
            const cv::Mat& points3d,
            const std::vector<cv::Point2f>& imagePoints,
            const cv::Matx33d& extrinsics_R,
            const cv::Vec3d& extrinsics_t,
            const cv::Matx33d& intrinsics);

        /**
        * @brief Compute reprojection errors for 3-D world points.
        *
        * @param[in] point3d 3-D world point, specified as a cv::Vec3d.
        * @param[in] imagePoint 2-D image point, specified as a cv::Point2f.
        * @param[in] extrinsics_R rotation matrix from world coordinate to camera coordinate, specified as a 3-by-3 cv::Mat of double type.
        * @param[in] extrinsics_t translation vector from world coordinate to camera coordinate, specified as a cv::Vec3d.
        * @param[in] intrinsics intrinsic parameters of the camera, specified as a 3-by-3 cv::Mat of double type.
        * @return An double scalar containing the reprojection error of the world point.
        */
        double computeReprojectionErrors(
            const cv::Vec3d& point3d,
            const cv::Point2f& imagePoint,
            const cv::Matx33d& extrinsics_R,
            const cv::Vec3d& extrinsics_t,
            const cv::Matx33d& intrinsics);
    }// namespace vslam
}// namespace vision

#endif // COMPUTEREPROJECTIONERRORS_HPP