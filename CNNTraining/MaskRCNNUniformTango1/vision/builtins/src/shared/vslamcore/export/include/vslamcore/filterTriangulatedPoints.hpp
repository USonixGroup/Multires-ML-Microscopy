////////////////////////////////////////////////////////////////////////////////
// Utility function to filter out outlier 3-D world points
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef FILTERTRIANGULATEDPOINTS_HPP
#define FILTERTRIANGULATEDPOINTS_HPP

#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"
#include "Configuration.hpp"

namespace vision {
    namespace vslam {

        /**
         * @brief Filter out outlier points using depth and reprojection error.
         *
         * @param[in]      relPose_t           relative translation between the two views, represented as a cv::Vec3d.
         * @param[in]      triangulatedPoints  all reconstructed 3-D world points, represented as an M-by-3 cv::Mat.
         * @param[in]      mask                mask for 3-D world points, with TRUE for valid points and FALSE for invalid points.
         * @param[in]      errorsInCamera1     reprojection error of each world point for view 1, represented as an M-element vector.
         * @param[in]      errorsInCamera2     reprojection error of each world point for view 2, represented as an M-element vector.
         * @param[in]      minCosParallax      double value representing the minimum cosine parallax for 3-D world points
         * @param[in]      maxReprojError      double value representing the maximum reprojection error allowed for inlier points
         * @param[in/out]  matchedPairs        indices of matched feature points which are updated after removing outliers by
         *           checking reprojection error and parallax. It has the same length as xyzPoints after update.
         * @param[out]     xyzPoints           reconstructed 3-D world points after removing outliers, represented as a cv::Mat.
         * @return integer indicating the number of xyzPoints with cosine parallax larger than minCostParallax
         */
        int filterTriangulatedPointsImpl(const cv::Vec3d& relPose_t, const cv::Mat& triangulatedPoints, const cv::Mat_<bool>& mask,
                          const std::vector<double>& errorsInCamera1, const std::vector<double>& errorsInCamera2, const double& minCosParallax, const double& maxReprojError,
                          std::vector<std::pair<int, int>>& matchedPairs, std::vector<cv::Vec3d>& xyzPoints);

        /**
         * @brief Filter out outlier points using depth and reprojection error, and use parallax to determine if map initialization is successful.
         *
         * @param[in]      relPose_t           relative translation between the two views, represented as a cv::Vec3d.
         * @param[in]      triangulatedPoints  all reconstructed 3-D world points, represented as an M-by-3 cv::Mat.
         * @param[in]      numPosDepth         number of 3-D world points with positive depth (+Z), with indices specified in hasPosDepth.
         * @param[in]      hasPosDepth         mask for 3-D world points, with TRUE for positive depth and FALSE for negative depth.
         * @param[in]      errorsInCamera1     reprojection error of each world point for view 1, represented as an M-element vector.
         * @param[in]      errorsInCamera2     reprojection error of each world point for view 2, represented as an M-element vector.
         * @param[in]      config              vision::vslam::ConfigurationMono struct
         * @param[in/out]  matchedPairs        indices of matched feature points which are updated after removing outliers by
         *           checking reprojection error and parallax. It has the same length as xyzPoints after update.
         * @param[out]     xyzPoints           reconstructed 3-D world points after removing outliers, represented as a cv::Mat.
         * @param[out]     isMapInitialized    boolean value indicating if map initialization is successful.
         * @param[out]     medianDepth         double value representing the median depth of the 3-D world points.
         */
        void filterTriangulatedPoints(const cv::Vec3d& relPose_t, const cv::Mat& triangulatedPoints, const int numPosDepth, const cv::Mat_<bool>& hasPosDepth,
                          const std::vector<double>& errorsInCamera1, const std::vector<double>& errorsInCamera2, const ConfigurationMono& config,
                          std::vector<std::pair<int, int>>& matchedPairs, std::vector<cv::Vec3d>& xyzPoints, bool& isMapInitialized, double& medianDepth);
    }// namespace vSLAM
}// namespace vision

#endif // FILTERTRIANGULATEDPOINTS_HPP