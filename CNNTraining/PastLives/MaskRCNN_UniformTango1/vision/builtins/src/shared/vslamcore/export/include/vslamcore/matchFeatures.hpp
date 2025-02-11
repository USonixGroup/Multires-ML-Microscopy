////////////////////////////////////////////////////////////////////////////////
// Find matched features between two sets of ORB feature descriptors
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef MATCHFEATURES_HPP
#define MATCHFEATURES_HPP

#include <algorithm>
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"

namespace vision {
    namespace vslam {
       /**
        * @brief Find matched features
        *
        * @param[in] features1 ORB feature set 1, specified as an M-by-32 cv::Mat.
        * @param[in] features2 ORB feature set 2, specified as an N-by-32 cv::Mat.
        * @param[in] points1 Key points corresponding to features1, specified as an M-element vector of cv::KeyPoint.
        * @param[in] points2 Key points corresponding to features2, specified as an N-element vector of cv::KeyPoint.
        * @param[in] matchThreshold Matching threshold, specified as an integer. The maximum value is 256.
        * @param[in] maxRatio Ratio threshold, specified as a float value. 
        * @param[in] maxOctaveDiff Octave level threshold, specified as an integer. 
        * @return Indices of matched features, returned as a vector of pairs with features1 indices monotonically increasing.
        */
        std::vector<std::pair<int, int>> matchFeatures(
            const cv::Mat& features1,
            const cv::Mat& features2,
            const std::vector<cv::KeyPoint>& points1,
            const std::vector<cv::KeyPoint>& points2,
            const int matchThreshold = 100,
            const float maxRatio = 1.0f,
            const int maxOctaveDiff = 8);

       /**
        * @brief Find matched features within a radius
        *
        * @param[in] features1 ORB feature set 1, specified as an M-by-32 cv::Mat.
        * @param[in] features2 ORB feature set 2, specified as an N-by-32 cv::Mat.
        * @param[in] points1 Key points corresponding to features1, specified as an M-element vector of cv::KeyPoint.
        * @param[in] points2 Key points corresponding to features2, specified as an N-element vector of cv::KeyPoint.
        * @param[in] centerPoints Expected matched locations in the second image that correspond to the feature 
        *            points from features1, specified as an M-by-2 cv::Mat.
        * @param[in] radius Search radius associated with the center points, specified as an M-element vector of float values.
        * @param[in] matchThreshold Matching threshold, specified as an integer. The maximum value is 256.
        * @param[in] maxRatio Ratio threshold, specified as a float value. 
        * @param[in] maxOctaveDiff Octave level threshold, specified as an integer. 
        * @return Indices of matched features, returned as a vector of pairs with features1 indices monotonically increasing.
        */
        std::vector<std::pair<int, int>> matchFeaturesInRadius(
            const cv::Mat& features1,
            const cv::Mat& features2,
            const std::vector<cv::KeyPoint>& points1,
            const std::vector<cv::KeyPoint>& points2,
            const cv::Mat& centerPoints,
            const std::vector<float>& radius,
            const int matchThreshold = 100,
            const float maxRatio = 1.0f,
            const int maxOctaveDiff = 8);
    }// namespace vslam
}// namespace vision

#endif // MATCHFEATURES_HPP
