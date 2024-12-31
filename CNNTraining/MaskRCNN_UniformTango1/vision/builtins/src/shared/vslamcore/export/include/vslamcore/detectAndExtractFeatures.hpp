////////////////////////////////////////////////////////////////////////////////
// Detect and extract uniformly-distributed ORB features from a grayscale image
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef DETECTANDEXTRACTFEATURES_HPP
#define DETECTANDEXTRACTFEATURES_HPP

#include <algorithm>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"

namespace vision {
    namespace vslam {
       /**
        * @brief Implementation of selectUniform
        *
        * @param[in] points Key points to be downsampled, specified as a vector of cv::KeyPoint.
        * @param[in] nRows Number of rows of the image matrix, specified as an integer.
        * @param[in] nCols Number of columns of the image matrix, specified as an integer.
        * @param[in] isSelected Flags indicating if a key point is selected, specified as a vector of booleans.
        * @return Indices of selected key points, returned as a vector of unsigned integers.
        */
        std::vector<int>  selectUniformImpl(
            const std::vector<cv::KeyPoint>& points,
            const int nRows,
            const int nCols,
            const int numPointsOut,
            std::vector<bool>& isSelected);

       /**
        * @brief Select uniformly-distributed key points
        *
        * @param[in] points Key points to be downsampled, specified as a vector of cv::KeyPoint.
        * @param[in] nRows Number of rows of the image matrix, specified as an integer.
        * @param[in] nCols Number of columns of the image matrix, specified as an integer.
        * @param[in] numPointsOut Number of uniformly selected key points, specified as an unsigned integer.
        * @return Indices of selected key points, returned as a vector of unsigned integers.
        */
        std::vector<int>  selectUniform(
            const std::vector<cv::KeyPoint>& points,
            const int nRows,
            const int nCols,
            const int numPointsOut);

       /**
        * @brief Detect and extract uniformly-distributed ORB features
        * 
        * @param[in] frame Gray image, specified as a cv::Mat. The image is assumed undistorted.
        * @param[in] scaleFactor Pyramid decimation ratio, specified as a float value.
        * @param[in] numLevels 	The number of pyramid levels, specified as an integer.
        * @param[in] numFeatures Number of ORB features to be extracted, specified as an unsigned integer.
        * @param[out] points Key points of the extracted ORB features, returned as a vector of cv::KeyPoint.
        * @param[out] features Extracted ORB feature descriptors, returned as a cv::Mat.
        * 
        */
        void detectAndExtractFeatures(
            const cv::Mat& frame,
            const float scaleFactor,
            const int numLevels,
            const int numFeatures,
            std::vector<cv::KeyPoint>& points,
            cv::Mat& features);
    }// namespace vslam
}// namespace vision

#endif // DETECTANDEXTRACTFEATURES_HPP
