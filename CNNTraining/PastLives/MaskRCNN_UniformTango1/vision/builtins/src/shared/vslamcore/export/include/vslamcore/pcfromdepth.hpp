///////////////////////////////////////////////////////////////////////////
// Convert a depth image to a point cloud
// 
// Copyright 2023 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////

#ifndef PCFROMDEPTH_HPP
#define PCFROMDEPTH_HPP

#include <vector>

#include "opencv2/core.hpp"
#include "Configuration.hpp"
#include "QueueInstance.hpp"

namespace vision {
    namespace vslam {
        /**
        * @brief Convert a depth image using camera intrinsics, into a point cloud. 
        *
        * @param[in] depthImage depth image, specified as a cv::Mat.
        * @param[in] intrinsics camera intrinsics parameters, specified as a 3-by-3 matrix.
        * @param[in/out] qObj, a QueueInstanceRGBD struct containing data for the current frame.
        * @param[in] config configuration parameters of the SLAM system.
        *
        * Note: the function writes to the following members of qObj:
        *   qObj.xyzPoints the output point cloud, returned as an M-element 3-D world coordinates with 
        *       its origin centered at the camera.
        *   qObj.validIndex a vector containing sorted indices of features whose corresponding 3-D world points are
        *       within the desired depth range.
        */

        void pcfromdepth(
            cv::Mat& depthImage,
            const cv::Matx33d& intrinsics,
            const ConfigurationRGBD& config,
            QueueInstanceRGBD& qObj);
    }// namespace vslam
}// namespace vision
#endif //PCFROMDEPTH_HPP

