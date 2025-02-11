////////////////////////////////////////////////////////////////////////////////
// Perform bundle adjustment over camera poses and 3-D world points
// 
// Copyright 2022-2024 The MathWorks, Inc. 
////////////////////////////////////////////////////////////////////////////////

#ifndef BUNDLEADJUSTMENT_HPP
#define BUNDLEADJUSTMENT_HPP

#include <iostream>
#include <stdint.h>
#include <cstdlib>
#include <string>
#include <cmath> //pow, sqrt
#include <numeric>
#include <algorithm>
#include <streambuf>

#include "Configuration.hpp"
#include "converter.hpp"
#include "LoopClosureDatabase.hpp"
#include "IMUInfo.hpp"

namespace vision {
    namespace vslam {
        /**
        * Struct for cout buffer replacement during vslam verbose mode.
        * The following class is required to capture all cout statements from ceres library.
        */
        struct CoutRedirectForCeres {
            CoutRedirectForCeres(std::streambuf * new_buffer) : old(std::cout.rdbuf(new_buffer))
            {}
        
            ~CoutRedirectForCeres() {
                std::cout.rdbuf( old );
            }
        private:
            std::streambuf * old;
        };

        class MapPointSet;
        class KeyFrameSet;

        /**
        * @brief Refine camera poses and 3-D world point locations using Ceres.
        *
        * @param[in] mpSet a MapPointSet object containing 3-D world points.
        * @param[in] kfSet a KeyFrameSet object containing camera poses.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d
        * @param[in] viewIds Id of camera poses to be refined in kfSet, specified as a vector integers.
        * @param[in] fixedViewIds Id of cameras to be fixed during the optimization, specified as a vector integers.
        * @param[in] config contains the configuration of the optimizer.
        * @param[in] abortBA Flag used to terminate BA.
        * @param[in] imuInfo a struct containing information related to the state of IMU fusion.
        */

        void bundleAdjustment(
            MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const cv::Matx33d& intrinsics,
            const std::vector<int>& viewIds,
            const std::vector<int>& fixedViewIds,
            const Configuration& config,
            bool* abortBA = nullptr,
            const IMUInfo& imuInfo = IMUInfo());

        /**
        * @brief Refine current camera pose using Ceres.
        *
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d
        * @param[in] worldPoints World points to be used in optimization.
        * @param[in] imagePoints Image points corresponding to the World points.
        * @param[in] currExtrinsics_R rotation matrix from world coordinate to camera coordinate, specified as a cv::Matx33d.
        * @param[in] currExtrinsics_t translation vector from world coordinate to camera coordinate, specified as a cv::Vec3d.
        * @param[in] config contains the configuration of the optimizer.
        * @param[in] imuInfo a struct containing information related to the state of IMU fusion.
        * @param[in] previousView the previous view.
        * @param[in] viewGyro the gyroscope measurements between the previous and current view.
        * @param[in] viewAccel the accelerometer measurements between the previous and current view.
        */

        void bundleAdjustmentMotionOnly(
            const cv::Matx33d& intrinsics,
            const std::vector<cv::Vec3d>& worldPoints,
            const std::vector<cv::Vec2f>& imagePoints,
            cv::Matx33d& currExtrinsics_R,
            cv::Vec3d& currExtrinsics_t,
            const Configuration& config,
            const IMUInfo& imuInfo = IMUInfo(),
            const std::shared_ptr<View>& previousView = std::shared_ptr<View>(),
            const std::vector<double>& viewGyro = std::vector<double>(),
            const std::vector<double>& viewAccel = std::vector<double>());

    }// namespace vslam
}// namespace vision
#endif //BUNDLEADJUSTMENT_HPP
