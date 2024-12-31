////////////////////////////////////////////////////////////////////////////////
//  IMUInfo.hpp contains information to track the state of the IMU fusion
//
//  IMUInfo struct header file.
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef IMUINFO_HPP
#define IMUINFO_HPP

#include <vector>
#include "opencv2/opencv.hpp"

namespace vision {
namespace vslam {

struct IMUInfo {

    IMUInfo() : numKeyFramesPostAlignment{ 0 },
                poseScale{ 1 },
                isIMUAligned{ false },
                isGSAvailable{ false },
                gravityRotationTransform(cv::Matx44d::eye()),
                camToIMUTransform(cv::Matx44d::eye()) { }   // default Constructor

    // Counter used to track the number of KF after a successful GS estimation
    int numKeyFramesPostAlignment;

    // Metric scale used to convert the camera poses to the same scale of the IMU
    double poseScale;

    // Flag to signal if the results of a successful GS estimation is available
    bool isGSAvailable;

    // Flag to signal if the camera and IMU frames have been aligned
    bool isIMUAligned;

    // 4x4 transform containing the rotation needed to align the camera global frame with the IMU local frame
    cv::Matx44d gravityRotationTransform;

    // 4x4 transform containing extrinsics between the camera and IMU sensors
    cv::Matx44d camToIMUTransform;
};

} // namespace vslam
} // namespace vision
#endif // IMUINFO_HPP