////////////////////////////////////////////////////////////////////////////////
//  QueueInstance.hpp
//
//  QueueInstance struct header file.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef QUEUEINSTANCE_HPP
#define QUEUEINSTANCE_HPP

#include <vector>
#include "opencv2/opencv.hpp"

namespace vision {
namespace vslam {

// Base struct used to share data between threads for all vSLAM implementations.
// This includes IMU data members.
struct QueueInstanceBase {
    public:
        int currFrameIndex;
        int currKeyFrameId;
        std::vector<cv::KeyPoint> currPoints;
        cv::Mat currFeatures;
        cv::Matx33d currExtrinsics_R;
        cv::Vec3d currExtrinsics_t;
        cv::Matx33d lastPose_R;
        cv::Vec3d lastPose_t;
        std::vector<int> trackedMapPointIds;
        std::vector<int> trackedFeatureIndices;
        std::vector<int> localKeyFrameIds;
        std::vector<double> accel;
        std::vector<double> gyro;
};

// Adds data members unique to stereo vSLAM
struct QueueInstanceStereo : public QueueInstanceBase {
    public:
        cv::Mat frameL;
        cv::Mat frameR;
        std::vector<cv::KeyPoint> pointsR;
        cv::Mat featuresR;
        cv::Mat disparity;
};

// Adds data members unique to RGBD vSLAM
struct QueueInstanceRGBD : public QueueInstanceBase {
    public:
        std::vector<cv::Vec3d> xyzPoints;
        std::vector<int> validIndex;
};

} // namespace vslam
} // namespace vision
#endif // QUEUEINSTANCE_HPP