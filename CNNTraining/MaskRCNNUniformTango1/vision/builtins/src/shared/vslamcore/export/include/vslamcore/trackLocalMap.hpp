////////////////////////////////////////////////////////////////////////////////
// Estimate the current camera extrinsics by tracking the local map
// 
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef TRACKLOCALMAP_HPP
#define TRACKLOCALMAP_HPP

#include "KeyFrameSet.hpp"
#include "MapPointSet.hpp"
#include "Configuration.hpp"
#include "QueueInstance.hpp"
#include "IMUInfo.hpp"

namespace vision {
    namespace vslam {
        class KeyFrameSet;
        class MapPointSet;

        /**
        * @brief Estimate the current camera extrinsics by tracking the local key frames
        *
        * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
        * @param[in] kfSet a KeyFrameSet object containing camera poses, including the last key frame.
        * @param[in/out] qObj, a QueueInstanceBase struct containing data for the current frame.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d.
        * @param[in] lastKeyFrameIndex frame index of the last key frame, specified as an integer.
        * @param[in] currFrameIndex current frame index, specified as an integer.
        * @param[in] config configuration parameters of the system.
        * @param[in] imuInfo a struct containing information related to the state of IMU fusion.
        * @param[in] viewGyro the gyroscope measurements between the previous and current view.
        * @param[in] viewAccel the accelerometer measurements between the previous and current view.
        *
        * Note: the function writes to the following members of qObj:
        *   qObj.currExtrinsics_R rotation matrix from world coordinate to camera coordinate, returned as a cv::Matx33d.
        *   qObj.currExtrinsics_t translation vector from world coordinate to camera coordinate, returned as a cv::Vec3d.
        *   qObj.trackedMapPointIds IDs of tracked map points, returned as a vector of integers.
        *   qObj.trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        *   qObj.localKeyFrameIds Ids of local key frames that are within distance of 2 to the current frame, returned as a vector of integers.
        */
            
        bool trackLocalMap(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            const int lastKeyFrameIndex,
            const int currFrameIndex,
            const Configuration& config,
            const IMUInfo& imuInfo = IMUInfo(),
            const std::vector<double>& viewGyro = std::vector<double>(),
            const std::vector<double>& viewAccel = std::vector<double>());
    }// namespace vslam
}// namespace vision
#endif //TRACKLOCALMAP
