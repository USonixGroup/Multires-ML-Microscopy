///////////////////////////////////////////////////////////////////////////
// Create new map points
// 
// Copyright 2022-23 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////

#ifndef CREATENEWMAPPOINTS_HPP
#define CREATENEWMAPPOINTS_HPP

#include "KeyFrameSet.hpp"
#include "MapPointSet.hpp"
#include "Configuration.hpp"
#include "matchFeatures.hpp"
#include "bundleAdjustment.hpp"
#include "computeReprojectionErrors.hpp"
#include "NodeIDGenerator.hpp"
#include "QueueInstance.hpp"

namespace vision {
    namespace vslam {
        class KeyFrameSet;
        class MapPointSet;
        
        /**
        * @brief Get translation from camera center to 3-D world point
        *
        * @param[in] worldPoint, a 3-D world point represented as a 3-by-1 cv::Mat
        * @param[in] pose, the position of camera center in world coordinate, specifiec as a cv::Vec<3,T>.
        * @param[out] camToPoint, the XYZ translation from camera center to world point, specified as cv::Vec<3,T>.
        */
        template<typename T>
        void getCamToPoint(const cv::Mat& worldPoint, const cv::Vec<T,3>& pose, cv::Vec<T,3>& camToPoint) {
            camToPoint[0] = worldPoint.at<T>(0,0) - pose[0];
            camToPoint[1] = worldPoint.at<T>(1,0) - pose[1];
            camToPoint[2] = worldPoint.at<T>(2,0) - pose[2];
        }

        /**
        * @brief Create new map points using triangulation for monocular visual SLAM
        *
        * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
        * @param[in/out] kfSet a KeyFrameSet object containing camera poses, including the last key frame.
        * @param[in/out] qObj, a QueueInstanceBase struct containing data for the current frame.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d.
        * @param[out] recentMapPointIds IDs of newly created map points, returned as a vector of integers.
        * @param[out] refinedViewIds Id of camera poses to be refined in kfSet, specified as a vector integers.
        * @param[out] fixedViewIds Id of cameras to be fixed during the optimization, specified as a vector integers.
        * @param[in] config configuration parameters of the system.
        *
        * Note: the function writes to the following members of qObj:
        *   qObj.trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        */

        void createNewMapPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            std::vector<int>& recentMapPointIds,
            std::vector<int>& refinedViewIds,
            std::vector<int>& fixedViewIds,
            const Configuration& config,
            NodeIDGenerator& generator);

        /**
        * @brief Create new map points using triangulation for stereo and RGB-D visual SLAM
        *
        * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
        * @param[in/out] kfSet a KeyFrameSet object containing camera poses, including the last key frame.
        * @param[in/out] qObj, a QueueInstanceBase struct containing data for the current frame.
        * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d.
        * @param[in/out] 2-D to 3-D correspondence between feature points and map points created using stereo methods for the current key frame, 
        *           specified as a map between feature indices and map point IDs.
        * @param[out] refinedViewIds Id of camera poses to be refined in kfSet, specified as a vector integers.
        * @param[out] fixedViewIds Id of cameras to be fixed during the optimization, specified as a vector integers.
        * @param[in] config configuration parameters of the system.
        *
        * Note: the function writes to the following members of qObj:
        *   qObj.trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        */

        void createNewMapPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            std::unordered_map<int, int>& stereoCorrespondence,
            std::vector<int>& refinedViewIds,
            std::vector<int>& fixedViewIds,
            const Configuration& config);
    }// namespace vslam
}// namespace vision
#endif //CREATENEWMAPPOINTS_HPP