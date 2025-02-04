////////////////////////////////////////////////////////////////////////////////
// Perform map initialization using homography or fundamental matrix
//
// Copyright 2022-2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef INITIALIZEMAP_HPP
#define INITIALIZEMAP_HPP

#include "MapPointSet.hpp"
#include "KeyFrameSet.hpp"
#include "StereoReconstructor.hpp"
#include "correctLoop.hpp"
#include "matchFeatures.hpp"
#include "reconstructFromHomography.hpp"
#include "reconstructFromFundamentalMatrix.hpp"
#include "bundleAdjustment.hpp"
#include "geometry.hpp"
#include "NodeIDGenerator.hpp"
#include "QueueInstance.hpp"

namespace vision {
    namespace vslam {

        class MapPointSet;
        class KeyFrameSet;
        class StereoReconstructor;

        /**
        * @brief Perform map initialization for monocular vSLAM using homography or fundamental matrix between previous view and current view
        *
        * @param[in/out] mpSet, a MapPointSet object. If map initialization is successful, mpSet will contain the 
        *           triangulated 3-D world points found.
        * @param[in/out] kfSet, a KeyFrameSet object containing camera poses.
        * @param[in/out] database instance to detect loop closure and store inverted image index.
        * @param[in/out] qObj, a QueueInstanceBase struct containing data for the current frame.
        * @param[in] intrinsics, intrinsic parameters of the camera, represented as a cv::Matx33d.
        * @param[in] config, a vision::vslam::Configuration struct.
        * @param[in] generator, an ID generator used to gerenate pointIds and viewIds.
        * @return boolean indicating if map initialization was successful.
        *
        * Note: the function writes to the following members of qObj:
        *   qObj.trackedMapPointIds IDs of tracked map points, returned as a vector of integers.
        *   qObj.trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        *   qObj.currKeyFrameId, an integer representing the ID of the current key frame.
        */
        bool initializeMapMono(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            const ConfigurationMono& config,
            NodeIDGenerator& generator);

        /**
        * @brief Perform map initialization for stereo vSLAM using disparity reconstruction
        *
        * @param[in/out] mpSet, a MapPointSet object. If map initialization is successful, mpSet will contain the 
        *           triangulated 3-D world points found.
        * @param[in/out] kfSet, a KeyFrameSet object containing camera poses.
        * @param[in/out] database instance to detect loop closure and store inverted image index.
        * @param[in] reconstructor StereoReconstructor object used to construct 3-D world points.
        * @param[in/out] qObj, a QueueInstanceStereo struct containing data for the current stereo frame.
        * @param[in] intrinsics, intrinsic parameters of the camera, represented as a cv::Matx33d.
        * @param[in] config, a vision::vslam::Configuration struct.
        * @param[in] generator, an ID generator used to gerenate pointIds and viewIds.
        * @return boolean indicating if map initialization was successful.
        *
        * Note: the function writes to the following members of qObj:
        *   qObj.trackedMapPointIds IDs of tracked map points, returned as a vector of integers.
        *   qObj.trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        *   qObj.currKeyFrameId, an integer representing the ID of the current key frame.
        */
        bool initializeMapStereo(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            const StereoReconstructor& reconstructor,
            QueueInstanceStereo& qObj,
            const cv::Matx33d& intrinsics,
            const ConfigurationStereo& config,
            NodeIDGenerator& generator);

        /**
        * @brief Perform map initialization for vSLAM using RGB-D camera
        *
        * @param[in/out] mpSet, a MapPointSet object. If map initialization is successful, mpSet will contain the 
        *           triangulated 3-D world points found.
        * @param[in/out] kfSet, a KeyFrameSet object containing camera poses.
        * @param[in/out] database instance to detect loop closure and store inverted image index.
        * @param[in/out] qObj, a QueueInstanceRGBD struct containing data for the current RGBD frame.
        * @param[in] intrinsics, intrinsic parameters of the camera, represented as a cv::Matx33d.
        * @param[in] config, a vision::vslam::Configuration struct.
        * @param[in] generator, an ID generator used to gerenate pointIds and viewIds.
        * @return boolean indicating if map initialization was successful.
        *
        * Note: the function writes to the following members of qObj:
        *   qObj.trackedMapPointIds IDs of tracked map points, returned as a vector of integers.
        *   qObj.trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        *   qObj.currKeyFrameId, an integer representing the ID of the current key frame.
        */
        bool initializeMapRGBD(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            QueueInstanceRGBD& qObj,
            const cv::Matx33d& intrinsics,
            const Configuration& config,
            NodeIDGenerator& generator);
    }
}
#endif // INITIALIZEMAP_HPP
