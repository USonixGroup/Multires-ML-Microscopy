////////////////////////////////////////////////////////////////////////////////
// Function to correct loop using the given loop candidates.
// 
// Copyright 2022-23 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef DETECT_LOOP_AND_UPDATE_MAP_HPP
#define DETECT_LOOP_AND_UPDATE_MAP_HPP

#include "LoopClosureDatabase.hpp"
#include "KeyFrameSet.hpp"
#include "MapPointSet.hpp"
#include "Configuration.hpp"

namespace vision {
    namespace vslam {

        /**
         * @brief Correct loop using the given the loop candidates.
         *
         * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
         * @param[in/out] kfSet KeyFrameSet object containing views that observe the world point.
         * @param[in/out] database instance to detect loop closure and store inverted image index.
         * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d
         * @param[in] currKeyFrameId frame ID of the current key frame, specified as an integer.
         * @param[in] currFeatures feature descriptors of current key frame.
         * @param[in] loopKeyFrameIds Groups of consecutive keyframe IDs forming loop with current key frame.
         * @param[in] config configuration parameters of the system.
         * @return return true if a loop is closed successfully, otherwise false.
         */

        bool correctLoop(MapPointSet& mpSet, 
                         KeyFrameSet& kfSet,
                         LoopClosureDatabase& database,
                         const cv::Matx33d& intrinsics,
                         const int currKeyFrameId,
                         const std::vector<cv::Mat>& currFeaturesVec,
                         const std::vector<std::vector<int>>& loopKeyFrameIds,
                         const Configuration& config);

    }
}
#endif // DETECT_LOOP_AND_UPDATE_MAP_HPP