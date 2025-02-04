///////////////////////////////////////////////////////////////////////////
// Add a new key frame to the data management object
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef ADDNEWKEYFRAME_HPP
#define ADDNEWKEYFRAME_HPP

#include <vector>

#include "Configuration.hpp"
#include "QueueInstance.hpp"

#include "opencv2/core.hpp"

namespace vision {
    namespace vslam {
        class KeyFrameSet;
        class MapPointSet;

        /**
        * @brief Add a new key frame and update 3-D to 2-D correspondences
        *
        * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
        * @param[in/out] kfSet a KeyFrameSet object containing camera poses, including the last key frame.
        * @param[in] qObj, a QueueInstanceBase struct containing data for the current frame.
        * @param[in] config configuration parameters of the SLAM system.
        */

        void addNewKeyFrame(
            MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const QueueInstanceBase& qObj,
            const Configuration& config);
    }// namespace vslam
}// namespace vision
#endif //ADDNEWKEYFRAME_HPP

