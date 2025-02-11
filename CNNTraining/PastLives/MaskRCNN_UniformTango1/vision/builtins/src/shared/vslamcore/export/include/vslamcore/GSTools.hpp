////////////////////////////////////////////////////////////////////////////////
// Tools for handeling scale and gravity alignement estimation
// 
// Copyright 2024 The MathWorks, Inc. 
////////////////////////////////////////////////////////////////////////////////

#ifndef GSTOOLS_HPP
#define GSTOOLS_HPP

namespace vision {
    namespace vslam {
        class MapPointSet;
        class KeyFrameSet;
        struct IMUInfo;

        /**
        * @brief Apply the output of the gravity rotation and scale estimation to the poses and points.
        *
        * @param[in] kfSet a KeyFrameSet object containing camera poses.
        * @param[in] mpSet a MapPointSet object containing 3-D world points.
        * @param[in] imuInfo a struct containing information related to the state of IMU fusion.
        */

        void applyGravityRotationAndScale(
            KeyFrameSet& keyFrames,
            MapPointSet& mapPoints,
            IMUInfo& imuInfo);

    }// namespace vslam
}// namespace vision
#endif //GSTOOLS_HPP
