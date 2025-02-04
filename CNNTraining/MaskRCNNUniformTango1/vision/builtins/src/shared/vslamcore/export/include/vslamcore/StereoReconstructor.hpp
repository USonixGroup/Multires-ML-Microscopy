///////////////////////////////////////////////////////////////////////////
// Find valid matched feature pairs for stereo images
// 
// Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef STEREORECONSTRUCTOR_HPP
#define STEREORECONSTRUCTOR_HPP

#include "Configuration.hpp"
#include "QueueInstance.hpp"

#include <vector>
#include "opencv2/opencv.hpp"

namespace vision {
    namespace vslam {
        class StereoReconstructor {

        public:
            StereoReconstructor() = delete;

            /**
            * @brief Constructor 
            *
            * @param[in] intrinsics Camera intrinsics matrix specified as a cv::Matx33d.
            * @param[in] baseline Distance between stereo cameras specified as a double.
            * @param[in] config Configuration struct
            */
            StereoReconstructor(const cv::Matx33d& intrinsics, const double baseline, const ConfigurationStereo& config);

            /**
            * @brief Destructor
            */
            ~StereoReconstructor();

            /**
            * @brief Get valid feature pairs and 3-D world points from stereo images
            *
            * @param[in] qObj a QueueInstanceStereo struct containing data for the current frame
            * @param[out] matchedPairs Indices of matched features, returned as a vector of pairs.
            * @param[out] xyzPoints 3-D world points in the left camera coordinate constructed from stereo images, specified as vector of cv::Vec3d.
            * @param[in] config Configuration struct
            *
            * Notes:
            *   If qObj.disparity is empty, the function will calculate the disparity using cv::StereoSGBM.
            *   If qObj.disparity is not empty, it must be a cv::Mat of type CV_32FC1 with the same size as qObj.frameL and qObj.frameR.
            */
            void reconstruct(
                const QueueInstanceStereo& qObj,
                std::vector<std::pair<int, int>>& matchedPairs,
                std::vector<cv::Vec3d>& xyzPoints,
                const ConfigurationStereo& config) const;

        private:
            const int blockSize = 15;
            const int P1 = 8*blockSize*blockSize;
            const int P2 = 32*blockSize*blockSize;
            const int disp12MaxDiff = -1; // disable maximum allowed difference in left-right disparity check
            const int preFilterCap = 31; // floor(ContrastThreshold*63)

            const cv::Ptr<cv::StereoSGBM> sgbm;
            const cv::Matx44d reprojectionMatrix;
            const float maxDepth;
        };
    }// namespace vslam
}// namespace vision
#endif //STEREORECONSTRUCTOR_HPP