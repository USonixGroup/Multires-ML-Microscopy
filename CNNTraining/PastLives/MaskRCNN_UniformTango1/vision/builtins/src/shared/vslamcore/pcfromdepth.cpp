///////////////////////////////////////////////////////////////////////////
// Convert a depth image to a point cloud
// 
// Copyright 2023 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/pcfromdepth.hpp"
#else
    #include "pcfromdepth.hpp"
#endif

namespace vision {
    namespace vslam {
        void pcfromdepth(
            cv::Mat& depthImage,
            const cv::Matx33d& intrinsics,
            const ConfigurationRGBD& config,
            QueueInstanceRGBD& qObj) {

            qObj.xyzPoints.clear();
            qObj.validIndex.clear();
            qObj.xyzPoints.reserve(qObj.currPoints.size());
            qObj.validIndex.reserve(qObj.currPoints.size());

            if (depthImage.type() != CV_32F)
                depthImage.convertTo(depthImage, CV_32F, 1.0 / config.rgbdParams.depthScaleFactor);
            else
                depthImage = depthImage / config.rgbdParams.depthScaleFactor;

            int featureIdx{ 0 };
            for (const auto& imgPt : qObj.currPoints) {

                const float Z = depthImage.at<float>(static_cast<cv::Point2i>(imgPt.pt));
                const float X = (imgPt.pt.x - intrinsics(0, 2)) / intrinsics(0, 0) * Z;
                const float Y = (imgPt.pt.y - intrinsics(1, 2)) / intrinsics(1, 1) * Z;

                // Filter valid points using depth range
                if (Z >= config.rgbdParams.depthRange[0] && Z <= config.rgbdParams.depthRange[1]) {
                    qObj.xyzPoints.push_back(cv::Vec3d(X, Y, Z));
                    qObj.validIndex.push_back(featureIdx);
                }
                featureIdx++;
            }
        }
    }
}