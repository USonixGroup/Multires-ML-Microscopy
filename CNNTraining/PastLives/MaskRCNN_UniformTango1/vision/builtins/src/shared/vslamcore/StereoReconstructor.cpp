///////////////////////////////////////////////////////////////////////////////////////
// Find valid matched feature pairs and reconstruct sparse 3-D points for stereo images
// 
// Copyright 2023-2024 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/StereoReconstructor.hpp"
#else
    #include "StereoReconstructor.hpp"
#endif

namespace vision {
    namespace vslam {

        StereoReconstructor::StereoReconstructor(const cv::Matx33d& intrinsics, const double baseline, const ConfigurationStereo& config) :
            sgbm(cv::StereoSGBM::create(config.stereoParams.minDisparity, config.stereoParams.maxDisparity-config.stereoParams.minDisparity,
                blockSize, P1, P2, disp12MaxDiff, preFilterCap, config.stereoParams.uniquenessThreshold)),
            reprojectionMatrix(1, 0, 0, -intrinsics(0,2),
                               0, 1, 0, -intrinsics(1,2),
                               0, 0, 0, intrinsics(0,0),
                               0, 0, 1.0/baseline, 0),
            maxDepth(config.stereoParams.maxDepthFactor * baseline)
        {}

        StereoReconstructor::~StereoReconstructor() {}

        void StereoReconstructor::reconstruct(
            const QueueInstanceStereo& qObj,
            std::vector<std::pair<int, int>>& matchedPairs,
            std::vector<cv::Vec3d>& xyzPoints,
            const ConfigurationStereo& config) const {

            cv::Mat xyzPointsAll;
            if (qObj.disparity.empty()) {
                // Compute disparity map using cv::StereoSGBM object
                cv::Mat disparity;
                sgbm->compute(qObj.frameL, qObj.frameR, disparity);

                // Convert 16-bit fixed-point disparity map to float
                // Divide by 16 = (2 ^ fractional-bits) as compute() gives 4 fractional bit output.
                disparity.convertTo(disparity, CV_32FC1, 1/16.);

                // Construct dense point cloud from disparity
                cv::reprojectImageTo3D(disparity, xyzPointsAll, reprojectionMatrix);
            }
            else {
                // Construct dense point cloud from disparity
                cv::reprojectImageTo3D(qObj.disparity, xyzPointsAll, reprojectionMatrix);
            }

            // Find valid feature pairs and corresponding world points
            xyzPoints.reserve(matchedPairs.size());
            std::vector<std::pair<int, int>> validPairs;
            validPairs.reserve(matchedPairs.size());

            cv::Point2f shift;
            for (const auto& mp : matchedPairs) {
                const cv::KeyPoint& point2dL = qObj.currPoints[mp.first], point2dR = qObj.pointsR[mp.second];
                const cv::Vec3f& point3f = xyzPointsAll.at<cv::Vec3f>(static_cast<cv::Point2i>(point2dL.pt));

                // Get XY difference between matched points in left and right images.
                shift = point2dL.pt-point2dR.pt;

                if (shift.x > config.stereoParams.minDisparity && shift.x < config.stereoParams.maxDisparity &&      // Is valid disparity
                    point2dL.octave == point2dR.octave &&                                  // Is octave identical
                    point3f[2] > 0 && point3f[2] < maxDepth &&                             // Is valid depth (+Z)
                    std::abs(shift.y) < 2*std::pow(config.baseParams.scaleFactor, point2dR.octave)) { // Is close to epipolar line

                    validPairs.push_back(mp);
                    xyzPoints.push_back(point3f);
                }
            }

            matchedPairs = std::move(validPairs);
        }
    }// namespace vslam
}// namespace vision

