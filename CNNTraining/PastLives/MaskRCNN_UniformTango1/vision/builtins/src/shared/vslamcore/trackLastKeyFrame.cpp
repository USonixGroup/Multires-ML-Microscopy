////////////////////////////////////////////////////////////////////////////////
// Estimate the current camera extrinsics by tracking the previous key frame
// 
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/View.hpp"
    #include "vslamcore/bundleAdjustment.hpp"
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/WorldPoint.hpp"
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
    #include "vslamcore/trackLastKeyFrame.hpp"
    #include "vslamcore/matchFeatures.hpp"
#else
    #include "View.hpp"
    #include "bundleAdjustment.hpp"
    #include "Connection.hpp"
    #include "WorldPoint.hpp"
    #include "MapPointSet.hpp"
    #include "KeyFrameSet.hpp"
    #include "trackLastKeyFrame.hpp"
    #include "matchFeatures.hpp"
#endif

namespace vision {
    namespace vslam {
        void trackLastKeyFrame(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            const Configuration& config,
            const IMUInfo& imuInfo,
            const std::vector<double>& viewGyro,
            const std::vector<double>& viewAccel) {

            qObj.trackedFeatureIndices.clear();
            qObj.trackedMapPointIds.clear();

            // Find features in the last key frame with known 3-D locations
            auto lastKeyFrame = kfSet.findView(kfSet.getMaxViewId());
            std::unordered_map<int, int> worldPoints = lastKeyFrame->getWorldPointsInView();

            if (static_cast<int>(worldPoints.size()) < config.baseParams.minNumPnPPoints) {
                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::WorldPointsNotEnough",
                                                {std::to_string(qObj.currFrameIndex + 1), 
                                                 std::to_string(static_cast<int>(worldPoints.size())), std::to_string(config.baseParams.minNumPnPPoints)});
                return;
            } else if (qObj.currFeatures.rows < config.baseParams.minNumPnPPoints) {
                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::ExtractedPointsNotEnough",
                                                {std::to_string(qObj.currFrameIndex + 1), 
                                                 std::to_string(qObj.currFeatures.rows), std::to_string(config.baseParams.minNumPnPPoints)});
                return;
            }

            std::vector<int> preWorldPointIds;
            std::vector<cv::Vec3d> prePoints3d;
            std::vector<cv::KeyPoint> preKeyPoints;
            preWorldPointIds.reserve(worldPoints.size());
            prePoints3d.reserve(worldPoints.size());
            preKeyPoints.reserve(worldPoints.size());

            cv::Mat preMatchedFeatures(worldPoints.size(), 32, CV_8U);
            int idx = 0;
            for (const auto& wp : worldPoints) {
                auto pt(mpSet.getWorldPoint(wp.first));
                lastKeyFrame->getFeatureDescriptors(wp.second).copyTo(preMatchedFeatures.row(idx));
                preWorldPointIds.push_back(wp.first);
                prePoints3d.push_back(pt->getLocation());
                preKeyPoints.push_back(lastKeyFrame->getFeaturePoints(wp.second));
                idx++;
            }

            // Match features in the last key frame with known 3-D locations
            constexpr int maxOctaveDiff{ 2 };
            std::vector<std::pair<int, int>> matchedPairs = matchFeatures(
                preMatchedFeatures, qObj.currFeatures, preKeyPoints, qObj.currPoints, config.baseParams.matchThreshold, config.baseParams.maxRatio, maxOctaveDiff);

            std::vector<cv::Vec2f> matchedImagePoints;
            std::vector<cv::Vec3d> matchedWorldPoints;
            matchedImagePoints.reserve(matchedPairs.size());
            matchedWorldPoints.reserve(matchedPairs.size());

            bool isWeakTracking{ false };
            int numMatches = static_cast<int>(matchedPairs.size());
            // When tracking from the last key frame is weak, skip ratio check to get more matched feature points which can be refined later
            if (numMatches < config.baseParams.minNumPnPPoints * 2) {
                matchedPairs = matchFeatures(
                    preMatchedFeatures, qObj.currFeatures, preKeyPoints, qObj.currPoints, config.baseParams.matchThreshold, 1.0f, maxOctaveDiff);
                isWeakTracking = true;
                numMatches = static_cast<int>(matchedPairs.size());
                // If still not enough number of matched points can be found, the tracking will be lost
                if (numMatches < config.baseParams.minNumPnPPoints) {
                    config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::MatchedPointsNotEnough",
                        { std::to_string(qObj.currFrameIndex + 1), std::to_string(numMatches),
                         std::to_string(config.baseParams.minNumPnPPoints) });
                    return;
                }
            }

            for (const auto& pair : matchedPairs) {
                matchedImagePoints.push_back(qObj.currPoints[pair.second].pt);
                matchedWorldPoints.push_back(prePoints3d[pair.first]);
            }

            // Estimate the current camera pose by solving an PnP problem
            const cv::Matx14d distCoeffs = cv::Matx14d::zeros(); // No lens distortion
            constexpr double confidence{ 0.99 };
            constexpr int maxIter{ 10000 };
            std::vector<int> inlierIdx;
            cv::Vec3d currExtrinsics_Rvec;
            cv::solvePnPRansac(matchedWorldPoints, matchedImagePoints, intrinsics,
                distCoeffs, currExtrinsics_Rvec, qObj.currExtrinsics_t, false, maxIter,
                isWeakTracking? config.baseParams.maxReprojError*2 : config.baseParams.maxReprojError,
                confidence, inlierIdx, cv::SOLVEPNP_EPNP);

            // Allow larger error when the number of inliers is small
            constexpr double inlierRatio{ 0.6 };
            if (static_cast<int>(inlierIdx.size()) < config.baseParams.minNumPnPPoints * inlierRatio) {
                cv::solvePnPRansac(matchedWorldPoints, matchedImagePoints, intrinsics,
                    distCoeffs, currExtrinsics_Rvec, qObj.currExtrinsics_t, false, maxIter, 4 * config.baseParams.maxReprojError,
                    confidence, inlierIdx, cv::SOLVEPNP_EPNP);

                config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::NumInlierPointsWithThreshold", {std::to_string(static_cast<int>(inlierIdx.size())),
                                                                                                        std::to_string(config.baseParams.minNumPnPPoints)});

                if (inlierIdx.empty()) {
                    return;
                }
                else {
                    isWeakTracking = true;
                }
            } else{
                config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::NumInlierPointsWithThreshold", {std::to_string(static_cast<int>(inlierIdx.size())),
                                                                                                        std::to_string(config.baseParams.minNumPnPPoints)});
            }

            // Get the inlier points
            std::vector<cv::Vec2f> inlierMatchedImagePoints;
            std::vector<cv::Vec3d> inlierMatchedWorldPoints;
            inlierMatchedImagePoints.reserve(matchedPairs.size());
            inlierMatchedWorldPoints.reserve(matchedPairs.size());

            for (const auto& idx : inlierIdx) {
                inlierMatchedImagePoints.push_back(matchedImagePoints[idx]);
                inlierMatchedWorldPoints.push_back(matchedWorldPoints[idx]);
            }
            
            // Motion-only bundle adjustment
            cv::Rodrigues(currExtrinsics_Rvec, qObj.currExtrinsics_R);

            bundleAdjustmentMotionOnly(intrinsics, inlierMatchedWorldPoints, inlierMatchedImagePoints,
                                          qObj.currExtrinsics_R, qObj.currExtrinsics_t, config,
                                          imuInfo, lastKeyFrame, viewGyro, viewAccel);

            cv::Rodrigues(qObj.currExtrinsics_R, currExtrinsics_Rvec);
            
            // Search for more matches using matchFeaturesInRadius
            cv::Mat projectedPoints, projectedPoints2d;
            cv::projectPoints(prePoints3d, currExtrinsics_Rvec, qObj.currExtrinsics_t, intrinsics, distCoeffs, projectedPoints);
            projectedPoints2d = projectedPoints.reshape(1, prePoints3d.size()); // Convert from 2-channel to 1-channel
            // Find the points that are within the image boundary
            std::vector<int> isInImage;
            isInImage.reserve(projectedPoints2d.cols);
            for (int i = 0; i != projectedPoints2d.rows; ++i) {
                if (projectedPoints2d.at<double>(i, 0) > 0 &&
                    projectedPoints2d.at<double>(i, 0) < config.baseParams.nCols &&
                    projectedPoints2d.at<double>(i, 1) > 0 &&
                    projectedPoints2d.at<double>(i, 1) < config.baseParams.nRows) {
                    isInImage.push_back(i);
                }
            }

            constexpr float radius{ 4.0f };
            std::vector<float> searchRadius(isInImage.size(), radius);
            cv::Mat inlierPreMatchedFeatures(isInImage.size(), 32, CV_8U), inlierProjectedPoints(isInImage.size(), 2, CV_64F);
            std::vector<cv::KeyPoint> inlierPreKeyPoints;
            inlierPreKeyPoints.reserve(isInImage.size());

            for (std::size_t k = 0; k != isInImage.size(); ++k) {
                preMatchedFeatures.row(isInImage[k]).copyTo(inlierPreMatchedFeatures.row(k));
                projectedPoints2d.row(isInImage[k]).copyTo(inlierProjectedPoints.row(k));
                searchRadius[k] *= std::pow(config.baseParams.scaleFactor, preKeyPoints[isInImage[k]].octave) * (isWeakTracking ? 2.0f : 1.0f);
                inlierPreKeyPoints.push_back(preKeyPoints[isInImage[k]]);
            }

            inlierProjectedPoints.convertTo(inlierProjectedPoints, CV_32F);

            std::vector<std::pair<int, int>> matchedPairsInRadius =
                matchFeaturesInRadius(inlierPreMatchedFeatures, qObj.currFeatures, inlierPreKeyPoints, qObj.currPoints,
                    inlierProjectedPoints, searchRadius, config.baseParams.matchThreshold, config.baseParams.maxRatioInRadius, maxOctaveDiff);

            if (static_cast<int>(matchedPairsInRadius.size()) < config.baseParams.minNumPnPPoints * 0.5) {
                // Increase searching radius when the matching is weak
                std::for_each(searchRadius.begin(), searchRadius.end(), [](float& c) { c *= 2; });
                matchedPairsInRadius =
                    matchFeaturesInRadius(inlierPreMatchedFeatures, qObj.currFeatures, inlierPreKeyPoints, qObj.currPoints,
                        inlierProjectedPoints, searchRadius, config.baseParams.matchThreshold, config.baseParams.maxRatioInRadius, maxOctaveDiff);
            }

            if ((static_cast<int>(matchedPairsInRadius.size()) < config.baseParams.minNumPnPPoints * 0.5))
                return;

            // Refine the camera pose again
            matchedImagePoints.clear();
            matchedWorldPoints.clear();
            matchedImagePoints.reserve(matchedPairsInRadius.size());
            matchedWorldPoints.reserve(matchedPairsInRadius.size());
            qObj.trackedFeatureIndices.reserve(matchedPairsInRadius.size());
            qObj.trackedMapPointIds.reserve(matchedPairsInRadius.size());
            for (const auto& pair : matchedPairsInRadius) {
                // Tracked map points
                int wpId = preWorldPointIds[isInImage[pair.first]];

                // Tracked map points
                qObj.trackedMapPointIds.push_back(wpId);

                // Tracked features 
                qObj.trackedFeatureIndices.push_back(pair.second);

                // Prepare for finding local map points in the next step
                mpSet.setLocal(wpId, true);

                matchedImagePoints.push_back(qObj.currPoints[pair.second].pt);
                matchedWorldPoints.push_back(mpSet.getWorldPoint(wpId)->getLocation());

            }

            // Motion-only bundle adjustment
            cv::Rodrigues(currExtrinsics_Rvec, qObj.currExtrinsics_R);

            bundleAdjustmentMotionOnly(intrinsics, matchedWorldPoints, matchedImagePoints,
                                          qObj.currExtrinsics_R, qObj.currExtrinsics_t, config,
                                          imuInfo, lastKeyFrame, viewGyro, viewAccel);
        }
    }
}