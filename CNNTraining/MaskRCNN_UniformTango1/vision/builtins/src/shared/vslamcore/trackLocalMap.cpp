////////////////////////////////////////////////////////////////////////////////
// Estimate the current camera extrinsics by tracking the local map
// 
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#include <numeric> // iota

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/View.hpp"
    #include "vslamcore/bundleAdjustment.hpp"
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/WorldPoint.hpp"
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
    #include "vslamcore/trackLocalMap.hpp"
    #include "vslamcore/matchFeatures.hpp"
    #include "vslamcore/converter.hpp"
#else
    #include "View.hpp"
    #include "bundleAdjustment.hpp"
    #include "Connection.hpp"
    #include "WorldPoint.hpp"
    #include "MapPointSet.hpp"
    #include "KeyFrameSet.hpp"
    #include "trackLocalMap.hpp"
    #include "matchFeatures.hpp"
    #include "converter.hpp"
#endif

namespace vision {
    namespace vslam {
        bool trackLocalMap(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            const int lastKeyFrameIndex,
            const int currFrameIndex,
            const Configuration& config,
            const IMUInfo& imuInfo,
            const std::vector<double>& viewGyro,
            const std::vector<double>& viewAccel) {

            // The reference key frame, i.e., the last key frame, has the most covisible map points
            int lastKeyFrameId = kfSet.getMaxViewId();
            auto refKeyFrame = kfSet.findView(kfSet.getMaxViewId());
            const int numPointsRefKeyFrame = static_cast<int>(refKeyFrame->getWorldPointsInView().size());

            // Check if tracking local map is needed
            const bool tooFewNonKeyFrames = currFrameIndex < lastKeyFrameIndex + config.baseParams.numSkippedFrames;
            const bool tooManyMapPoints = static_cast<int>(qObj.trackedFeatureIndices.size()) >= config.baseParams.minNumPointWeakFrame;

            if (tooFewNonKeyFrames && tooManyMapPoints) {
                // Reset flags
                for (const auto& wpId : qObj.trackedMapPointIds) {
                    mpSet.setLocal(wpId, false);
                }
                return false;
            }
            else {
                // If only two key frames are available, there is no need to track the local map
                if (kfSet.getNumViews() <= 2) {
                    // Reset flags
                    for (const auto& wpId : qObj.trackedMapPointIds) {
                        mpSet.setLocal(wpId, false);
                    }
                    return true;
                }
            }

            // Find local key frames and local map points that are not included in trackedMapPointIds
            std::vector<int> distances;
            constexpr int maxDistance{ 2 };
            kfSet.connectedViews(lastKeyFrameId, qObj.localKeyFrameIds, distances, maxDistance, config.baseParams.minNumMatches);
            constexpr int minNumLocalKeyFrames{ 5 };
            // If not enough local key frames can be found, reduce the threshold on connectivity strength
            if (static_cast<int>(kfSet.getNumViews()) > minNumLocalKeyFrames && 
                static_cast<int>(qObj.localKeyFrameIds.size()) < minNumLocalKeyFrames) {
                kfSet.connectedViews(lastKeyFrameId, qObj.localKeyFrameIds, distances, maxDistance, config.baseParams.minNumMatches/2);
            }
            qObj.localKeyFrameIds.push_back(lastKeyFrameId);

            std::vector<int> listOfFrameIndices;
            std::vector<int> localMapPointIds;
            std::vector<cv::Vec3d> localPoints3d;
            listOfFrameIndices.reserve(qObj.localKeyFrameIds.size());
            localMapPointIds.reserve(qObj.trackedFeatureIndices.size() * qObj.localKeyFrameIds.size());
            localPoints3d.reserve(qObj.trackedFeatureIndices.size() * qObj.localKeyFrameIds.size());

            for (const auto& kframeId : qObj.localKeyFrameIds) {
                auto kframe = kfSet.findView(kframeId);
                std::unordered_map<int, int> pointIds = kframe->getWorldPointsInView();

                listOfFrameIndices.push_back(kfSet.getFrameIndexByViewId(kframeId));
                for (auto& wp : pointIds) {
                    auto pt(mpSet.getWorldPoint(wp.first));
                    if (!mpSet.isLocal(wp.first)) {
                        localMapPointIds.push_back(wp.first);
                        mpSet.setLocal(wp.first, true);
                        localPoints3d.push_back(pt->getLocation());
                    }
                }
            }

            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::localKeyFramesAndPoints", 
                                           {std::to_string(static_cast<int>(localPoints3d.size()))});
            // Print frame ids in the next line
            config.loggerPtr->logMessageV2("", {convertVectorToString<int>(listOfFrameIndices)});

            if (localPoints3d.empty())
                return false;

            // Project the local map points onto the current frame
            cv::Vec3d currExtrinsics_Rvec;
            cv::Rodrigues(qObj.currExtrinsics_R, currExtrinsics_Rvec);

            cv::Matx14d distCoeffs = cv::Matx14d::zeros();
            cv::Mat projectedPoints, projectedPoints2d;
            cv::projectPoints(localPoints3d, currExtrinsics_Rvec, qObj.currExtrinsics_t, intrinsics, distCoeffs, projectedPoints);
            projectedPoints2d = projectedPoints.reshape(1, localPoints3d.size()); // covert from 2-channel to 1-channel

            // Find the valid map points which are
            // 1) within the image bounds
            // 2) within 60-degree parallax
            // 3) within the distance limit
            std::vector<int> validPointIndices;
            validPointIndices.reserve(projectedPoints2d.rows);
            cv::Matx33d currPose_R;
            cv::Vec3d currPose_t;
            extr2pose(qObj.currExtrinsics_R, qObj.currExtrinsics_t, currPose_R, currPose_t);
            const float distCoef{ config.baseParams.scaleFactor * config.baseParams.scaleFactor };
            for (int i = 0; i != projectedPoints2d.rows; ++i) {
                auto pt(mpSet.getWorldPoint(localMapPointIds[i]));

                // Check image boundary
                const bool isWithinImageBounds =
                    projectedPoints2d.at<double>(i, 0) > 0 &&
                    projectedPoints2d.at<double>(i, 0) < config.baseParams.nCols &&
                    projectedPoints2d.at<double>(i, 1) > 0 &&
                    projectedPoints2d.at<double>(i, 1) < config.baseParams.nRows;

                // Check parallax
                cv::Vec3d pointsToCamera = currPose_t - localPoints3d[i];
                cv::Vec3d viewingDirection = pt->getViewingDirection(); // Unit vector
                const double pointsToCameraDist = cv::norm(pointsToCamera);
                const bool isWithinParallax = viewingDirection.dot(pointsToCamera) >= 0.5 * pointsToCameraDist; // cosd(60) = 0.5;

                // Check distance to camera
                const auto distanceLimits = pt->getDistanceLimits();
                const bool isWithinDistance = pointsToCameraDist <= distanceLimits.second * distCoef &&
                    pointsToCameraDist >= distanceLimits.first / distCoef; // +/- 2 levels
                if (isWithinImageBounds && isWithinParallax && isWithinDistance) {
                    validPointIndices.push_back(i);
                }

            }
            if (validPointIndices.empty())
                return false;

            // Get the representative features and key points of the valid local map points
            cv::Mat localFeatures(validPointIndices.size(), 32, CV_8U);
            cv::Mat validProjectedPoints2d(validPointIndices.size(), 2, CV_64F);
            std::vector<cv::KeyPoint> localKeyPoints;
            localKeyPoints.reserve(validPointIndices.size());

            for (std::size_t i = 0; i != validPointIndices.size(); ++i) {
                int idx = validPointIndices[i];
                auto pt(mpSet.getWorldPoint(localMapPointIds[idx]));

                pt->getRepresentativeFeatureDescriptor(kfSet).copyTo(localFeatures.row(i));
                localKeyPoints.push_back(pt->getRepresentativeFeaturePoint(kfSet));
                projectedPoints2d.row(idx).copyTo(validProjectedPoints2d.row(i));

            }

            // Find the unmatched features in the current frame
            std::vector<int> allFeatureIndices(qObj.currFeatures.rows);
            std::vector<int> currUnmatchedFeatureIndices(qObj.currFeatures.rows - qObj.trackedFeatureIndices.size());
            std::iota(allFeatureIndices.begin(), allFeatureIndices.end(), 0);

            std::vector<int>  trackedFeatureIndicesSorted(qObj.trackedFeatureIndices);
            std::sort(trackedFeatureIndicesSorted.begin(), trackedFeatureIndicesSorted.end());
            // set_difference requires the second set to be assorted
            std::set_difference(allFeatureIndices.begin(), allFeatureIndices.end(),
                trackedFeatureIndicesSorted.begin(), trackedFeatureIndicesSorted.end(), currUnmatchedFeatureIndices.begin());

            cv::Mat currUnmatchedFeatures(currUnmatchedFeatureIndices.size(), 32, CV_8U);
            std::vector<cv::KeyPoint> currUnmatchedPoints;
            currUnmatchedPoints.reserve(currUnmatchedFeatureIndices.size());
            for (std::size_t i = 0; i != currUnmatchedFeatureIndices.size(); ++i) {
                qObj.currFeatures.row(currUnmatchedFeatureIndices[i]).copyTo(currUnmatchedFeatures.row(i));
                currUnmatchedPoints.push_back(qObj.currPoints[currUnmatchedFeatureIndices[i]]);
            }

            validProjectedPoints2d.convertTo(validProjectedPoints2d, CV_32F);
            constexpr float radius{ 4.0f };
            std::vector<float> searchRadius(validProjectedPoints2d.rows, radius * distCoef);
            constexpr int maxOctaveDiff{ 2 };

            std::vector<std::pair<int, int>> matchedPairsInRadius =
                matchFeaturesInRadius(localFeatures, currUnmatchedFeatures, localKeyPoints, currUnmatchedPoints,
                    validProjectedPoints2d, searchRadius, config.baseParams.matchThreshold, config.baseParams.maxRatioInRadius, maxOctaveDiff);

            for (const auto& pair : matchedPairsInRadius) {
                // Add more tracked map points and feature indices
                qObj.trackedMapPointIds.push_back(localMapPointIds[validPointIndices[pair.first]]);
                qObj.trackedFeatureIndices.push_back(currUnmatchedFeatureIndices[pair.second]);
            }

            // Check if the current frame is a key frame
            const bool tooFewMapPoints = static_cast<int>(qObj.trackedFeatureIndices.size()) < config.baseParams.minNumPointWeakFrame;
            const bool isLargeViewChange = qObj.trackedFeatureIndices.size() < 0.9 * numPointsRefKeyFrame;
            bool isKeyFrame = (!tooFewNonKeyFrames || tooFewMapPoints) && isLargeViewChange;
            if (isKeyFrame) {
                if(tooFewMapPoints)
                    config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::keyFrameDetectedWeakFrame");
                else
                    config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::keyFrameDetectedManyNonKeyFrames");

                // Refine the camera pose with all the 3-D to 2-D correspondences
                std::vector<cv::Vec2f> matchedInRadiusImagePoints;
                std::vector<cv::Vec3d> matchedInRadiusWorldPoints;
                matchedInRadiusImagePoints.reserve(matchedPairsInRadius.size());
                matchedInRadiusWorldPoints.reserve(matchedPairsInRadius.size());

                for (std::vector<int>::size_type i = 0; i != qObj.trackedFeatureIndices.size(); ++i) {
                    auto pt(mpSet.getWorldPoint(qObj.trackedMapPointIds[i]));

                    matchedInRadiusImagePoints.push_back(qObj.currPoints[qObj.trackedFeatureIndices[i]].pt);
                    matchedInRadiusWorldPoints.push_back(pt->getLocation());

                }

                // Motion-only bundle adjustment
                cv::Rodrigues(currExtrinsics_Rvec, qObj.currExtrinsics_R);

                bundleAdjustmentMotionOnly(intrinsics, matchedInRadiusWorldPoints, matchedInRadiusImagePoints,
                    qObj.currExtrinsics_R, qObj.currExtrinsics_t, config,
                    imuInfo, refKeyFrame, viewGyro, viewAccel);
            }

            // Set flags back
            for (const auto& wpId : qObj.trackedMapPointIds) {
                mpSet.setLocal(wpId, false);
            }

            return isKeyFrame;
        }
    }
}
