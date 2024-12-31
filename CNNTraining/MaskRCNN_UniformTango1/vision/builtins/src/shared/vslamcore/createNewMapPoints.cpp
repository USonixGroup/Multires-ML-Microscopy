///////////////////////////////////////////////////////////////////////////
// Create new map points
// 
// Copyright 2022-23 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/View.hpp"
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/WorldPoint.hpp"
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
    #include "vslamcore/createNewMapPoints.hpp"
    #include "vslamcore/converter.hpp"
#else
    #include "View.hpp"
    #include "Connection.hpp"
    #include "WorldPoint.hpp"
    #include "MapPointSet.hpp"
    #include "KeyFrameSet.hpp"
    #include "createNewMapPoints.hpp"
    #include "converter.hpp"
#endif



namespace vision {
    namespace vslam {

        void createNewMapPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            std::vector<int>& recentTriMapPointIds,
            std::vector<int>& refinedViewIds,
            std::vector<int>& fixedViewIds,
            const Configuration& config,
            NodeIDGenerator& generator) {

            recentTriMapPointIds.clear();
            fixedViewIds.clear();
            refinedViewIds.clear();
            
            // Get connected key frames
            std::vector<int> distances;
            constexpr int maxDist{ 2 };
            kfSet.connectedViews(qObj.currKeyFrameId, refinedViewIds, distances, maxDist, config.baseParams.minNumMatchesBA);

            std::vector<int> actualRefinedViewIds;
            actualRefinedViewIds.reserve(refinedViewIds.size());

            std::vector<int> allFeatureIndices(qObj.currFeatures.rows);
            std::iota(allFeatureIndices.begin(), allFeatureIndices.end(), 0);

            // Compute camera projection matrix
            cv::Matx34d prevCamProjMat, currCamProjMat;
            cv::hconcat(intrinsics * qObj.currExtrinsics_R, intrinsics * qObj.currExtrinsics_t, currCamProjMat);
            cv::Vec3d absPose_t = -qObj.currExtrinsics_R.t() * qObj.currExtrinsics_t;

            std::shared_ptr<View> currView = kfSet.findView(qObj.currKeyFrameId);
            cv::Matx33d prevExtrinsics_R;
            cv::Vec3d prevPose_t, prevExtrinsics_t;
            for (std::vector<int>::size_type i = 0; i != refinedViewIds.size(); ++i) {
                int prevViewId = refinedViewIds[i];
                std::shared_ptr<View> prevView = kfSet.findView(prevViewId);

                // Alway fix the very first view
                if (prevViewId == 0) {
                    fixedViewIds.push_back(prevViewId);
                }
                else {
                    // Distance-2 key frames are fixed during bundle adjustment. No reconstruction is performed.
                    if (distances[i] == 2) {
                        if (static_cast<int>(fixedViewIds.size()) < config.baseParams.maxNumFixedViewsBA) // Do not add too many fixed key frames
                        {
                            fixedViewIds.push_back(prevViewId);
                            actualRefinedViewIds.push_back(prevViewId);
                        }
                        continue;
                    }
                }
                actualRefinedViewIds.push_back(prevViewId);

                // Reconstruct new map points with distance-1 key frames
                prevExtrinsics_R = prevView->getOrientation().t();
                prevPose_t = prevView->getLocation();
                prevExtrinsics_t = -prevExtrinsics_R * prevPose_t;

                std::vector<int> prevTrackedFeatureIndices;
                prevTrackedFeatureIndices.reserve(config.baseParams.numFeatures);

                std::unordered_map<int, int> prevWorldPoints = prevView->getWorldPointsInView();

                std::vector<double> depthOfPoint3d;
                depthOfPoint3d.reserve(prevWorldPoints.size());

                for (const auto& wp : prevWorldPoints) {
                    depthOfPoint3d.push_back(cv::norm(mpSet.getWorldPoint(wp.first)->getLocation() - prevPose_t));
                    prevTrackedFeatureIndices.push_back(wp.second);
                }

                // Find median depth
                std::nth_element(depthOfPoint3d.begin(), depthOfPoint3d.begin() + depthOfPoint3d.size() / 2, depthOfPoint3d.end());
                double medianDepth = depthOfPoint3d[depthOfPoint3d.size() / 2];

                // Skip the key frame if the change of view is small
                bool isViewTooClose = cv::norm(prevPose_t - absPose_t) < config.baseParams.minSinParallax * medianDepth;
                if (!isViewTooClose) {
                    // Find the unmatched features in the distance-1 key frame
                    std::vector<int> prevUnmatchedFeatureIndices(prevView->getFeatureDescriptors().rows - prevTrackedFeatureIndices.size());
                    std::vector<int> prevAllFeatureIndices(prevView->getFeatureDescriptors().rows);
                    std::iota(prevAllFeatureIndices.begin(), prevAllFeatureIndices.end(), 0);
                    std::sort(prevTrackedFeatureIndices.begin(), prevTrackedFeatureIndices.end());
                    auto iter = std::set_difference(prevAllFeatureIndices.begin(), prevAllFeatureIndices.end(),
                        prevTrackedFeatureIndices.begin(), prevTrackedFeatureIndices.end(), prevUnmatchedFeatureIndices.begin());
                    cv::Mat prevUnmatchedFeatures(prevUnmatchedFeatureIndices.size(), 32, CV_8U);

                    std::vector<cv::KeyPoint> prevUnmatchedPoints;
                    prevUnmatchedPoints.reserve(prevUnmatchedFeatureIndices.size());
                    for (int i = 0; i != static_cast<int>(prevUnmatchedFeatureIndices.size()); ++i) {
                        prevView->getFeatureDescriptors(prevUnmatchedFeatureIndices[i]).copyTo(prevUnmatchedFeatures.row(i));
                        prevUnmatchedPoints.push_back(prevView->getFeaturePoints(prevUnmatchedFeatureIndices[i]));
                    }

                    // Find the unmatched features in the current key frame
                    std::vector<int> currUnmatchedFeatureIndices(qObj.currFeatures.rows - qObj.trackedFeatureIndices.size());
                    std::sort(qObj.trackedFeatureIndices.begin(), qObj.trackedFeatureIndices.end());
                    std::set_difference(allFeatureIndices.begin(), allFeatureIndices.end(),
                        qObj.trackedFeatureIndices.begin(), qObj.trackedFeatureIndices.end(), currUnmatchedFeatureIndices.begin());

                    cv::Mat currUnmatchedFeatures(currUnmatchedFeatureIndices.size(), 32, CV_8U);
                    std::vector<cv::KeyPoint> currUnmatchedPoints;
                    currUnmatchedPoints.reserve(currUnmatchedFeatureIndices.size());
                    for (int i = 0; i != static_cast<int>(currUnmatchedFeatureIndices.size()); ++i) {
                        qObj.currFeatures.row(currUnmatchedFeatureIndices[i]).copyTo(currUnmatchedFeatures.row(i));
                        currUnmatchedPoints.push_back(qObj.currPoints[currUnmatchedFeatureIndices[i]]);
                    }

                    // Match features
                    constexpr int maxOctaveDiff{ 2 };
                    std::vector<std::pair<int, int>> matchedPairs = matchFeatures(
                        prevUnmatchedFeatures, currUnmatchedFeatures, prevUnmatchedPoints, currUnmatchedPoints,
                        config.baseParams.matchThreshold, config.baseParams.maxRatioMapping, maxOctaveDiff);

                    if (static_cast<int>(matchedPairs.size()) < config.baseParams.minNumMatchesMapping)
                        continue;

                    auto conn = kfSet.findConnection(prevViewId, qObj.currKeyFrameId);

                    // Create 3-D points from triangulation
                    cv::hconcat(intrinsics * prevExtrinsics_R, intrinsics * prevExtrinsics_t, prevCamProjMat);

                    std::vector<cv::Point2f> matchedPrevImagePoints, matchedCurrImagePoints;
                    matchedPrevImagePoints.reserve(matchedPairs.size());
                    matchedCurrImagePoints.reserve(matchedPairs.size());

                    for (const auto& pair : matchedPairs) {
                        matchedPrevImagePoints.push_back(prevUnmatchedPoints[pair.first].pt);
                        matchedCurrImagePoints.push_back(currUnmatchedPoints[pair.second].pt);
                    }

                    // Triangulation to create new 3-D map points
                    cv::Mat worldPoints;
                    triangulatePoints(prevCamProjMat, currCamProjMat, matchedPrevImagePoints, matchedCurrImagePoints, worldPoints);

                    // Convert points from 4-D single to 3-D double
                    hom2cart<double>(worldPoints);

                    // Compute reprojection errors
                    std::vector<double> errorsInPrevCamera = computeReprojectionErrors(
                        worldPoints, matchedPrevImagePoints, prevExtrinsics_R, prevExtrinsics_t, intrinsics);

                    std::vector<double> errorsInCurrCamera = computeReprojectionErrors(
                        worldPoints, matchedCurrImagePoints, qObj.currExtrinsics_R, qObj.currExtrinsics_t, intrinsics);

                    cv::Vec3d currCamNormalVec(&qObj.currExtrinsics_R(2, 0)), prevCamNormalVec(&prevExtrinsics_R(2, 0)),
                        currCamCenterToPoint3d, prevCamCenterToPoint3d;

                    for (int j = 0; j != worldPoints.cols; ++j) {
                        // Check if the 3-D point is valid
                        getCamToPoint(worldPoints.col(j), absPose_t, currCamCenterToPoint3d);
                        getCamToPoint(worldPoints.col(j), prevPose_t, prevCamCenterToPoint3d);

                        bool isInFrontOfCamera = currCamCenterToPoint3d.dot(currCamNormalVec) > 0 &&
                            prevCamCenterToPoint3d.dot(prevCamNormalVec) > 0;
                        bool hasLargeParallax = currCamCenterToPoint3d.dot(prevCamCenterToPoint3d) /
                            (cv::norm(currCamCenterToPoint3d) * cv::norm(prevCamCenterToPoint3d)) < config.baseParams.maxCosParallax;
                        bool hasSmallReprojError =
                            errorsInPrevCamera[j] <= config.baseParams.maxReprojError * std::pow(config.baseParams.scaleFactor, prevUnmatchedPoints[matchedPairs[j].first].octave) &&
                            errorsInCurrCamera[j] <= config.baseParams.maxReprojError * std::pow(config.baseParams.scaleFactor, currUnmatchedPoints[matchedPairs[j].second].octave);
                        if (isInFrontOfCamera && hasLargeParallax && hasSmallReprojError) {
                            // Add new map points
                            const int currPointId = generator.newIDs(NodeType::POINT_XYZ).identifier;
                            auto newPoint(std::make_shared<WorldPoint>(currPointId, cv::Vec3d(worldPoints.col(j))));
                            mpSet.addWorldPoint(newPoint);
                            int newFeatureIdxPrev = prevUnmatchedFeatureIndices[matchedPairs[j].first],
                                newFeatureIdxCurr = currUnmatchedFeatureIndices[matchedPairs[j].second];
                            mpSet.addCorrespondence(currPointId, prevViewId, newFeatureIdxPrev, kfSet);
                            mpSet.addCorrespondence(currPointId, qObj.currKeyFrameId, newFeatureIdxCurr, kfSet);

                            // Update represetative view
                            mpSet.updateRepresentativeView(currPointId, kfSet);

                            // Update connections with new feature matches
                            conn->addMatches(std::make_pair(newFeatureIdxPrev, newFeatureIdxCurr));

                            prevView->updateConnectedView(qObj.currKeyFrameId);
                            currView->updateConnectedView(prevViewId);

                            // More map points are tracked
                            recentTriMapPointIds.push_back(currPointId);
                            qObj.trackedFeatureIndices.push_back(newFeatureIdxCurr);
                        }
                    }
                }
            }

            // For local bundle adjustment
            actualRefinedViewIds.push_back(qObj.currKeyFrameId);
            refinedViewIds = std::move(actualRefinedViewIds);
            std::sort(refinedViewIds.begin(), refinedViewIds.end());
            if (fixedViewIds.empty()) {
                // If there is not a distance-2 view, set the oldest 2 views as fixed views
                std::sort(refinedViewIds.begin(), refinedViewIds.end());
                fixedViewIds.push_back(refinedViewIds[0]);
                fixedViewIds.push_back(refinedViewIds[1]);
            }
        }

        void createNewMapPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            std::unordered_map<int, int>& stereoCorrespondence,
            std::vector<int>& refinedViewIds,
            std::vector<int>& fixedViewIds,
            const Configuration& config) {

            // Get connected key frames
            fixedViewIds.clear();
            refinedViewIds.clear();
            std::vector<int> distances;
            constexpr int maxDist{ 2 };
            kfSet.connectedViews(qObj.currKeyFrameId, refinedViewIds, distances, maxDist, config.baseParams.minNumMatchesBA);

            std::vector<int> actualRefinedViewIds;
            actualRefinedViewIds.reserve(refinedViewIds.size());

            std::vector<int> allFeatureIndices(qObj.currFeatures.rows);
            std::iota(allFeatureIndices.begin(), allFeatureIndices.end(), 0);

            // Compute camera projection matrix
            cv::Matx34d prevCamProjMat, currCamProjMat;
            cv::hconcat(intrinsics * qObj.currExtrinsics_R, intrinsics * qObj.currExtrinsics_t, currCamProjMat);
            const cv::Vec3d absPose_t = -qObj.currExtrinsics_R.t() * qObj.currExtrinsics_t;

            std::shared_ptr<View> currView = kfSet.findView(qObj.currKeyFrameId);
            cv::Matx33d prevExtrinsics_R;
            cv::Vec3d prevPose_t, prevExtrinsics_t;
            for (std::vector<int>::size_type i = 0; i != refinedViewIds.size(); ++i) {
                const int prevViewId = refinedViewIds[i]; 
                std::shared_ptr<View> prevView = kfSet.findView(prevViewId);

                // Alway fix the very first view
                if (prevViewId == 0) {
                    fixedViewIds.push_back(prevViewId);
                }
                else {
                    // Distance-2 key frames are fixed during bundle adjustment. No reconstruction is performed.
                    if (distances[i] == 2) {
                        if (static_cast<int>(fixedViewIds.size()) < config.baseParams.maxNumFixedViewsBA) // Do not add too many fixed key frames
                        {
                            fixedViewIds.push_back(prevViewId);
                            actualRefinedViewIds.push_back(prevViewId);
                        }   
                        continue;
                    }
                }
                actualRefinedViewIds.push_back(prevViewId);

                // Reconstruct new map points with distance-1 key frames
                prevExtrinsics_R = prevView->getOrientation().t();
                prevPose_t = prevView->getLocation();
                prevExtrinsics_t = -prevExtrinsics_R * prevPose_t;

                std::vector<int> prevTrackedFeatureIndices;
                prevTrackedFeatureIndices.reserve(config.baseParams.numFeatures);

                const std::unordered_map<int, int> prevWorldPoints = prevView->getWorldPointsInView();

                std::vector<double> depthOfPoint3d;
                depthOfPoint3d.reserve(prevWorldPoints.size());

                for (const auto& wp : prevWorldPoints) {
                    depthOfPoint3d.push_back(cv::norm(mpSet.getWorldPoint(wp.first)->getLocation() - prevPose_t));
                    prevTrackedFeatureIndices.push_back(wp.second);
                }

                // Find median depth
                std::nth_element(depthOfPoint3d.begin(), depthOfPoint3d.begin() + depthOfPoint3d.size() / 2, depthOfPoint3d.end());
                const double medianDepth = depthOfPoint3d[depthOfPoint3d.size() / 2];

                // Skip the key frame if the change of view is small
                const bool isViewTooClose = cv::norm(prevPose_t - absPose_t) < config.baseParams.minSinParallax * medianDepth;
                if (!isViewTooClose) {
                    // Find the unmatched features in the distance-1 key frame
                    std::vector<int> prevUnmatchedFeatureIndices(prevView->getFeatureDescriptors().rows - prevTrackedFeatureIndices.size());
                    std::vector<int> prevAllFeatureIndices(prevView->getFeatureDescriptors().rows);
                    std::iota(prevAllFeatureIndices.begin(), prevAllFeatureIndices.end(), 0);
                    std::sort(prevTrackedFeatureIndices.begin(), prevTrackedFeatureIndices.end());
                    auto iter = std::set_difference(prevAllFeatureIndices.begin(), prevAllFeatureIndices.end(),
                        prevTrackedFeatureIndices.begin(), prevTrackedFeatureIndices.end(), prevUnmatchedFeatureIndices.begin());
                    cv::Mat prevUnmatchedFeatures(prevUnmatchedFeatureIndices.size(), 32, CV_8U);

                    std::vector<cv::KeyPoint> prevUnmatchedPoints;
                    prevUnmatchedPoints.reserve(prevUnmatchedFeatureIndices.size());
                    for (int i = 0; i != static_cast<int>(prevUnmatchedFeatureIndices.size()); ++i) {
                        prevView->getFeatureDescriptors(prevUnmatchedFeatureIndices[i]).copyTo(prevUnmatchedFeatures.row(i));
                        prevUnmatchedPoints.push_back(prevView->getFeaturePoints(prevUnmatchedFeatureIndices[i]));
                    }

                    std::vector<int> currUnmatchedFeatureIndices;
                    currUnmatchedFeatureIndices.reserve(stereoCorrespondence.size());

                    for (const auto& pair : stereoCorrespondence) {
                        currUnmatchedFeatureIndices.push_back(pair.first);
                    }

                    cv::Mat currUnmatchedFeatures(currUnmatchedFeatureIndices.size(), 32, CV_8U);
                    std::vector<cv::KeyPoint> currUnmatchedPoints;
                    currUnmatchedPoints.reserve(currUnmatchedFeatureIndices.size());
                    for (int i = 0; i != static_cast<int>(currUnmatchedFeatureIndices.size()); ++i) {
                        qObj.currFeatures.row(currUnmatchedFeatureIndices[i]).copyTo(currUnmatchedFeatures.row(i));
                        currUnmatchedPoints.push_back(qObj.currPoints[currUnmatchedFeatureIndices[i]]);
                    }

                    // Match features
                    constexpr int maxOctaveDiff{ 2 };
                    std::vector<std::pair<int, int>> matchedPairs = matchFeatures(
                        prevUnmatchedFeatures, currUnmatchedFeatures, prevUnmatchedPoints, currUnmatchedPoints,
                        config.baseParams.matchThreshold, config.baseParams.maxRatioMapping, maxOctaveDiff);

                    if (static_cast<int>(matchedPairs.size()) < config.baseParams.minNumMatchesMapping)
                        continue;

                    auto conn = kfSet.findConnection(prevViewId, qObj.currKeyFrameId);

                    // Check if the matched feature points have known 3-D location computed from disparity/depth map
                    for (const auto pair : matchedPairs) {

                        // Add 2-D to 3-D correspondence between the previous key frame and 
                        // the 3-D point computed from disparity/depth map
                        const int featureIdxPrev = prevUnmatchedFeatureIndices[pair.first],
                                  featureIdxCurr = currUnmatchedFeatureIndices[pair.second],
                                  point3dIdx     = stereoCorrespondence.at(featureIdxCurr);
                        const cv::Vec3d point3d  = mpSet.getLocation(point3dIdx);

                        // Compute reprojection error in the previous key frame
                        const double errorsInPrevCamera = computeReprojectionErrors(
                            point3d, prevUnmatchedPoints[pair.first].pt, prevExtrinsics_R, prevExtrinsics_t, intrinsics);
                 
                        if (errorsInPrevCamera <= config.baseParams.maxReprojError * std::pow(config.baseParams.scaleFactor, prevUnmatchedPoints[pair.first].octave)) {

                            // Add correspondence
                            mpSet.addCorrespondence(point3dIdx, prevViewId, featureIdxPrev, kfSet);

                            // Update attributes
                            mpSet.updateLimitsAndDirection(point3dIdx, kfSet);

                            // Update representative view
                            mpSet.updateRepresentativeView(point3dIdx, kfSet);

                            // Update connections with new feature matches
                            conn->addMatches(std::make_pair(featureIdxPrev, featureIdxCurr));
                            prevView->updateConnectedView(qObj.currKeyFrameId);
                            currView->updateConnectedView(prevViewId);

                            // Update the correspondence 
                            stereoCorrespondence.erase(featureIdxCurr);
                       
                            // Update tracked feature indices
                            qObj.trackedFeatureIndices.push_back(featureIdxCurr);
                        }
                    }
                }
            }

            // For local bundle adjustment
            actualRefinedViewIds.push_back(qObj.currKeyFrameId);
            refinedViewIds = std::move(actualRefinedViewIds);
            std::sort(refinedViewIds.begin(), refinedViewIds.end());
            if (fixedViewIds.empty()) {
                // If there is not a distance-2 view, set the oldest 2 views as fixed views
                std::sort(refinedViewIds.begin(), refinedViewIds.end());
                fixedViewIds.push_back(refinedViewIds[0]);
                if (static_cast<int>(refinedViewIds.size()) > 2) {
                    fixedViewIds.push_back(refinedViewIds[1]); // Do not fix all the views
                }
            }
        }
    }
}
