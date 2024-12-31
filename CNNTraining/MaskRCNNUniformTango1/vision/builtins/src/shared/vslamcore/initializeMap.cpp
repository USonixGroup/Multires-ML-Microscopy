////////////////////////////////////////////////////////////////////////////////
// Perform map initialization using homography or fundamental matrix
// 
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/View.hpp"
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/WorldPoint.hpp"
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
    #include "vslamcore/initializeMap.hpp"
#else
    #include "View.hpp"
    #include "Connection.hpp"
    #include "WorldPoint.hpp"
    #include "MapPointSet.hpp"
    #include "KeyFrameSet.hpp"
    #include "initializeMap.hpp"
#endif

namespace vision {
    namespace vslam {
        
        /**
        * @brief Set trackedFeatureIndices with values from 0 to size-1
        *
        * @param[in/out] trackedFeatureIndices, specified as a vector of integers.
        * @param[in] size, The number of tracked feature indices, specified as an integer.
        */
        void setDefaultTrackedFeatures(std::vector<int>& trackedFeatureIndices, const int size) {
            trackedFeatureIndices.resize(size);
            std::iota(std::begin(trackedFeatureIndices), std::end(trackedFeatureIndices), 0);
        }

        bool initializeMapMono(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            QueueInstanceBase& qObj,
            const cv::Matx33d& intrinsics,
            const ConfigurationMono& config,
            NodeIDGenerator& generator) {

            // Check if enough feature points were extracted
            if (qObj.currFeatures.rows < config.baseParams.numFeatures) {
                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::ExtractedPointsNotEnough",
                                                {std::to_string(qObj.currFrameIndex + 1), 
                                                 std::to_string(qObj.currFeatures.rows), std::to_string(config.baseParams.numFeatures)});
                return false;
            }

            if (!kfSet.getNumViews()) { // no key frame has been added
                setDefaultTrackedFeatures(qObj.trackedFeatureIndices, qObj.currFeatures.rows);

                qObj.currKeyFrameId = config.imuParams.hasIMU ? generator.newIDs(SensorConfig::CAMERA_IMU_SIM3)[0].identifier
                                                              : generator.newIDs(NodeType::POSE_SE3, 2)[0].identifier;

                // Add key frame and update LoopClosureDatabase
                auto view(std::make_shared<View>(qObj.currKeyFrameId, std::move(qObj.currPoints), qObj.currFeatures, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0), qObj.currFrameIndex));
                kfSet.addView(view);

                // Update LoopClosureDatabase
                // Convert features into feature vector
                std::vector<cv::Mat> currFeaturesVec;
                database.changeFeaturesStructure(qObj.currFeatures, currFeaturesVec);

                // Add features to database
                database.addImageFeatures(qObj.currKeyFrameId, std::move(currFeaturesVec));

                return false;
            }

            // match features
            auto preView = kfSet.findView().front(); // Only one view is available
            cv::Mat preFeatures = preView->getFeatureDescriptors();
            std::vector<cv::KeyPoint> prePoints = preView->getFeaturePoints();

            constexpr int maxOctaveDiff{ 0 };
            std::vector<std::pair<int, int>> matchedPairs = matchFeatures(
                preFeatures, qObj.currFeatures, prePoints, qObj.currPoints, config.baseParams.matchThreshold, config.baseParams.maxRatio, maxOctaveDiff);

            const int numMatches = matchedPairs.size();
            if (numMatches < config.baseParams.minNumWorldPoints) {
                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::MatchedPointsNotEnough",
                                               {std::to_string(qObj.currFrameIndex + 1), std::to_string(numMatches), 
                                                std::to_string(config.baseParams.minNumWorldPoints)});
                return false;
            }

            std::vector<cv::Point2f> imagePoints1(numMatches), imagePoints2(numMatches);
            for (int i = 0; i != numMatches; ++i) {
                imagePoints1[i] = prePoints[matchedPairs[i].first].pt;
                imagePoints2[i] = qObj.currPoints[matchedPairs[i].second].pt;
            }

            std::vector<uchar> inlierIdxH(numMatches), inlierIdxF(numMatches);
            constexpr double prob{ 0.99 }, thresholdH{ 5.99 }, thresholdF{ 3.84 };

            cv::Matx33d tformF = cv::findFundamentalMat(imagePoints1, imagePoints2, cv::FM_RANSAC, std::sqrt(thresholdF), prob, inlierIdxF);
            cv::Matx33d tformH = cv::findHomography(imagePoints1, imagePoints2, cv::RANSAC, std::sqrt(thresholdH), inlierIdxH);

            // compute symmetric transfer error
            double scoreH{ 0 }, scoreF{ 0 };
            
            std::vector<cv::Vec3f> epLines1(numMatches), epLines2(numMatches);

            // error of fundamental matrix
            cv::computeCorrespondEpilines(imagePoints1, 1, tformF, epLines1); // find epipolar lines in image2
            cv::computeCorrespondEpilines(imagePoints2, 2, tformF, epLines2); // find epipolar lines in image1

            for (std::vector<cv::Vec3d>::size_type i = 0; i != imagePoints1.size(); ++i) {
                if (inlierIdxF[i]) // if the point in image 1 is a inlier
                {
                    double distToLine1 = distancePointLine(imagePoints1[i], epLines2[i]);
                    double distToLine2 = distancePointLine(imagePoints2[i], epLines1[i]);

                    scoreF += (std::max(thresholdF - distToLine1 * distToLine1, 0.0) +
                        std::max(thresholdF - distToLine2 * distToLine2, 0.0));
                }
            }

            // error of homography
            std::vector<cv::Point2f> projPtIn2(numMatches), projPtIn1(numMatches);
            cv::perspectiveTransform(imagePoints1, projPtIn2, tformH);
            cv::perspectiveTransform(imagePoints2, projPtIn1, tformH.inv());
            for (std::vector<cv::Vec3d>::size_type i = 0; i != imagePoints1.size(); ++i) {
                if (inlierIdxH[i]) // if the point in image 1 is an inlier
                {
                    scoreH += (std::max(thresholdH - std::pow(cv::norm(projPtIn2[i] - imagePoints2[i]), 2), 0.0) +
                        std::max(thresholdH - std::pow(cv::norm(projPtIn1[i] - imagePoints1[i]), 2), 0.0));
                }
            }

            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::ReprojectionErrorF", {std::to_string(scoreF)});
            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::ReprojectionErrorH", {std::to_string(scoreH)});

            // select the model based on a heuristic
            const double ratio = scoreH / (scoreH + scoreF);
            constexpr double ratioThreshold{ 0.45 };

            cv::Matx33d relPose_R;
            cv::Vec3d relPose_t;

            std::vector<std::pair<int, int>> inlierMatchedPairs;
            std::vector<cv::Vec3d> xyzPoints;
            xyzPoints.reserve(numMatches);
            inlierMatchedPairs.reserve(numMatches);
            double medianDepth{ 1 };

            std::vector<cv::Point2f> matchedImagePoints1, matchedImagePoints2;
            bool isMapInitialized;
            if (ratio > ratioThreshold) {

                for (std::vector<uchar>::size_type i = 0; i != inlierIdxH.size(); ++i) {
                    if (inlierIdxH[i]) {
                        matchedImagePoints1.push_back(imagePoints1[i]);
                        matchedImagePoints2.push_back(imagePoints2[i]);
                        inlierMatchedPairs.push_back(matchedPairs[i]);
                    }
                }

                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::UsingHMatrix");

                reconstructFromHomography(tformH, matchedImagePoints1, matchedImagePoints2, intrinsics, inlierMatchedPairs,
                    xyzPoints, isMapInitialized, medianDepth, relPose_R, relPose_t, config);
            }
            else {

                for (std::vector<uchar>::size_type i = 0; i != inlierIdxF.size(); ++i) {
                    if (inlierIdxF[i]) {
                        matchedImagePoints1.push_back(imagePoints1[i]);
                        matchedImagePoints2.push_back(imagePoints2[i]);
                        inlierMatchedPairs.push_back(matchedPairs[i]);
                    }
                }

                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::UsingFMatrix");

                reconstructFromFundamentalMatrix(tformF, matchedImagePoints1, matchedImagePoints2, intrinsics, inlierMatchedPairs,
                    xyzPoints, isMapInitialized, medianDepth, relPose_R, relPose_t, config);
            }

            if (isMapInitialized) {
                qObj.currKeyFrameId = config.imuParams.hasIMU ? generator.newIDs(SensorConfig::CAMERA_IMU_SIM3)[0].identifier
                                                              : generator.newIDs(NodeType::POSE_SE3, 2)[0].identifier;

                // add views and connections
                auto currView(std::make_shared<View>(qObj.currKeyFrameId, std::move(qObj.currPoints), qObj.currFeatures, relPose_R, relPose_t / medianDepth, qObj.currFrameIndex));
                auto conn(std::make_shared<Connection>(preView->getViewId(), qObj.currKeyFrameId, relPose_R, relPose_t / medianDepth, inlierMatchedPairs));

                kfSet.addView(currView);
                kfSet.addConnection(conn);

                // add map points
                for (std::vector<cv::Point3f>::size_type i = 0; i != xyzPoints.size(); ++i) {
                    const int pointId = generator.newIDs(NodeType::POINT_XYZ).identifier;
                    auto currPoint(std::make_shared<WorldPoint>(pointId, xyzPoints[i] / medianDepth));
                    mpSet.addWorldPoint(currPoint);
                    mpSet.addCorrespondence(pointId, preView->getViewId(), inlierMatchedPairs[i].first, kfSet);
                    mpSet.addCorrespondence(pointId, currView->getViewId(), inlierMatchedPairs[i].second, kfSet);

                    mpSet.updateRepresentativeView(pointId, kfSet);

                    qObj.trackedMapPointIds.push_back(pointId);
                    qObj.trackedFeatureIndices.push_back(inlierMatchedPairs[i].second);
                }

                // bundle adjustment of two views
                std::vector<int> fixedViewIds = { preView->getViewId() }, viewIds = { preView->getViewId(), qObj.currKeyFrameId };
                bundleAdjustment(mpSet, kfSet, intrinsics, viewIds, fixedViewIds, config);

                // Update LoopClosureDatabase
                // Convert features into feature vector
                std::vector<cv::Mat> currFeaturesVec;
                database.changeFeaturesStructure(qObj.currFeatures, currFeaturesVec);

                // Add features to database
                database.addImageFeatures(qObj.currKeyFrameId, std::move(currFeaturesVec));
            } else{
                setDefaultTrackedFeatures(qObj.trackedFeatureIndices, qObj.currFeatures.rows);
            }

            return isMapInitialized;
        }

        bool initializeMapStereo(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            const StereoReconstructor& reconstructor,
            QueueInstanceStereo& qObj,
            const cv::Matx33d& intrinsics,
            const ConfigurationStereo& config,
            NodeIDGenerator& generator) {

            // Check if enough feature points were extracted
            if (qObj.currFeatures.rows < config.baseParams.numFeatures) {
                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::ExtractedPointsNotEnough",
                                                    {std::to_string(qObj.currFrameIndex + 1), 
                                                     std::to_string(qObj.currFeatures.rows), std::to_string(config.baseParams.numFeatures)});
                setDefaultTrackedFeatures(qObj.trackedFeatureIndices, qObj.currFeatures.rows);
                return false;
            }

            constexpr int maxOctaveDiff{ 0 };
            constexpr float maxRatioStereo{ 1.f };
            std::vector<std::pair<int, int>> matchedPairs = matchFeatures(
                qObj.currFeatures, qObj.featuresR, qObj.currPoints, qObj.pointsR, config.baseParams.matchThreshold, maxRatioStereo, maxOctaveDiff);

            std::vector<cv::Vec3d> xyzPointsL;
            reconstructor.reconstruct(qObj, matchedPairs, xyzPointsL, config);

            int numMatches = static_cast<int>(matchedPairs.size());
            if (numMatches < config.baseParams.minNumWorldPoints) {
                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::MatchedPointsNotEnough",
                                               {std::to_string(qObj.currFrameIndex + 1), std::to_string(numMatches), 
                                                std::to_string(config.baseParams.minNumWorldPoints)});
                setDefaultTrackedFeatures(qObj.trackedFeatureIndices, qObj.currFeatures.rows);
                return false;
            }

            qObj.currKeyFrameId = generator.newIDs(NodeType::POSE_SE3).identifier;
            auto view(std::make_shared<View>(qObj.currKeyFrameId, std::move(qObj.currPoints), qObj.currFeatures, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0), qObj.currFrameIndex));
            kfSet.addView(view);

            // add map points
            for (std::vector<cv::Point3d>::size_type i = 0; i != xyzPointsL.size(); ++i) {
                const int pointId = generator.newIDs(NodeType::POINT_XYZ).identifier;
                
                // initializeMap assumes initial pose is zero so xyzPointsL need not be transformed
                mpSet.addWorldPoint(std::make_shared<WorldPoint>(pointId, xyzPointsL[i]));
                mpSet.addCorrespondence(pointId, qObj.currKeyFrameId, matchedPairs[i].first, kfSet);

                // update additional attributes
                mpSet.updateLimitsAndDirection(pointId, kfSet);
                mpSet.updateRepresentativeView(pointId, kfSet);

                qObj.trackedMapPointIds.push_back(pointId);
                qObj.trackedFeatureIndices.push_back(matchedPairs[i].first);
            }

            // Update LoopClosureDatabase
            // Convert features into feature vector
            std::vector<cv::Mat> featuresVec;
            database.changeFeaturesStructure(qObj.currFeatures, featuresVec);

            // Add features to database
            database.addImageFeatures(qObj.currKeyFrameId, std::move(featuresVec));

            return true;
        }

        bool initializeMapRGBD(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            LoopClosureDatabase& database,
            QueueInstanceRGBD& qObj,
            const cv::Matx33d& intrinsics,
            const Configuration& config,
            NodeIDGenerator& generator) {

            // Check if enough feature points were extracted
            if (qObj.currFeatures.rows < config.baseParams.numFeatures ||
                static_cast<int>(qObj.xyzPoints.size()) < config.baseParams.minNumWorldPoints) {
                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::ExtractedPointsNotEnough",
                                                    {std::to_string(qObj.currFrameIndex + 1), 
                                                     std::to_string(qObj.currFeatures.rows), std::to_string(config.baseParams.numFeatures)});
                setDefaultTrackedFeatures(qObj.trackedFeatureIndices, qObj.currFeatures.rows);
                return false;
            }

            const int keyFrameId = generator.newIDs(NodeType::POSE_SE3).identifier;
            auto view(std::make_shared<View>(keyFrameId, std::move(qObj.currPoints), qObj.currFeatures, cv::Matx33d::eye(), cv::Vec3d(0, 0, 0), qObj.currFrameIndex));
            kfSet.addView(view);

            // add map points
            for (std::vector<cv::Point3d>::size_type i = 0; i != qObj.xyzPoints.size(); ++i) {
                const int pointId = generator.newIDs(NodeType::POINT_XYZ).identifier;
                
                // initializeMap assumes initial pose is zero so xyzPointsL need not be transformed
                mpSet.addWorldPoint(std::make_shared<WorldPoint>(pointId, qObj.xyzPoints[i]));
                mpSet.addCorrespondence(pointId, keyFrameId, qObj.validIndex[i], kfSet);

                // update additional attributes
                mpSet.updateLimitsAndDirection(pointId, kfSet);
                mpSet.updateRepresentativeView(pointId, kfSet);

                qObj.trackedMapPointIds.push_back(pointId);
                qObj.trackedFeatureIndices.push_back(qObj.validIndex[i]);
            }

            // Update LoopClosureDatabase
            // Convert features into feature vector
            std::vector<cv::Mat> currFeaturesVec;
            database.changeFeaturesStructure(qObj.currFeatures, currFeaturesVec);

            // Add features to database
            database.addImageFeatures(keyFrameId, std::move(currFeaturesVec));

            return true;
        }
    }
}