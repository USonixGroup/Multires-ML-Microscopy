//////////////////////////////////////////////////////////////////////////////////
// Utility class for loop closure detection and corresponding map correction.
// 
// Copyright 2024 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////////

#include <unordered_set>
#include <string>
#include <memory>
#include <unordered_map>
#include <map>
#include <cmath>
#include <vector>

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/InvertedImageIndex.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/matchFeatures.hpp"
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/LoopClosureDatabase.hpp"
    #include "vslamcore/converter.hpp"
#else
    #include "InvertedImageIndex.hpp"
    #include "KeyFrameSet.hpp"
    #include "MapPointSet.hpp"
    #include "matchFeatures.hpp"
    #include "Connection.hpp"
    #include "LoopClosureDatabase.hpp"
    #include "converter.hpp"
#endif

namespace vision {
    namespace vslam {

        bool LoopClosureDatabase::checkLoopClosure(KeyFrameSet& kfSet,
            const int currKeyFrameId,
            const std::vector<cv::Mat>& currFeatures,
            const Configuration& config,
            std::vector<std::vector<int>>& loopKeyFrameIds) {

            auto currKeyFrame(kfSet.findView(currKeyFrameId));

            // The following buffer stores keyframe IDs to a pair of score and visited status.
            // Visited status is used to check if a keyframe is already considered for a loop.
            std::map<int, std::pair<double, bool>> buffer;

            constexpr int maxResult(10);

            // Retrieve images with scores
            auto queryResult(this->database.retrieveImages(currFeatures, maxResult));

            // Get min score from strongly connected keyframes
            double minScore(1.0);
            bool foundConnectedView(false);
            for (const auto& item : queryResult) {
                // Find keyframe id
                auto viewEntry(this->bowIdToViewId.find(item.Id));
                if (viewEntry != this->bowIdToViewId.end()) {

                    // Collect min score if keyframe is connected
                    int keyframeId(viewEntry->second);
                    if (currKeyFrame->isConnectedView(keyframeId, config.baseParams.minNumMatchesLoop)) {
                        foundConnectedView = true;
                        if (minScore > item.Score) {
                            minScore = item.Score;
                        }
                        
                        config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::loopCandidateConnectedIdScore",
                                                    {std::to_string(kfSet.getFrameIndexByViewId(keyframeId) + 1), 
                                                     std::to_string(currKeyFrame->getFrameIndex() + 1),  // 1-based indexing
                                                     std::to_string(item.Score)});
                    }
                    else if (!currKeyFrame->isConnectedView(keyframeId)) { // Only storing non-connected frames
                       // false signifies unvisited
                        buffer[item.Id] = std::make_pair(item.Score, false);

                        config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::loopCandidateIdScore",
                                                   {std::to_string(kfSet.getFrameIndexByViewId(keyframeId) + 1), 
                                                    std::to_string(currKeyFrame->getFrameIndex() + 1),  // 1-based indexing
                                                    std::to_string(item.Score)}); 
                    }
                }
            }

            loopKeyFrameIds.clear();

            if (!foundConnectedView) {
                // No connected views
                return false;
            }

            // Filter candidate frames and organize frames into group of atleast 3
            // Use BoWID to check if the candidate frames are consecutive as KeyFrame IDs are not sequential
            double thresholdScore(0.75 * minScore);
            for (auto iter = buffer.begin(); iter != buffer.end(); iter++) {
                int bowId(iter->first);
                double score(iter->second.first);
                bool isVisited(iter->second.second);

                std::vector<int> consecutiveList;
                if (score > thresholdScore && !isVisited) {
                    buffer[bowId] = std::make_pair(score, true);
                    consecutiveList.push_back(bowIdToViewId[bowId]);

                    // Explore in ascending order
                    int ascBoWId(bowId);
                    while (1) {
                        ascBoWId++;
                        auto ascItem(buffer.find(ascBoWId));
                        // Check the following conditions:
                        //  - If next view exists
                        //  - If it is not visited
                        //  - If score is above threshold
                        if (ascItem != buffer.end()) {
                            auto ascViewScore(ascItem->second.first);
                            auto isAscViewVisited(ascItem->second.second);
                            if (ascViewScore > thresholdScore &&
                                !isAscViewVisited) {

                                consecutiveList.push_back(bowIdToViewId[ascBoWId]);
                                // Mark it visited
                                buffer[ascBoWId] = std::make_pair(ascViewScore, true);
                            }
                            else {
                                break;
                            }
                        }
                        else {
                            break;
                        }
                    }
                }

                // Add the valid loop keyframe IDs
                if (consecutiveList.size() >= 3) {
                    loopKeyFrameIds.push_back(consecutiveList);
                }
            }

            return !loopKeyFrameIds.empty();
        }

        bool LoopClosureDatabase::addLoopConnections(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const Matx33d& intrinsics,
            const int currKeyFrameId,
            const std::vector<std::vector<int>>& loopKeyFrameIds,
            const Configuration& config,
            std::vector<std::pair<std::shared_ptr<Connection>, double>>& loopConnections) {

            bool retval(false);
            constexpr int maxOctaveDiff(8);
            auto currKeyFrame(kfSet.findView(currKeyFrameId));
            auto poseCurrKF(currKeyFrame->getPose());
            loopConnections.clear();

            // Extract world points and their features for current frame
            auto wpointsFeatureIndexPairCurrKF(currKeyFrame->getWorldPointsInView());
        
            std::vector<int> worldPointIndicesCurrKF;
            worldPointIndicesCurrKF.reserve(wpointsFeatureIndexPairCurrKF.size());
            std::vector<int> featuresIndicesCurrKF;
            featuresIndicesCurrKF.reserve(wpointsFeatureIndexPairCurrKF.size());

            // TODO future - See how we can optimize this loop
            for (const auto& kv : wpointsFeatureIndexPairCurrKF) {
                worldPointIndicesCurrKF.push_back(kv.first);
                featuresIndicesCurrKF.push_back(kv.second);
            }

            auto validFeaturesCurrKF(currKeyFrame->getFeatureDescriptors(featuresIndicesCurrKF));
            auto validKeypointsCurrKF(currKeyFrame->getFeaturePoints(featuresIndicesCurrKF));
            // Store world points to be set invalid
            std::unordered_set<int> invalidPoints;

            for (const auto& consecutiveKeyFrameIds : loopKeyFrameIds) {
                for (const auto& loopKeyFrameId : consecutiveKeyFrameIds) {
                    auto loopKeyFrame(kfSet.findView(loopKeyFrameId));
                    auto posePrevKF(loopKeyFrame->getPose());

                    // Extract world points and their features for loop candidate
                    auto wpointsFeatureIndexPairPrevKF(kfSet.findWorldPointsInView(loopKeyFrameId));

                    std::vector<int> worldPointIndicesPrevKF;
                    worldPointIndicesPrevKF.reserve(wpointsFeatureIndexPairPrevKF.size());
                    std::vector<int> featuresIndicesPrevKF;
                    featuresIndicesPrevKF.reserve(wpointsFeatureIndexPairPrevKF.size());

                    for (const auto& kv : wpointsFeatureIndexPairPrevKF) {
                        worldPointIndicesPrevKF.push_back(kv.first);
                        featuresIndicesPrevKF.push_back(kv.second);
                    }

                    auto validFeaturesPrevKF(loopKeyFrame->getFeatureDescriptors(featuresIndicesPrevKF));
                    auto validKeypointsPrevKF(loopKeyFrame->getFeaturePoints(featuresIndicesPrevKF));

                    // Match current frame with previous frame
                    // Order matters i.e. we want to match current frame with loop keyframe
                    auto indexPairs(matchFeatures(validFeaturesCurrKF, validFeaturesPrevKF,
                        validKeypointsCurrKF, validKeypointsPrevKF,
                        config.baseParams.matchThreshold, config.baseParams.maxRatio, maxOctaveDiff));

                    // Check if the candidate keyframe has a strong connection with the current keyframe
                    auto numPoints(static_cast<int>(indexPairs.size()));
                    if (numPoints < config.baseParams.minNumMatchesLoop) {
                        continue;
                    }

                    auto oriCurrKF(currKeyFrame->getOrientation());
                    auto locCurrKF(currKeyFrame->getLocation());
                    auto oriPrevKF(loopKeyFrame->getOrientation());
                    auto locPrevKF(loopKeyFrame->getLocation());

                    std::vector<cv::KeyPoint> imagePointsCurrKF, imagePointsPrevKF;
                    imagePointsCurrKF.reserve(numPoints);
                    imagePointsPrevKF.reserve(numPoints);

                    cv::Mat worldPointsCurrKFInCameraCurrKF(numPoints, 3, CV_64F), worldPointsPrevKFInCameraPrevKF(numPoints, 3, CV_64F);
                    for (int c = 0; c < numPoints; c++) {
                        auto wpPrevKF(mpSet.getLocation(worldPointIndicesPrevKF[indexPairs[c].second]));
                        auto wpCurrKF(mpSet.getLocation(worldPointIndicesCurrKF[indexPairs[c].first]));

                        cv::Vec3d wpCF(oriCurrKF.t() * wpCurrKF - oriCurrKF.t() * locCurrKF);
                        worldPointsCurrKFInCameraCurrKF.at<double>(c, 0) = wpCF[0];
                        worldPointsCurrKFInCameraCurrKF.at<double>(c, 1) = wpCF[1];
                        worldPointsCurrKFInCameraCurrKF.at<double>(c, 2) = wpCF[2];

                        cv::Vec3d wpPF(oriPrevKF.t() * wpPrevKF - oriPrevKF.t() * locPrevKF);
                        worldPointsPrevKFInCameraPrevKF.at<double>(c, 0) = wpPF[0];
                        worldPointsPrevKFInCameraPrevKF.at<double>(c, 1) = wpPF[1];
                        worldPointsPrevKFInCameraPrevKF.at<double>(c, 2) = wpPF[2];

                        imagePointsCurrKF.push_back(validKeypointsCurrKF[indexPairs[c].first]);
                        imagePointsPrevKF.push_back(validKeypointsPrevKF[indexPairs[c].second]);
                    }

                    // Estimate geometric transform 3D
                    cv::Mat inliers;
                    cv::Matx44d hForm;
                    
                    try {
                        if (config.isMono) {
                            estimateSimilarity3D(worldPointsCurrKFInCameraCurrKF, worldPointsPrevKFInCameraPrevKF,
                                imagePointsCurrKF, imagePointsPrevKF, intrinsics, hForm, inliers, config.baseParams.maxReprojError * 2);
                        }
                        else{
                            estimateRigid3D(worldPointsCurrKFInCameraCurrKF, worldPointsPrevKFInCameraPrevKF,
                                imagePointsCurrKF, imagePointsPrevKF, intrinsics, hForm, inliers, config.baseParams.maxReprojError * 2);
                        }
                    }
                    catch (...) {
                        continue;
                    }

                    // Extract rotation matrix and translation vector
                    cv::Matx33d rot;
                    cv::Vec3d trans;
                    pose2rt(hForm, rot, trans);

                    // Converting similarity transform to rigid and extracting scale
                    double scale(1.0);
                    if (config.isMono) {
                        scale = std::cbrt(cv::determinant(rot));
                        if (scale < 1e-3) {
                            continue; // Scale cannot be zero
                        }
                        rot = rot / scale;
                    }

                    // Filter inliers
                    std::vector<std::pair<int, int>> matchesCurrKF;
                    int numInliers(static_cast<int>(cv::sum(inliers)[0]));
                    matchesCurrKF.reserve(numInliers);

                    for (int k = 0; k < numPoints; k++) {
                        // If the below condition is true then it means it is an inlier
                        if (static_cast<int>(inliers.at<uchar>(k, 0)) == 1) {

                            // Fuse co-visible map points
                            int pointIdPrevKF(worldPointIndicesPrevKF[indexPairs[k].second]);
                            int pointIdCurrKF(worldPointIndicesCurrKF[indexPairs[k].first]);

                            bool needsFuse = pointIdCurrKF != pointIdPrevKF && // Only fuse when the map point indices are different and unqiue
                                std::find(worldPointIndicesCurrKF.begin(), worldPointIndicesCurrKF.end(), pointIdPrevKF) == worldPointIndicesCurrKF.end();

                            if (needsFuse) {

                                worldPointIndicesCurrKF[indexPairs[k].first] = pointIdPrevKF;

                                // Store worldPoint of the current frame so that it can be removed from MapPointSet
                                int featureIdxCurrKF(currKeyFrame->getCorrespondence(pointIdCurrKF));
                                
                                mpSet.addCorrespondence(pointIdPrevKF, currKeyFrameId, featureIdxCurrKF, kfSet);
                                
                                invalidPoints.insert(pointIdCurrKF);
                            }

                            // Always update matches
                            matchesCurrKF.push_back(std::make_pair(featuresIndicesPrevKF[indexPairs[k].second], featuresIndicesCurrKF[indexPairs[k].first]));
                        }
                    }

                    config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NewConnectionWithPoints",
                                                    {std::to_string(currKeyFrame->getFrameIndex() + 1), std::to_string(loopKeyFrame->getFrameIndex() + 1), 
                                                     std::to_string(static_cast<int>(matchesCurrKF.size()))});
                    
                    // Add connection between the loop key frame and the current key frame
                    auto connCurr(std::make_shared<Connection>(loopKeyFrameId, currKeyFrameId, rot, trans, matchesCurrKF));
                    kfSet.addConnection(connCurr);
                    loopConnections.push_back(std::make_pair(connCurr, scale));

                }
                retval = true;
            }

            // Set invalid point since they are now replaced by points from previous frames (loop keyframes)
            for (const auto& pointId : invalidPoints) {
                mpSet.setInvalid(pointId, kfSet);
            }
            return retval;
        }

        unsigned int LoopClosureDatabase::addImageFeatures(const int currKeyFrameId,
            const std::vector<cv::Mat>& imageFeatures) {
            auto internalId(this->database.addImageFeatures(imageFeatures));
            this->bowIdToViewId[internalId] = currKeyFrameId;
            return internalId;
        }

        void LoopClosureDatabase::changeFeaturesStructure(const cv::Mat& plainFeatures, std::vector<cv::Mat>& outFeatures) {
            outFeatures.resize(plainFeatures.rows);
            for (int i = 0; i < plainFeatures.rows; ++i)
            {
                outFeatures[i] = plainFeatures.row(i);
            }
        }
    }// namespace vslam
}// namespace vision