////////////////////////////////////////////////////////////////////////////////
//  MonoVisualSLAMImpl.hpp
//
//  This is the implementation of mono visual SLAM.
//
//  Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/MonoVisualSLAMImpl.hpp"
    #include "vslamcore/initializeMap.hpp"
    #include "vslamcore/createNewMapPoints.hpp"
    #include "vslamcore/detectAndExtractFeatures.hpp"
    #include "vslamcore/GSTools.hpp"
#else
    #include "MonoVisualSLAMImpl.hpp"
    #include "initializeMap.hpp"
    #include "createNewMapPoints.hpp"
    #include "detectAndExtractFeatures.hpp"
    #include "GSTools.hpp"
#endif

namespace vision {
    namespace vslam {

        MonoVisualSLAMImpl::MonoVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const ConfigurationMono& configuration,
                const char* vocabFile,
                const VSlamConcurrency desiredLevel,
                const cv::Matx44d& camToIMU)
                : BaseVisualSLAMImpl<QueueInstanceBase, ConfigurationMono>(fx, fy, cx, cy, configuration, vocabFile, desiredLevel, camToIMU)
        {
            config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::systemSetup");
        }

        void MonoVisualSLAMImpl::addFrame(const cv::Mat& frame, const cv::Mat& imuG, const cv::Mat& imuA) {

            startThreadsIfIdle();

            QueueInstanceBase qObj;
            qObj.trackedMapPointIds.reserve(config.baseParams.numFeatures);
            qObj.trackedFeatureIndices.reserve(config.baseParams.numFeatures);

            detectAndExtractFeatures(frame, config.baseParams.scaleFactor, config.baseParams.numLevels, config.baseParams.numFeatures, qObj.currPoints, qObj.currFeatures);
            
            if (config.imuParams.hasIMU && !imuG.empty() && !imuA.empty()) {
                pushToIMUBuffers(imuG, imuA, qObj.gyro, qObj.accel);
            }

            addFrameToTrackingQueue(std::move(qObj));
        }

        void MonoVisualSLAMImpl::storeGravityRotationAndScale(const cv::Matx44d& camToLocalIMU, const double metricScale) {

            imuInfo.gravityRotationTransform = camToLocalIMU;
            imuInfo.poseScale = metricScale;
            imuInfo.isGSAvailable = true;

        }

        void MonoVisualSLAMImpl::initializeMap(QueueInstanceBase& qObj) {
            
            // Print "Starting map initialization" if no frame is added yet
            std::vector<int> frameIDs = this->getKeyFrameIndex();
            int szBefore = static_cast<int>(frameIDs.size());
            if(szBefore == 0) {
                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::startMapInitialization");
            }

            // This should be false for the first frame
            const bool keyFrameExistsBeforeInit = !keyFrames->getKeyFrameIndex().empty();

            // Attempt map initialization
            isMapInitialized = initializeMapMono(*mapPoints, *keyFrames, *loopDatabase, qObj, intrinsics, config, generator);

            // This should be true after the first frame is added
            const bool keyFrameExistsAfterInit = !keyFrames->getKeyFrameIndex().empty();
            
            // Call initializationKeyFrameAdded() if map is initialized OR
            // if the first frame is added
            if (isMapInitialized || (!keyFrameExistsBeforeInit && keyFrameExistsAfterInit)) {
                initializationKeyFrameAdded();
            }
            
            // After attempting map initialization with mono, three cases are possible:
            // 1. Map got initialized
            // 2. Initialized failed between first and second frame
            // 3. Initialized failed at first frame
            frameIDs = this->getKeyFrameIndex();
            int szAfter = static_cast<int>(frameIDs.size());
            if (isMapInitialized) {
                if(szAfter > 1) {
                    config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::endMapInitialization",
                                                   {std::to_string(frameIDs[0] + 1),  // Convert to 1-based indexing 
                                                    std::to_string(frameIDs[1] + 1)});
                }
            } else if (szAfter == 1 && szBefore == 1) { // Initialization failed with previous frame
                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::failedMapInitialization", 
                                               {std::to_string(frameIDs[0] + 1),  // Convert to 1-based indexing 
                                                std::to_string(qObj.currFrameIndex + 1)});
            } else if (szAfter == 0) { // Initialization failed at first frame
                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::failedMapInitializationOneFrame", {std::to_string(qObj.currFrameIndex + 1)});
            } 
        }

        void MonoVisualSLAMImpl::addNewMapPoints(QueueInstanceBase& qObj, std::vector<int>& refinedViewIds, std::vector<int>& fixedViewIds) {
            createNewMapPoints(*mapPoints, *keyFrames, qObj, intrinsics, lastMapPointIds, refinedViewIds, fixedViewIds, config, generator);
        }

        void MonoVisualSLAMImpl::cullRecentMapPoints() {
            constexpr int minNumViews{ 3 };
            int numCulledPoints = 0;
            // If two keyframes have passed from map point creation, the map point must be observed by at least three keyframes
            for (const auto& pointId : secondLastMapPointIds) {
                if (mapPoints->getWorldPoint(pointId)->getViewsOfWorldPoint().size() < minNumViews) {
                    mapPoints->setInvalid(pointId, *keyFrames);
                    numCulledPoints++;
                }
            }
            secondLastMapPointIds = lastMapPointIds;
            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NumCulledPoints", { std::to_string(numCulledPoints) });
        }

        int MonoVisualSLAMImpl::generateNewPoseID() {
            if (config.imuParams.hasIMU) {
                std::vector<NodeID> newNodeIDs = generator.newIDs(SensorConfig::CAMERA_IMU_SIM3);
                return newNodeIDs[0].identifier;
            }
            else {
                return generator.newIDs(NodeType::POSE_SE3, 2)[0].identifier;
            }
        }

    } // namespace vslam
} // namespace vision
