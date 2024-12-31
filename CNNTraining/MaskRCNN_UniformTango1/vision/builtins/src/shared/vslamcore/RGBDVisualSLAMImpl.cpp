////////////////////////////////////////////////////////////////////////////////
//  RGBDVisualSLAMImpl.hpp
//
//  This is the implementation of RGB-D visual SLAM.
//
//  Copyright 2022-2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/RGBDVisualSLAMImpl.hpp"
    #include "vslamcore/initializeMap.hpp"
    #include "vslamcore/createNewMapPoints.hpp"
    #include "vslamcore/pcfromdepth.hpp"
    #include "vslamcore/detectAndExtractFeatures.hpp"
#else
    #include "RGBDVisualSLAMImpl.hpp"
    #include "initializeMap.hpp"
    #include "createNewMapPoints.hpp"
    #include "pcfromdepth.hpp"
    #include "detectAndExtractFeatures.hpp"
#endif

namespace vision {
    namespace vslam {

        RGBDVisualSLAMImpl::RGBDVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const ConfigurationRGBD& configuration,
                const char* vocabFile,
                const VSlamConcurrency desiredLevel)
                : BaseVisualSLAMImpl<QueueInstanceRGBD, ConfigurationRGBD>(fx, fy, cx, cy, configuration, vocabFile, desiredLevel)
                {
                    config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::systemSetupRGBD");
                }

        void RGBDVisualSLAMImpl::addFrame(const cv::Mat& colorImage, cv::Mat& depthImage) {

            startThreadsIfIdle();

            QueueInstanceRGBD qObj;
            qObj.xyzPoints.reserve(config.baseParams.numFeatures);
            qObj.validIndex.reserve(config.baseParams.numFeatures);
            qObj.trackedMapPointIds.reserve(config.baseParams.numFeatures);
            qObj.trackedFeatureIndices.reserve(config.baseParams.numFeatures);

            // Detect and extract features from left and right frames in parallel
            detectAndExtractFeatures(colorImage, config.baseParams.scaleFactor, config.baseParams.numLevels, config.baseParams.numFeatures, qObj.currPoints, qObj.currFeatures);
            pcfromdepth(depthImage, intrinsics, config, qObj); //  qObj.validIndex is sorted

            addFrameToTrackingQueue(std::move(qObj));
        }

        void RGBDVisualSLAMImpl::initializeMap(QueueInstanceRGBD& qObj) {
            
            // Print "Starting map initialization" if no frame is added yet
            std::vector<int> frameIDs = this->getKeyFrameIndex();
            int sz = static_cast<int>(frameIDs.size());
            if(sz == 0) {
                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::startMapInitialization");
            }

            // Attempt map initialization
            isMapInitialized = initializeMapRGBD(*mapPoints, *keyFrames, *loopDatabase, qObj, intrinsics, config, generator);

            // After attempting map initialization with mono, two cases are possible:
            // 1. Map got initialized
            // 2. Initialized failed with second frame
            frameIDs = this->getKeyFrameIndex();
            sz = static_cast<int>(frameIDs.size());
            if (isMapInitialized) {
                initializationKeyFrameAdded();
                frameIDs = this->getKeyFrameIndex();
                sz = static_cast<int>(frameIDs.size());
                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::endMapInitializationOneFrame",
                                               {std::to_string(frameIDs[0] + 1)});  // Convert to 1-based indexing 
            } else if (sz == 0) { // Initialization failed at first frame
                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::failedMapInitializationOneFrame", {std::to_string(qObj.currFrameIndex + 1)});
            }
        }

        void RGBDVisualSLAMImpl::addNewMapPoints(QueueInstanceRGBD& qObj, std::vector<int>& refinedViewIds, std::vector<int>& fixedViewIds) {
            // Get the 3-D world points that are tracked in the current frame
            std::vector<int> untrackedFeatureIdx, ia;
            untrackedFeatureIdx.reserve(qObj.validIndex.size());
            ia.reserve(qObj.validIndex.size());
            std::sort(qObj.trackedFeatureIndices.begin(), qObj.trackedFeatureIndices.end());
            set_difference_index(qObj.validIndex.begin(), qObj.validIndex.end(),
                qObj.trackedFeatureIndices.begin(), qObj.trackedFeatureIndices.end(),
                std::inserter(untrackedFeatureIdx, untrackedFeatureIdx.begin()), ia);

            // Add 3-D world points created from depth image
            stereoCorrespondence.clear();
            stereoCorrespondence.reserve(ia.size());
            for (int i = 0; i != static_cast<int>(ia.size()); ++i) {
                const int currPointId = generator.newIDs(NodeType::POINT_XYZ).identifier;
                cv::Vec3d locationInWorld = qObj.currExtrinsics_R.t() * (qObj.xyzPoints[ia[i]] - qObj.currExtrinsics_t);
                auto newPoint(std::make_shared<WorldPoint>(currPointId, locationInWorld));
                mapPoints->addWorldPoint(newPoint);

                // Add 3-D to 2-D correspondence for the current keyframe
                mapPoints->addCorrespondence(currPointId, qObj.currKeyFrameId, untrackedFeatureIdx[i], *keyFrames);

                // Update attributes so that they are not empty
                mapPoints->updateLimitsAndDirection(currPointId, *keyFrames);

                // Update representative view
                mapPoints->updateRepresentativeView(currPointId, *keyFrames);

                // Record the 2-D to 3-D correspondences built from depth image
                stereoCorrespondence[untrackedFeatureIdx[i]] = currPointId;
            }
            createNewMapPoints(*mapPoints, *keyFrames, qObj, intrinsics, stereoCorrespondence, refinedViewIds, fixedViewIds, config);
        }

        void RGBDVisualSLAMImpl::cullRecentMapPoints() {
            constexpr int minNumViewsDepth{ 2 };
            int numCulledPoints = 0;
            for (const auto& pair : stereoCorrespondence) {
                if (mapPoints->getWorldPoint(pair.second)->getViewsOfWorldPoint().size() < minNumViewsDepth) {
                    mapPoints->setInvalid(pair.second, *keyFrames);
                    numCulledPoints++;
                }
            }
            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NumCulledPoints", {std::to_string(numCulledPoints)});
        }

    } // namespace vslam
} // namespace vision