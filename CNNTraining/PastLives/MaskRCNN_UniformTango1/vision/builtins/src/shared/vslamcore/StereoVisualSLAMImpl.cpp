////////////////////////////////////////////////////////////////////////////////
//  StereoVisualSLAMImpl.cpp
//
//  This is the implementation of stereo visual SLAM.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/StereoVisualSLAMImpl.hpp"
    #include "vslamcore/initializeMap.hpp"
    #include "vslamcore/addStereoReconstructionPoints.hpp"
    #include "vslamcore/createNewMapPoints.hpp"
    #include "vslamcore/detectAndExtractFeatures.hpp"
#else
    #include "StereoVisualSLAMImpl.hpp"
    #include "initializeMap.hpp"
    #include "addStereoReconstructionPoints.hpp"
    #include "createNewMapPoints.hpp"
    #include "detectAndExtractFeatures.hpp"
#endif

namespace vision {
    namespace vslam {

        StereoVisualSLAMImpl::StereoVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const double baseline,
                const ConfigurationStereo& configuration,
                const char* vocabFile,
                const VSlamConcurrency desiredLevel)
                : BaseVisualSLAMImpl<QueueInstanceStereo, ConfigurationStereo>(fx, fy, cx, cy, configuration, vocabFile, desiredLevel),
                  reconstructor(std::make_unique<const StereoReconstructor>(StereoReconstructor(intrinsics, baseline, config)))
                  {
                      config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::systemSetupStereo");
                  }

        void StereoVisualSLAMImpl::addFrame(const cv::Mat& frameL, const cv::Mat& frameR, const cv::Mat& disparity) {
            startThreadsIfIdle();

            QueueInstanceStereo qObj;
            qObj.frameL = frameL;
            qObj.frameR = frameR;
            qObj.disparity = disparity;
            qObj.trackedMapPointIds.reserve(config.baseParams.numFeatures);
            qObj.trackedFeatureIndices.reserve(config.baseParams.numFeatures);

            // Detect and extract features from left and right frames in parallel
            std::thread detectFrameRThread(&detectAndExtractFeatures, std::ref(qObj.frameR), config.baseParams.scaleFactor,
                    config.baseParams.numLevels, config.baseParams.numFeatures, std::ref(qObj.pointsR), std::ref(qObj.featuresR));
            detectAndExtractFeatures(qObj.frameL, config.baseParams.scaleFactor, config.baseParams.numLevels, config.baseParams.numFeatures, qObj.currPoints, qObj.currFeatures);
            detectFrameRThread.join();
            
            addFrameToTrackingQueue(std::move(qObj));
        }

        void StereoVisualSLAMImpl::initializeMap(QueueInstanceStereo& qObj) {
            
            // Print "Starting map initialization" if no frame is added yet
            std::vector<int> frameIDs = this->getKeyFrameIndex();
            int sz = static_cast<int>(frameIDs.size());
            if(sz == 0) {
                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::startMapInitialization");
            }

            // Attempt map initialization
            isMapInitialized = initializeMapStereo(*mapPoints, *keyFrames, *loopDatabase, *reconstructor,
                qObj, intrinsics, config, generator);

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

        void StereoVisualSLAMImpl::addNewMapPoints(QueueInstanceStereo& qObj, std::vector<int>& refinedViewIds, std::vector<int>& fixedViewIds) {
            stereoCorrespondence = addStereoReconstructionPoints(*mapPoints, *keyFrames, *reconstructor, qObj, config, generator);

            createNewMapPoints(*mapPoints, *keyFrames, qObj, intrinsics, stereoCorrespondence, refinedViewIds, fixedViewIds, config);
        }

        void StereoVisualSLAMImpl::cullRecentMapPoints() {
            constexpr int minNumViews{ 2 };
            int numCulledPoints = 0;
            for (const auto& pair : stereoCorrespondence) {
                if (mapPoints->getWorldPoint(pair.second)->getViewsOfWorldPoint().size() < minNumViews) {
                    mapPoints->setInvalid(pair.second, *keyFrames);
                    numCulledPoints++;
                }
            }
            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NumCulledPoints", {std::to_string(numCulledPoints)});
        }

    } // namespace vslam
} // namespace vision
