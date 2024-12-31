////////////////////////////////////////////////////////////////////////////////
//  BaseVisualSLAMImpl.cpp
//
//  This is the base class template for visual SLAM.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/BaseVisualSLAMImpl.hpp"
    #include "vslamcore/correctLoop.hpp"
    #include "vslamcore/detectAndExtractFeatures.hpp"
    #include "vslamcore/converter.hpp"
    #include "vslamcore/geometry.hpp"
    #include "vslamcore/View.hpp"
    #include "vslamcore/bundleAdjustment.hpp"
    #include "vslamcore/addNewKeyFrame.hpp"
    #include "vslamcore/trackLocalMap.hpp"
    #include "vslamcore/trackLastKeyFrame.hpp"
    #include "vslamcore/GSTools.hpp"
#else
    #include "BaseVisualSLAMImpl.hpp"
    #include "correctLoop.hpp"
    #include "detectAndExtractFeatures.hpp"
    #include "converter.hpp"
    #include "geometry.hpp"
    #include "View.hpp"
    #include "bundleAdjustment.hpp"
    #include "addNewKeyFrame.hpp"
    #include "trackLocalMap.hpp"
    #include "trackLastKeyFrame.hpp"
    #include "GSTools.hpp"
#endif

namespace vision {
    namespace vslam{

        /**
        * @brief Determine vocab filepath during class initialization.
        *
        * @param[in] vocabFile, the filepath passed into the constructor
        * @return filepath as a const char*
        */
        const char* getVocabFile(const char* vocabFile) {
            #ifdef VSLAMROSCODEGEN
                return "bagOfFeatures.bin.gz";
            #else
                return vocabFile;
            #endif
        }

        /**
        * @brief Determine thread level during class initialization based on user input and 
        * the number of cores.
        *
        * @param[in] desiredLevel, a VSlamConcurrency representing the desired level of multithreading.
        * @return VSlamConcurrency enum for the level of multithreading
        */
        VSlamConcurrency getThreadConcurrency(const VSlamConcurrency desiredLevel) {
            if(desiredLevel == VSlamConcurrency::DEBUG || desiredLevel == VSlamConcurrency::THREAD_L1) {
                return desiredLevel;
            } else {
                // With L2, we need to assess hardware concurrency to assign the thread level.
                const int numCores = static_cast<int>(std::thread::hardware_concurrency());
                if(numCores > 4){ // 4 or more processor cores is good for L2
                    return VSlamConcurrency::THREAD_L2;
                } else {
                    return VSlamConcurrency::THREAD_L1;
                }
            }
        }

        template<typename TQObj,typename TConfig>
        BaseVisualSLAMImpl<TQObj,TConfig>::~BaseVisualSLAMImpl() {
            reset();
        }

        template<typename TQObj,typename TConfig>
        BaseVisualSLAMImpl<TQObj,TConfig>::BaseVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const TConfig& configuration,
                const char* vocabFile,
                const VSlamConcurrency desiredLevel,
                const cv::Matx44d& camToIMU)
                : intrinsics(fx, 0, cx, 0, fy, cy, 0, 0, 1),
                  config(configuration),
                  vocabularyFilePath(getVocabFile(vocabFile)),
                  threadConcurrency(getThreadConcurrency(desiredLevel)) {
            if (config.imuParams.hasIMU) {
                imuInfo.camToIMUTransform = camToIMU;
            }

            // Internalize internal data structures and start threads
            initialize(true, true);
        }


        template<typename TQObj,typename TConfig>
        std::vector<cv::Vec3d> BaseVisualSLAMImpl<TQObj,TConfig>::getWorldPoints() {
            return mapPoints->getLocation();
        }

        template<typename TQObj,typename TConfig>
        std::pair<std::vector<int>, std::vector<cv::Matx44d>> BaseVisualSLAMImpl<TQObj,TConfig>::getCameraPoses() {
            return keyFrames->getPoses();
        }

        template<typename TQObj, typename TConfig>
        std::vector<int> BaseVisualSLAMImpl<TQObj, TConfig>::getKeyFrameIndex() {
            return keyFrames->getKeyFrameIndex();
        }

        template<typename TQObj, typename TConfig>
        std::vector<int> BaseVisualSLAMImpl<TQObj, TConfig>::getViewIDs() {
            return keyFrames->getViewIds();
        }

        template<typename TQObj,typename TConfig>
        int BaseVisualSLAMImpl<TQObj,TConfig>::getNumTrackedPoints() {
            return numTrackedPoints;
        }

        template<typename TQObj,typename TConfig>
        bool BaseVisualSLAMImpl<TQObj,TConfig>::hasNewKeyFrame() {
            const bool retval(isNewKeyFrameAdded);
            if(retval) { // Reset value once read
                isNewKeyFrameAdded = false;
            }
            return retval;
        }

        template<typename TQObj,typename TConfig>
        bool BaseVisualSLAMImpl<TQObj,TConfig>::isDone() {
            return trackingQueue.empty() && mappingQueue.empty() && loopClosingQueue.empty() &&
                   !isTrackingInProgress && !isMappingInProgress && !isLoopClosingInProgress;
        }

        template<typename TQObj,typename TConfig>
        bool BaseVisualSLAMImpl<TQObj,TConfig>::getIsMapInitialized() const {
            return isMapInitialized;
        }

        template<typename TQObj,typename TConfig>
        bool BaseVisualSLAMImpl<TQObj,TConfig>::getIsLoopRecentlyClosed(const bool reset) {
            const bool retval(isLoopRecentlyClosed);
            if(retval && reset) { // Reset value once read
                isLoopRecentlyClosed = false;
            }
            return retval;
        }

        template<typename TQObj,typename TConfig>
        std::string BaseVisualSLAMImpl<TQObj,TConfig>::getLogFileName() const {
            std::string retval = "";
            if(config.loggerPtr != nullptr) {
                retval = config.loggerPtr->getLogFileName();
            }
            return retval;
        }

        template<typename TQObj, typename TConfig>
        std::pair< std::vector<double>, std::vector<double> > BaseVisualSLAMImpl<TQObj, TConfig>::getViewIMUMeasurements(const int viewId) const {
            return keyFrames->findView(viewId)->getIMU();
        }
        
        template<typename TQObj, typename TConfig>
        int BaseVisualSLAMImpl<TQObj, TConfig>::getNumIMUMeasurements(const int viewId) const {
            return static_cast<int>(keyFrames->findView(viewId)->getIMU().first.size() / 3);
        }

        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::reset() {
            const std::chrono::milliseconds ms1(1);
            // Try stopping the threads and wait if any thread is still running
            stopFlag = true;
            while(isTrackingInProgress || isMappingInProgress || isLoopClosingInProgress){
                std::this_thread::sleep_for(ms1);
            }

            // Reset all internal data structures
            // Do not restart the threads
            initialize(false, true);
        }

        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::startThreadsIfIdle() {
            if(threadManager.empty() && threadConcurrency != VSlamConcurrency::DEBUG){
                initialize(true, false);
            }
        }

        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::addFrameToTrackingQueue(TQObj&& qObj) {
            trackingQueue.push(std::move(qObj));

            // Call function directly when we are in debug mode
            if (threadConcurrency == VSlamConcurrency::DEBUG) {
                trackingModule();
            }
        }

        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::initializationKeyFrameAdded() {
            lastKeyFrameIndex = currFrameIndex;
            isNewKeyFrameAdded = true;

            // Check if the IMU is in use, then if the current KF is the second KF (and not the first).
            if (config.imuParams.hasIMU && keyFrames->getMaxViewId() > 1) {
                // Attach IMU data to the view and clear the buffers
                setCurrentViewIMU();
            }
        }

        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::pushToIMUBuffers(const cv::Mat& imuG, const cv::Mat& imuA, std::vector<double>& gyro, std::vector<double>& accel) {
            // Push to buffer in row-major fashion
            auto pushToBuffer = [](const cv::Mat& inMat, std::vector<double>& buffer) {
                for (int i = 0; i < inMat.rows; ++i) {
                    for (int j = 0; j < inMat.cols; ++j) {
                        buffer.push_back(inMat.at<double>(i,j));
                    }
                }
            };
            pushToBuffer(imuG, gyro);
            pushToBuffer(imuA, accel);
        }

        template<typename TQObj,typename TConfig>
        int BaseVisualSLAMImpl<TQObj,TConfig>::generateNewPoseID() {
            if (config.imuParams.hasIMU) {
                return generator.newIDs(SensorConfig::CAMERA_IMU_SIM3)[0].identifier;
            }
            else {
                return generator.newIDs(NodeType::POSE_SE3).identifier;
            }
        }

        template<typename TQObj, typename TConfig>
        void BaseVisualSLAMImpl<TQObj, TConfig>::setCurrentViewIMU() {
            keyFrames->findView(keyFrames->getMaxViewId())->setIMU(gyroBuffer, accelBuffer);
            gyroBuffer.clear();
            accelBuffer.clear();
        }

        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::trackingModule() {
            while(!stopFlag) {
                if(!trackingQueue.empty()) {
                    isTrackingInProgress = true;

                    auto qObj(trackingQueue.front());
                    trackingQueue.pop();
                    
                    qObj.currFrameIndex = currFrameIndex;

                    if (!isMapInitialized) {
                        initializeMap(qObj);

                        // Update number of tracked points as they might have been updated
                        numTrackedPoints = static_cast<int>(qObj.trackedFeatureIndices.size());

                        config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NumTrackedPointsWithThreshold", {std::to_string(numTrackedPoints),
                                                                                                        std::to_string(config.baseParams.minNumPnPPoints)});
                    }
                    else
                    {
                        // Tracking is NOT allowed to run any number of frames ahead of mapping.
                        // It can though run in parallel with BA.
                        while (static_cast<int>(mappingQueue.size()) > 0)
                        {
                            std::chrono::milliseconds ms1(1);
                            std::this_thread::sleep_for(ms1);
                        }

                        keyFrames->lock();

                        // If loop detection is in progress, then wait for it to complete.
                        if(isLoopCorrectionInitiated) {
                            keyFrames->unlock();
                            while(isLoopCorrectionInitiated)
                            {
                                std::chrono::milliseconds ms1(1);
                                std::this_thread::sleep_for(ms1);
                            }
                            keyFrames->lock();
                        }

                        gyroBuffer.insert(gyroBuffer.end(), qObj.gyro.begin(), qObj.gyro.end());
                        accelBuffer.insert(accelBuffer.end(), qObj.accel.begin(), qObj.accel.end());

                        if (imuInfo.isGSAvailable) {

                            // keyFrames is already locked and will be unlocked at the end of BA
                            applyGravityRotationAndScale(*keyFrames, *mapPoints, imuInfo);
                            std::vector<int> fixedViewIds = {};
                            bundleAdjustment(*mapPoints, *keyFrames, intrinsics, keyFrames->getViewIds(), fixedViewIds, config, &abortBA, imuInfo);
                            imuInfo.isGSAvailable = false;
                        }

                        config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::startTracking", {std::to_string(qObj.currFrameIndex + 1)});

                        trackLastKeyFrame(*mapPoints, *keyFrames, qObj, intrinsics, config, imuInfo, gyroBuffer, accelBuffer);

                        // Update number of tracked points as they might have been updated
                        numTrackedPoints = static_cast<int>(qObj.trackedFeatureIndices.size());

                        config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NumTrackedPointsWithThreshold", {std::to_string(numTrackedPoints),
                                                                                                        std::to_string(config.baseParams.minNumPnPPoints)});

                        if (!qObj.trackedMapPointIds.empty()) {
                            const bool isKeyFrame = trackLocalMap(*mapPoints, *keyFrames, qObj, intrinsics, lastKeyFrameIndex, currFrameIndex, config, imuInfo, gyroBuffer, accelBuffer);
                            // Update number of tracked points as they might have been updated
                            numTrackedPoints = static_cast<int>(qObj.trackedFeatureIndices.size());

                            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NumTrackedPointsWithThreshold", {std::to_string(numTrackedPoints),
                                                                                                        std::to_string(config.baseParams.minNumPnPPoints)});

                            if (isKeyFrame) {
                                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::keyFrameDetected", {std::to_string(qObj.currFrameIndex + 1)});

                                lastKeyFrameIndex = currFrameIndex;
                                
                                // Store the unoptimized pose for last key frame. We do this
                                // so that if the last key frame is optimized after BA, then
                                // we can use this last keyframe pose to compute updated values for current keyframe.
                                const auto lastKeyFrame(keyFrames->findView(keyFrames->getMaxViewId()));
                                qObj.lastPose_R = lastKeyFrame->getOrientation();
                                qObj.lastPose_t = lastKeyFrame->getLocation();

                                mappingQueue.push(std::move(qObj));

                                // Call function directly when we are in debug mode or thread_L1 mode.
                                // Mapping becomes a separate thread in L2 mode only
                                if (threadConcurrency != VSlamConcurrency::THREAD_L2) {
                                    mappingModule();
                                }
                            } else if(numTrackedPoints < config.baseParams.minNumPnPPoints){
                                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::trackingLostAtFrame");
                            }
                        } else if(numTrackedPoints < config.baseParams.minNumPnPPoints){
                            config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::trackingLostAtFrame");
                        }


                        keyFrames->unlock();
                    }
                    currFrameIndex++;

                    isTrackingInProgress = false;
                }

                // For debug mode, we do not need an infinite while loop, so break out
                if (threadConcurrency == VSlamConcurrency::DEBUG) {
                    break;
                }
            }
        }

        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::mappingModule() {
            while(!stopFlag) {
                if(!mappingQueue.empty()) {
                    isMappingInProgress = true;

                    keyFrames->lock();

                    auto qObj(mappingQueue.front());
                    mappingQueue.pop();

                    qObj.currKeyFrameId = generateNewPoseID();

                    config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::startMapping", {std::to_string(qObj.currFrameIndex + 1)});


                    // Consider a scenario where BA of current frame ran after tracking of next frame 
                    // (and before mapping of next frame) the current keyframe. This means that current
                    // keyframe holds obsolete values. Update current KF extrinsics using the optimized 
                    // and unoptimized pose of last keyframe. Unoptimized pose of last keyframe comes
                    // from the queue object and optimized values come from the keyframe set.
                    const auto lastPose_opt = keyFrames->findView(keyFrames->getMaxViewId())->getPose();
                    unoptimized2OptimizedCurrExtrinsicsUsingLastPose(qObj, lastPose_opt);


                    addNewKeyFrame(*mapPoints, *keyFrames, qObj, config);

                    if (config.imuParams.hasIMU) {
                        // Attach IMU data to the view and clear the buffers
                        setCurrentViewIMU();
                    }


                    cullRecentMapPoints();


                    std::vector<int> refinedViewIds, fixedViewIds;
                    addNewMapPoints(qObj, refinedViewIds, fixedViewIds);

                    // Update number of tracked points as they might have been updated
                    numTrackedPoints = static_cast<int>(qObj.trackedFeatureIndices.size());

                    // We start loop closure detection first, so that it can run in parallel to BA
                    isLoopCorrectionInitiated = true;
                    loopClosingQueue.push(std::move(qObj));

                    keyFrames->unlock();

                    // Print max depth before BA in V3
                    if(config.baseParams.verbose == 3) {
                        config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::MaxDepthBeforeBA",
                                                       {std::to_string(qObj.currFrameIndex + 1), 
                                                        std::to_string(getMaxDepth(*mapPoints, *keyFrames, 
                                                                                    qObj.currKeyFrameId, qObj.currExtrinsics_R, 
                                                                                    qObj.currExtrinsics_t))});
                    }

                    
                    bundleAdjustment(*mapPoints, *keyFrames, intrinsics, refinedViewIds, fixedViewIds, config, &abortBA, imuInfo);


                    // Print max depth after BA in V3
                    if(config.baseParams.verbose == 3) {
                        config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::MaxDepthAfterBA",
                                                       {std::to_string(qObj.currFrameIndex + 1), 
                                                        std::to_string(getMaxDepth(*mapPoints, *keyFrames, 
                                                                                    qObj.currKeyFrameId, qObj.currExtrinsics_R, 
                                                                                    qObj.currExtrinsics_t))});
                    }

                    // Call function directly when we are in debug mode or thread_L1 mode.
                    // LoopClosing becomes a separate thread in L2 mode only
                    if(threadConcurrency != VSlamConcurrency::THREAD_L2) {
                        loopClosingModule();
                    }

                    isNewKeyFrameAdded = true;
                    isMappingInProgress = false;
                }

                // For debug mode, we do not need an infinite while loop, so break out
                if(threadConcurrency != VSlamConcurrency::THREAD_L2) {
                    break;
                }
            }
        }

        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::loopClosingModule() {
            while(!stopFlag) {
                if(!loopClosingQueue.empty()) {
                    isLoopClosingInProgress = true;
                    bool isLoopClosed = false;

                    auto qObj(loopClosingQueue.front());
                    loopClosingQueue.pop();

                    // Change features to feature vector, database accepts a vector
                    std::vector<cv::Mat> currFeaturesVec;
                    loopDatabase->changeFeaturesStructure(qObj.currFeatures, currFeaturesVec);

                    const bool addImgFeaturesOnly = numKeyFramesSinceLastLoop < config.baseParams.optimizationInterval;
                    if (!addImgFeaturesOnly) {
                        std::vector<std::vector<int>> loopKeyFrameIds;

                        config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::loopDetectionStart");

                        // Check loop closure
                        loopDatabase->checkLoopClosure(*keyFrames, qObj.currKeyFrameId, currFeaturesVec, config, loopKeyFrameIds);

                        config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::loopDetectionEnd");

                        // If loop exists then correct the loop
                        if (!loopKeyFrameIds.empty()) {
                            config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::loopDetected", {std::to_string(qObj.currFrameIndex + 1)});
                            
                            if(config.baseParams.verbose > 1) {
                                // Extract frame indices from all loop candidates
                                std::vector<std::vector<int>> loopFrameIndices;
                                for(int i = 0; i < static_cast<int>(loopKeyFrameIds.size()); i++) {
                                    std::vector<int> tempInd;
                                    for(int j = 0; j < static_cast<int>(loopKeyFrameIds[i].size()); j++) {
                                        tempInd.push_back(keyFrames->getFrameIndexByViewId(loopKeyFrameIds[i][j]));
                                    }
                                    loopFrameIndices.push_back(tempInd);
                                }
                                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::loopCandidates", {std::to_string(qObj.currFrameIndex + 1)});
                                // Print frame ids in the next line
                                config.loggerPtr->logMessageV2("", {convertVectorOfVectorToString<int>(loopFrameIndices)});
                            }

                            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::numKeyFramesSinceLastLoop", {std::to_string(numKeyFramesSinceLastLoop)});

                            // Get lock as nothing can run during loop correction
                            keyFrames->lock();

                            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::loopClosureStart");
                            
                            isLoopClosed = correctLoop(*mapPoints, *keyFrames, *loopDatabase, intrinsics, qObj.currKeyFrameId, currFeaturesVec, loopKeyFrameIds, config);
                            
                            if(isLoopClosed) {
                                config.loggerPtr->logMessageV1("vision::vslamVerboseCpp::loopClosed", {std::to_string(qObj.currFrameIndex + 1)});

                                if(threadConcurrency == VSlamConcurrency::THREAD_L2) {
                                    // Aborting is only for L2 mode
                                    abortBA = true;
                                }
                                isLoopRecentlyClosed = isLoopClosed;
                            } else {
                                config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::loopNotClosed");
                            }
                            
                            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::loopClosureEnd");
                            keyFrames->unlock();
                        } else {
                            config.loggerPtr->logMessageV3("vision::vslamVerboseCpp::loopNotDetected", {std::to_string(qObj.currFrameIndex + 1)});
                        }
                    }

                    // Add image features to database
                    loopDatabase->addImageFeatures(qObj.currKeyFrameId, std::move(currFeaturesVec));

                    if (isLoopClosed) {
                        numKeyFramesSinceLastLoop = 0;
                    }
                    else {
                        numKeyFramesSinceLastLoop++;
                    }

                    isLoopCorrectionInitiated = false;
                    isLoopClosingInProgress = false;
                }

                // For debug mode, we do not need an infinite while loop, so break out
                if(threadConcurrency != VSlamConcurrency::THREAD_L2) {
                    break;
                }
            }
        }
        
        template<typename TQObj,typename TConfig>
        void BaseVisualSLAMImpl<TQObj,TConfig>::initialize(const bool createThreads, const bool resetData) {
            if (resetData){
                numTrackedPoints = config.baseParams.numFeatures;

                keyFrames    = std::make_unique<KeyFrameSet>();
                mapPoints    = std::make_unique<MapPointSet>();
                loopDatabase = std::make_unique<LoopClosureDatabase>(LoopClosureDatabase(vocabularyFilePath));

                cv::theRNG().state = 0;

                // Thread related data
                isTrackingInProgress = false;
                isMappingInProgress = false;
                isLoopClosingInProgress = false;
                isLoopCorrectionInitiated = false;
                isNewKeyFrameAdded = false;
                abortBA = false;
                trackingThreadIdx = -1;
                mappingThreadIdx = -1;
                loopClosingThreadIdx = -1;
                trackingQueue.clear();
                mappingQueue.clear();
                loopClosingQueue.clear();

                currFrameIndex = 0;
                numKeyFramesSinceLastLoop = 0;
                isMapInitialized = false;
                isLoopRecentlyClosed = false;
            }

            if (createThreads) { // Thread management
                // If threads already exist then close them first
                if(!threadManager.empty()){
                    std::chrono::milliseconds ms1(1);
                    // Try stopping the threads and wait if any thread is still running
                    stopFlag = true;
                    while(isTrackingInProgress || isMappingInProgress || isLoopClosingInProgress){
                        std::this_thread::sleep_for(ms1);
                    }
                }

                stopFlag = false;

                // Create threads for non-debug modes only
                if (threadConcurrency != VSlamConcurrency::DEBUG) {
                    constexpr int threadCount = 1;
                    trackingThreadIdx = threadManager.createThread(threadCount, &BaseVisualSLAMImpl<TQObj,TConfig>::trackingModule, std::ref(*this));
                    threadManager.detachThread(trackingThreadIdx);

                    // Create threads for mapping and loop closure for L2 level only
                    if (threadConcurrency == VSlamConcurrency::THREAD_L2) {
                        mappingThreadIdx = threadManager.createThread(threadCount, &BaseVisualSLAMImpl<TQObj,TConfig>::mappingModule, std::ref(*this));
                        threadManager.detachThread(mappingThreadIdx);
                        loopClosingThreadIdx = threadManager.createThread(threadCount, &BaseVisualSLAMImpl<TQObj,TConfig>::loopClosingModule, std::ref(*this));
                        threadManager.detachThread(loopClosingThreadIdx);
                    }
                }
            }
        }

        // Explicit instantiations with the LIBMWVSLAMCORE_API flag are required to avoid linking errors on Linux
        template class LIBMWVSLAMCORE_API BaseVisualSLAMImpl<QueueInstanceBase, ConfigurationMono>;
        template class LIBMWVSLAMCORE_API BaseVisualSLAMImpl<QueueInstanceStereo, ConfigurationStereo>;
        template class LIBMWVSLAMCORE_API BaseVisualSLAMImpl<QueueInstanceRGBD, ConfigurationRGBD>;

    }
}