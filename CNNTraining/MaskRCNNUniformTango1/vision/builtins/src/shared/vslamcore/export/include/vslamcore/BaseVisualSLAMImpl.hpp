////////////////////////////////////////////////////////////////////////////////
//  BaseVisualSLAMImpl.hpp
//
//  BaseVisualSLAMImpl class header file.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef BASEVISUALSLAMIMPL_HPP
#define BASEVISUALSLAMIMPL_HPP

#include <fstream>
#include <algorithm>
#include <string>
#include <numeric>
#include <cassert>
#include <cmath>
#include <thread>

#include "BaseVisualSLAMInterface.hpp"

// Data Management
#include "MapPointSet.hpp"
#include "KeyFrameSet.hpp"
#include "WorldPoint.hpp"
#include "QueueInstance.hpp"
#include "IMUInfo.hpp"
#include "LoopClosureDatabase.hpp"
#include "NodeIDGenerator.hpp"

// Thread management
#include <chrono>
#include <mutex>
#include <thread>
#include "SafeQueue.hpp"
#include "SafeX.hpp"
#include "ThreadManager.hpp"
#include "VSLAMLoggerBase.hpp"

namespace vision {
    namespace vslam {

        // Enum used for concurrency level
        enum class VSlamConcurrency : int {
            DEBUG,      // Single thread
            THREAD_L1,  // One separate thread for Tracking, Mapping, Loop closing
            THREAD_L2   // Three threads, one each for Tracking, Mapping, Loop closing
        };

        template <typename TQObj, typename TConfig>
        class LIBMWVSLAMCORE_API BaseVisualSLAMImpl : public BaseVisualSLAMInterface {

        public:
            BaseVisualSLAMImpl() = delete;
            virtual ~BaseVisualSLAMImpl();

            /**
            * @brief Constructor
            */
            BaseVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const TConfig& configuration,
                const char* vocabFile,
                const VSlamConcurrency desiredLevel = VSlamConcurrency::THREAD_L2,
                const cv::Matx44d& camToIMU = cv::Matx44d::eye());

            /**
            * @brief Get the map points
            *
            * @return [X, Y, Z] map points represented as a vector of cv::Vec3d
            *
            */
            std::vector<cv::Vec3d> getWorldPoints() override;

            /**
            * @brief Get the camera poses of key frames
            *
            * @return a pair of vectors containing view Ids and camera poses of the key frames
            *
            */
            std::pair<std::vector<int>, std::vector<cv::Matx44d>> getCameraPoses() override;

            /**
            * @brief Get the key frame IDs
            *
            * @return a vector of key frame IDs
            *
            */
            std::vector<int> getKeyFrameIndex() override;

            /**
            * @brief Get the view IDs
            *
            * @return a vector of view IDs
            *
            */
            std::vector<int> getViewIDs() override;

            /**
            * @brief Get the number of tracked feature points/map points in the current frame
            *
            * @return number of points as an integer
            *
            */
            int getNumTrackedPoints() override;

            /**
            * @brief Check whether a new key frame is added
            *
            * @return flag indicating if new key frame is added
            *
            */
            bool hasNewKeyFrame() override;

            /**
            * @brief Check whether the visual SLAM object is done 
            *        processing all added frames
            *
            * @return flag indicating all frames are processed
            *
            */
            bool isDone() override;

            /**
            * @brief Check if the map has been initialized
            *
            * @return flag indicating if map initialization is successful
            *
            */
            bool getIsMapInitialized() const override;

            /**
            * @brief Check if pose graph optimization was performed recently
            *
            * @return flag indicating if the loop was closed recently
            *
            */
            bool getIsLoopRecentlyClosed(const bool reset = true) override;

            /**
            * @brief Get verbose log filename
            *
            * @return log filename
            *
            */
            std::string getLogFileName() const override;

            /**
            * @brief Get the IMU measurements attached the the viewId
            *
            * param[in]: viewId, an integer representing the ID of a view.
            * param[out]: a pair containing the gyroscope and accelerometer measurements attached the the viewId.
            */
            std::pair<std::vector<double>, std::vector<double>> getViewIMUMeasurements(const int viewId) const override;
            
            /**
            * @brief Get the number of IMU measurements for the inputted viewId
            *
            * param[in] viewId, and integer representing the ID of a view.
            *
            * @return N, the number of IMU measurements (N-by-3) for the inputted viewId
            */
            int getNumIMUMeasurements(const int viewId) const override;

            /**
            * @brief Closes all internal processes and clears all data
            *
            */
            void reset() override;

        protected:
            /**
            * @brief Initialize threads if idle and not in DEBUG mode.
            */
            void startThreadsIfIdle();

            /**
            * @brief Push extracted features and points to the trackingQueue.
            *
            * @param[in] qObj
            */
            void addFrameToTrackingQueue(TQObj&& qObj);

            /**
            * @brief Set key frame properties during map initialization
            */
            void initializationKeyFrameAdded();
            
            /**
            * @brief Push copy of current frame IMU data to buffers
            *
            * @param[in] imuG current gyroscope measurements represented as an N-by-3 cv::Mat
            * @param[in] imuA current accelerometer measurements represented as an N-by-3 cv::Mat
            */
            void pushToIMUBuffers(const cv::Mat& imuG, const cv::Mat& imuA, std::vector<double>& gyro, std::vector<double>& accel);

            /**
            * Camera intrinsics matrix
            */
            const cv::Matx33d intrinsics;

            /**
            * Configuration of the system
            */
            const TConfig config;

            /**
            * ID generator for poses and points
            */
            NodeIDGenerator generator;

            /**
            * Stores info related to IMU fusion
            */
            IMUInfo imuInfo;

            /**
            * Flag indicating if map initialization is successful
            */
            bool isMapInitialized{ false };

            /**
            * Key frames
            */
            std::unique_ptr<KeyFrameSet> keyFrames;

            /**
            * Map points
            */
            std::unique_ptr<MapPointSet> mapPoints;

            /**
            * Loop closure detection database
            */
            std::unique_ptr<LoopClosureDatabase> loopDatabase;

            /**
        private:
            /**
            * @brief Pure virtual function for initializing map
            */
            virtual void initializeMap(TQObj& qObj) = 0;

            /**
            * Pure virtual function for adding 3-D map points after adding a new key frame
            */
            virtual void addNewMapPoints(TQObj& qObj, std::vector<int>& refinedViewIds, std::vector<int>& fixedViewIds) = 0;

            /**
            * Pure virtual function to remove recently-created map points that are observed by too few views.
            * The purpose is to keep only map points with strong observability.
            */
            virtual void cullRecentMapPoints() = 0;

            /**
            * @brief generate unique identifier for a pose
            *
            * @return new unique identifier
            */
            virtual int generateNewPoseID();

            /**
             * @brief Function for tracking thread
             *
             */
            void trackingModule();

            /**
             * @brief Function for mapping thread
             *
             */
            void mappingModule();

            /**
             * @brief Function for loop closing thread
             *
             */
            void loopClosingModule();

            /**
            * @brief Reset variables and initialize threads
            *
            * @param[in] createThreads set true to create threads
            * @param[in] resetData set true to reset internal data structures
            */
            void initialize(const bool createThreads, const bool resetData);

            /**
            * @brief Attaches the stored IMU data to the latest view
            *
            */
            void setCurrentViewIMU();

            /**
            * Vocabulary file path
            */
            const std::string vocabularyFilePath;

            /**
            * This flag indicates whether we run in single thread or not
            */
            const VSlamConcurrency threadConcurrency;

            /**
            * Flag indicating if the loop was closed recently
            */
            SafeX<bool> isLoopRecentlyClosed;

            /**
            * Indices and IDs used in the tracking process
            */
            int currFrameIndex{ 0 }, lastKeyFrameIndex{ 0 };

            /**
            * Number of key frames passed since last pose graph optimization
            */
            int numKeyFramesSinceLastLoop{ 0 };

            /**
            * Thread manager storing all threads
            */
            ThreadManager threadManager;

            /**
            * Queue to store extracted keypoints and feature pairs
            */
            SafeQueue< TQObj > trackingQueue;

            /**
            * Queue to store tracking artifacts
            */
            SafeQueue< TQObj > mappingQueue;

            /**
            * Queue to store loop closing artifacts
            */
            SafeQueue< TQObj > loopClosingQueue;

            /**
            * Buffer used to store gyroscope data while waiting for the next KF
            */
            std::vector<double> gyroBuffer;

            /**
            * Buffer used to store accelerometer data while waiting for the next KF
            */
            std::vector<double> accelBuffer;

            /**
            * Integer identifiers for tracking, mapping, and loop closure threads
            */
            int trackingThreadIdx, mappingThreadIdx, loopClosingThreadIdx;

            /**
            * This flag controls the life of running threads.
            * If set to true, all threads will terminate
            */
            SafeX<bool> stopFlag;

            /**
            * Number of tracked points used for computing status of vslam
            */
            SafeX<int> numTrackedPoints;

            /**
            * This flag indicates whether tracking thread is processing
            */
            SafeX<bool> isTrackingInProgress;

            /**
            * This flag indicates whether mapping thread is processing
            */
            SafeX<bool> isMappingInProgress;

            /**
            * This flag indicates whether loop closure thread is processing
            */
            SafeX<bool> isLoopClosingInProgress;

            /**
            * This flag indicates whether loop detection was initiated
            */
            SafeX<bool> isLoopCorrectionInitiated;

            /**
            * This flag indicates whether a new key frame is added
            */
            SafeX<bool> isNewKeyFrameAdded;

            /**
            * Flag used to terminate BA when loop is detected
            */
            bool abortBA;

        }; // class BaseVisualSLAMImpl

    } // namespace vslam
} // namespace vision
#endif // BASEVISUALSLAMIMPL_HPP