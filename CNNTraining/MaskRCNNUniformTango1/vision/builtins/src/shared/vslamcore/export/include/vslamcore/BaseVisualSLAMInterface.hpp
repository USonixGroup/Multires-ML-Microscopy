////////////////////////////////////////////////////////////////////////////////
//  BaseVisualSLAMInterface.hpp
//
//  BaseVisualSLAMInterface class header file.
//
//  Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef BASEVISUALSLAMINTERFACE_HPP
#define BASEVISUALSLAMINTERFACE_HPP

#include <vector>
#include <string>

#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"

#include "libmwvslamcore_util.hpp"

namespace vision {
    namespace vslam {

        class LIBMWVSLAMCORE_API BaseVisualSLAMInterface {

        public:
            /**
            * @brief Pure virtual function to check whether the visual
            *        SLAM object is done processing all added frames
            *
            * @return flag indicating all frames are processed
            *
            */
            virtual bool isDone() = 0;

            /**
            * @brief Pure virtual function to close all internal processes and clear all data
            *
            */
            virtual void reset() = 0;

            /**
            * @brief Get the map points
            *
            * @return [X, Y, Z] map points represented as a vector of cv::Vec3d
            *
            */
            virtual std::vector<cv::Vec3d> getWorldPoints() = 0;

            /**
            * @brief Get the camera poses of key frames
            *
            * @return a pair of vectors containing view Ids and camera poses of the key frames
            *
            */
            virtual std::pair<std::vector<int>, std::vector<cv::Matx44d>> getCameraPoses() = 0;

            /**
            * @brief Get the key frame IDs
            *
            * @return a vector of key frame IDs
            *
            */
            virtual std::vector<int> getKeyFrameIndex() = 0;

            /**
            * @brief Get the view IDs
            *
            * @return a vector of View IDs.
            *
            */
            virtual std::vector<int> getViewIDs() = 0;

            /**
            * @brief Get the number of tracked feature points/map points in the current frame
            *
            * @return number of points as an integer
            *
            */
            virtual int getNumTrackedPoints() = 0;

            /**
            * @brief Check whether a new key frame is added
            *
            * @return flag indicating if new key frame is added
            *
            */
            virtual bool hasNewKeyFrame() = 0;

            /**
            * @brief Check if the map has been initialized
            *
            * @return flag indicating if map initialization is successful
            *
            */
            virtual bool getIsMapInitialized() const = 0;

            /**
            * @brief Check if pose graph optimization was performed recently
            *
            * @return flag indicating if the loop was closed recently
            *
            */
            virtual bool getIsLoopRecentlyClosed(const bool reset = true) = 0;

            /**
            * @brief Get verbose log filename
            *
            * @return log filename
            *
            */
            virtual std::string getLogFileName() const = 0;

            /**
            * @brief Get the IMU measurements attached the the viewId
            *
            * param[in]: viewId, an integer representing the ID of a view.
            * param[out]: a pair containing the gyroscope and accelerometer measurements attached the the viewId.
            */
            virtual std::pair<std::vector<double>, std::vector<double>> getViewIMUMeasurements(const int viewId) const = 0;
            
            /**
            * @brief Get the number of IMU measurements for the inputted viewId
            *
            * param[in] viewId, and integer representing the ID of a view.
            */
            virtual int getNumIMUMeasurements(const int viewId) const = 0;
        }; // class BaseVisualSLAMInterface

    } // namespace vslam
} // namespace vision
#endif // BASEVISUALSLAMINTERFACE_HPP