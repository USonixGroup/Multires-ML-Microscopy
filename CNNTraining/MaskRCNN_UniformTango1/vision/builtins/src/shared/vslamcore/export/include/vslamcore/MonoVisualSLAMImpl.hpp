////////////////////////////////////////////////////////////////////////////////
//  MonoVisualSLAMImpl.hpp
//
//  MonoVisualSLAMImpl class header file.
//
//  Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef MONOVISUALSLAMIMPL_HPP
#define MONOVISUALSLAMIMPL_HPP

// base class
#include "BaseVisualSLAMImpl.hpp"

namespace vision {
    namespace vslam {

        class LIBMWVSLAMCORE_API MonoVisualSLAMImpl : public BaseVisualSLAMImpl<QueueInstanceBase, ConfigurationMono> {

        public:
            MonoVisualSLAMImpl() = delete;

            /**
            * @brief Constructor
            */
            MonoVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const ConfigurationMono& configuration,
                const char* vocabFile,
                const VSlamConcurrency desiredLevel = VSlamConcurrency::THREAD_L2,
                const cv::Matx44d& camToIMU = cv::Matx44d::zeros());
            
            /**
            * @brief Add an image frame to the system and check if it is a key frame
            *
            * @param[in] img current frame represented as a cv::Mat
            * @param[in] imuG current gyroscope measurements represented as an N-by-3 cv::Mat
            * @param[in] imuA current accelerometer measurements represented as an N-by-3 cv::Mat
            */
            void addFrame(const cv::Mat& frame,
                const cv::Mat& imuG = cv::Mat(),
                const cv::Mat& imuA = cv::Mat());

            /**
            * @brief Stores the results of the scale and gravity direction estimation
            *
            * param[in]: camToLocalIMU, transform containing the rotation needed to align the camera frame with IMU frame.
            * param[in]: poseScale, number used to scale the camera poses to the same metric units as the IMU measurements.
            */
            void storeGravityRotationAndScale(const cv::Matx44d& camToLocalIMU, const double poseScale);

        private:
            /**
            * @brief Initializes map for stereo images
            *
            * @param[in] qObj
            */
            void initializeMap(QueueInstanceBase& qObj) override;

            /**
            * @brief Adds new map points from stereo reconstruction and triangulation
            *
            * @param[in] qObj
            * @param[in] refinedViewIds
            * @param[in] fixedViewIds
            */
            void addNewMapPoints(QueueInstanceBase& qObj, std::vector<int>& refinedViewIds, std::vector<int>& fixedViewIds) override;

            /**
            * @brief Remove recently-created map points that are observed by less than 2 views.
            */
            void cullRecentMapPoints() override;
            
            /**
            * @brief generate unique identifier for a pose. Override so that two ID
            * are generated, one for pose and one for scale.
            *
            * @return new unique identifier
            */
            int generateNewPoseID() override;
            
            /**
            * IDs of map points created recently using triangulation from the last key frame
            */
            std::vector<int> lastMapPointIds;

            /**
            * IDs of map points created recently using triangulation from the second last key frame
            */
            std::vector<int> secondLastMapPointIds;
        }; // class MonoVisualSLAMImpl

    } // namespace vslam
} // namespace vision
#endif // MONOVISUALSLAMIMPL_HPP