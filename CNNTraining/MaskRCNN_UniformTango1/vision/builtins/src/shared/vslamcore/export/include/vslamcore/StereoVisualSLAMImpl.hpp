////////////////////////////////////////////////////////////////////////////////
//  StereoVisualSLAMImpl.hpp
//
//  StereoVisualSLAMImpl class header file.
//
//  Copyright 2023-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef STEREOVISUALSLAMIMPL_HPP
#define STEREOVISUALSLAMIMPL_HPP

// base class
#include "BaseVisualSLAMImpl.hpp"

// utility classes
#include "StereoReconstructor.hpp"

namespace vision {
    namespace vslam {

        class LIBMWVSLAMCORE_API StereoVisualSLAMImpl : public BaseVisualSLAMImpl<QueueInstanceStereo, ConfigurationStereo> {

        public:
            StereoVisualSLAMImpl() = delete;

            /**
            * @brief Constructor
            */
            StereoVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const double baseline,
                const ConfigurationStereo& configuration,
                const char* vocabFile,
                const VSlamConcurrency desiredLevel = VSlamConcurrency::THREAD_L2);

            /**
            * @brief Add an image frame with optional disparity map to the system and check if it is a key frame
            *
            * @param[in] frameL left frame represented as a cv::Mat
            * @param[in] frameR right frame represented as a cv::Mat
            * @param[in] disparity map for the frame represented as a cv::Mat
            *
            * Notes: 
            *   If non-empty, disparity must be of same size as frameL and frameR and be of type CV_32FC1.
            *   
            *
            */
            void addFrame(const cv::Mat& frameL, const cv::Mat& frameR, const cv::Mat& disparity = cv::Mat());

        private:
            /**
            * @brief Initializes map for stereo images
            *
            * @param[in] qObj
            */
            void initializeMap(QueueInstanceStereo& qObj) override;

            /**
            * @brief Adds new map points from stereo reconstruction and triangulation
            *
            * @param[in] qObj
            * @param[in] refinedViewIds
            * @param[in] fixedViewIds
            */
            void addNewMapPoints(QueueInstanceStereo& qObj, std::vector<int>& refinedViewIds, std::vector<int>& fixedViewIds) override;

            /**
            * @brief Remove recently-created map points that are observed by less than 2 views.
            */
            void cullRecentMapPoints() override;

            /**
            * StereoReconstructor object used to construct 3-D world points using disparity
            */
            const std::unique_ptr<const StereoReconstructor> reconstructor;
            
            /**
            * 2-D to 3-D correspondence of map points created using stereo methods 
            */
            std::unordered_map<int, int> stereoCorrespondence;

        }; // class StereoVisualSLAMImpl

    } // namespace vslam
} // namespace vision
#endif // STEREOVISUALSLAMIMPL_HPP