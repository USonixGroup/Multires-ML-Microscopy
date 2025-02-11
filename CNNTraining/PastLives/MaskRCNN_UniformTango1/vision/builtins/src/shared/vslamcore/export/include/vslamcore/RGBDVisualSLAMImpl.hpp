////////////////////////////////////////////////////////////////////////////////
//  RGBDVisualSLAMImpl.hpp
//
//  RGBDVisualSLAMImpl class header file.
//
//  Copyright 2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef RGBDVISUALSLAMIMPL_HPP
#define RGBDVISUALSLAMIMPL_HPP

// base class
#include "BaseVisualSLAMImpl.hpp"

namespace vision {
    namespace vslam {

        // Mimic the behavior of setdiff in MATLAB
        template<class InputIt1, class InputIt2, class OutputIt>
        OutputIt set_difference_index(InputIt1 first1, InputIt1 last1,
            InputIt2 first2, InputIt2 last2, OutputIt d_first, std::vector<int>& ia)
        {
            int idx{ 0 };
            while (first1 != last1)
            {
                if (first2 == last2){
                    ia.push_back(idx); // Last element
                    return std::copy(first1, last1, d_first);
                }

                if (*first1 < *first2) {
                    *d_first++ = *first1++;
                    ia.push_back(idx);
                    idx++;
                }
                else
                {
                    if (!(*first2 < *first1)) {
                        ++first1;
                        ++idx;
                    }
                    ++first2;
                }
            }
            return d_first;
        }

        class LIBMWVSLAMCORE_API RGBDVisualSLAMImpl : public BaseVisualSLAMImpl<QueueInstanceRGBD, ConfigurationRGBD> {

        public:
            RGBDVisualSLAMImpl() = delete;

            /**
            * @brief Constructor
            */
            RGBDVisualSLAMImpl(const double fx,
                const double fy,
                const double cx,
                const double cy,
                const ConfigurationRGBD& configuration,
                const char* vocabFile,
                const VSlamConcurrency desiredLevel = VSlamConcurrency::THREAD_L2);

            /**
            * @brief Add an image frame to the system and check if it is a key frame
            *
            * @param[in] colorImage color image of the current RGB-D pair as a cv::Mat
            * @param[in] depthImage depth image of the current RGB-D pair as a cv::Mat
            *
            */
            void addFrame(const cv::Mat& colorImage, cv::Mat& depthImage);

        private:
            /**
            * @brief Initializes map for stereo images
            *
            * @param[in] qObj
            */
            void initializeMap(QueueInstanceRGBD& qObj) override;

            /**
            * @brief Adds new map points from stereo reconstruction and triangulation
            *
            * @param[in] qObj
            * @param[in] refinedViewIds
            * @param[in] fixedViewIds
            */
            void addNewMapPoints(QueueInstanceRGBD& qObj, std::vector<int>& refinedViewIds, std::vector<int>& fixedViewIds) override;

            /**
            * @brief Remove recently-created map points that are observed by less than 2 views.
            */
            void cullRecentMapPoints() override;
            
            /**
            * 2-D to 3-D correspondence of map points created using stereo methods 
            */
            std::unordered_map<int, int> stereoCorrespondence;
        }; // class RGBDVisualSLAMImpl

    } // namespace vslam
} // namespace vision
#endif // RGBDVISUALSLAMIMPL_HPP
