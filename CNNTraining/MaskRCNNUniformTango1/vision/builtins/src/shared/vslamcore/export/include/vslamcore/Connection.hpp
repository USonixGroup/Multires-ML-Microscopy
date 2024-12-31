////////////////////////////////////////////////////////////////////////////////
// Connection class storing relative camera pose and feature matches
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef CONNECTION_HPP
#define CONNECTION_HPP

#include <vector>
#include <utility>

#include "View.hpp"

namespace vision {
    namespace vslam {
        class View;
        class Connection {

        public:
            Connection() = delete;

            /**
            * @brief Constructor 
            *
            * @param[in] viewId1 Id of view 1, specified as an integer.
            * @param[in] viewId2 Id of view 2, specified as an integer.
            * @param[in] rot relative rotation between view 1 and view 2, specified as a cv::Matx33d.
            * @param[in] loc relative translation between view 1 and view 2, specified as a cv::Vec3d.
            * @param[in] matches indices of matched features, specified as a vector of index pairs.
            */
            Connection(const int viewId1,
                const int viewId2,
                const cv::Matx33d& rot,
                const cv::Vec3d& trans,
                const std::vector<std::pair<int, int>>& matches);

            //-------------------------------------------------------------------------
            // Getters
            //-------------------------------------------------------------------------

            /**
            * @brief Get relative camera pose
            *
            * @return Camera pose, returned as a cv::Matx44d.
            */
            cv::Matx44d getRelPose() const;

            /**
            * @brief Get relative camera rotation
            *
            * @return Relative camera rotation, returned as a cv::Matx33d.
            */
            cv::Matx33d getRelRotation() const;

            /**
            * @brief Get relative camera translation
            *
            * @return Relative Camera translation, returned as a cv::Vec3d.
            */
            cv::Vec3d getRelTranslation() const;

            /**
            * @brief Get matched feature indices
            *
            * @return Matched feature indices, returned as a vector of integer pairs.
            */
            std::vector<std::pair<int, int>> getMatches()  const;

            /**
            * @brief Get view IDs of the connection
            *
            * @return View IDs, returned as a pair of integers.
            */
            std::pair<int, int> getViewIdPair() const;

            
            //-------------------------------------------------------------------------
            // Setters
            //-------------------------------------------------------------------------

            /**
            * @brief Set relative camera pose
            *
            * @param[in] rot relative rotation specified as a cv::Matx33d.
            * @param[in] trans relative translation specified as a cv::Vec3d.
            */
            void setRelPose(const cv::Matx33d& rot, const cv::Vec3d& trans);

            /**
            * @brief Set relative camera rotation
            *
            * @param[in] rot relative rotation specified as a cv::Matx33d.
            */
            void setRelRotation(const cv::Matx33d& rot);

            /**
            * @brief Set relative camera translation
            *
            * @param[in] trans relative translation specified as a cv::Vec3d.
            */
            void setRelTranslation(const cv::Vec3d& trans);

            /**
            * @brief Add matched feature pairs on top of existing matched pairs
            *
            * @param[in] matches matched feature pairs specified as a vector of integer pairs.
            */
            void addMatches(const std::vector<std::pair<int, int>>& matches);

            /**
            * @brief Add a matched feature pair on top of existing matched pairs
            *
            * @param[in] matches matched feature pair specified as a pair of integers.
            */
            void addMatches(const std::pair<int, int>& matches);

            /**
            * @brief Destructor
            */
            ~Connection();
        private:
            
            /**
            * Relative camera rotation and translation
            */
            cv::Matx33d relRotation;
            cv::Vec3d relTranslation;

            
            /**
            * Ids of the two views in the connection
            */
            std::pair<int, int> viewIdPair;

            
            /**
            * Matched feature pairs
            */
            std::vector<std::pair<int, int>> matchedPairs;
        };
    }// namespace vslam
}// namespace vision

#endif // CONNECTION_HPP
