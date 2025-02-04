////////////////////////////////////////////////////////////////////////////////
// World point class storing location, distance limits, viewing direction and 
// representative view.
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef WORLDPOINT_HPP
#define WORLDPOINT_HPP

#include <limits>
#include <unordered_map>

#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"

#include "View.hpp"
#include "KeyFrameSet.hpp"

namespace vision {
    namespace vslam {

        class View;
        class KeyFrameSet;

        class WorldPoint {

        public:
            WorldPoint() = delete;

            /**
            * @brief Constructor
            *
            * @param[in] pointId point Id, specified as an integer.
            * @param[in] location [X, Y, Z] location of the world point, specified as a cv::Vec3d.
            */
            WorldPoint(const int pointId, const cv::Vec3d& location);


            //-------------------------------------------------------------------------
            // Getters
            //-------------------------------------------------------------------------

            /**
            * @brief Get point Id
            *
            * @return point Id, returned as an integer.
            */
            int getPointId() const;

            /**
            * @brief Get point location
            *
            * @return point [X, Y, Z] location, returned as a cv::Vec3d.
            */
            cv::Vec3d getLocation() const;

            /**
            * @brief Get mean viewing direction
            *
            * @return mean viewing direction, returned as a cv::Vec3d.
            */
            cv::Vec3d getViewingDirection() const;

            /**
            * @brief Get viewing distance limits
            *
            * @return viewing distance limits, returned as pair of double values.
            */
            std::pair<double, double> getDistanceLimits() const;

            /**
           * @brief Get the representative view
           *
           * @return Id of the representative view, returned as an integer.
           */
            int getRepresentativeView();

            /**
            * @brief Get the representative feature descriptor
            * 
            * @param[in] kfSet key frame set storing key frames data
            * @return representative feature descriptor, returned as a 1-by-32 cv::Mat.
            */
            cv::Mat getRepresentativeFeatureDescriptor(KeyFrameSet& kfSet);

            /**
            * @brief Get the representative feature point
            * 
            * @param[in] kfSet key frame set storing key frames data
            * @return representative feature point, returned as a cv::KeyPoint.
            */
            cv::KeyPoint getRepresentativeFeaturePoint(KeyFrameSet& kfSet);

            /**
            * @brief Get the views that observe a world point
            *
            * @return views of the world point, returned as a map between view Id and the
            *       index of the corresponding feature point.
            */
            std::unordered_map<int, int> getViewsOfWorldPoint() const;

            /**
            * @brief Check if a world point is valid
            *
            * @return boolean flag
            */
            bool isValid() const;

            //-------------------------------------------------------------------------
            // Setters
            //-------------------------------------------------------------------------

            /**
            * @brief Update world location
            *
            * @param[in] new [X, Y, Z] location, specified as a cv::Vec3d.
            */
            void updateLocation(const cv::Vec3d& loc);

            /**
            * @brief Update distance limits and mean viewing direction
            * 
            * @param[in] kfSet key frame set storing the key frames data
            */
            void updateLimitsAndDirection(KeyFrameSet& kfSet);

            /**
            * @brief Update representative view
            * 
            * @param[in] kfSet key frame set storing the key frames data
            */
            void updateRepresentativeView(KeyFrameSet& kfSet);

            /**
            * @brief Add a 3-D to 2-D correspondence between a view and a world point
            *
            * @param[in] viewId Id of the view that observes the world point, specified as an integer.
            * @param[in] featureIdx index of the feature point in the view that corresponds to the world point,
            *       specified as an integer.
            */
            void addCorrespondence(const int viewId, const int featureIdx);

            /**
            * @brief Remove 3-D to 2-D correspondences between a view and a world point
            *
            * @param[in] viewId Id of view that observes the world point, specified as an integer.
            */
            void removeCorrespondence(const int viewId);

            /**
            * @brief Set a world point to be invalid. The correspondence between the world point and
            *       all the key frames observing it will be deleted.
            *
            */
            void setInvalid();

            /**
            * @brief Destructor
            *
            */
            ~WorldPoint();

        private:
            /**
            * [X, Y, Z] location
            */
            cv::Vec3d location;

            /**
            * Mean viewing direction
            */
            cv::Vec3d viewingDirection;

            /**
            * Distance limits
            */
            std::pair<double, double> distanceLimits;

            /**
            * Representative view Id
            */
            int representativeView;

            /**
            * Id of view and the corresponding feature index
            */
            std::unordered_map<int, int> viewsOfPoint;

            /**
            * Point Id
            */
            int pointId;

            /**
            * Validity flag indicating if a world point is an outlier
            */
            bool validity = true;
        };
    }// namespace vslam
}// namespace vision

#endif // WORLDPOINT_HPP
