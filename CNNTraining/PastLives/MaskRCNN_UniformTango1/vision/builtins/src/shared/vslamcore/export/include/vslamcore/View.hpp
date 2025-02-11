////////////////////////////////////////////////////////////////////////////////
// View class storing camera pose, feature descriptors, and key points
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef VIEW_HPP
#define VIEW_HPP

#include <vector>

#include "WorldPoint.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"

namespace vision { 
    namespace vslam {

        class WorldPoint;
        class View {

        public:
            View() = delete;

            /**
            * @brief Constructor
            *
            * @param[in] Id view Id, specified as an integer.
            * @param[in] pts feature points, specified as an M-element vector of cv::KeyPoint.
            * @param[in] fs feature descriptors, specified as an M-by-32 cv::Mat.
            * @param[in] ori orientation of the camera, specified as a cv::Matx33d.
            * @param[in] loc location of the camera, specified as a cv::Vec3d.
            * @param[in] fIndex corresponding frame index, specified as an integer.
            */
            View(const int Id,
                const std::vector<cv::KeyPoint>& pts,
                const cv::Mat& fs,
                const cv::Matx33d& ori = cv::Matx33d::eye(),
                const cv::Vec3d& loc = cv::Vec3d(0, 0, 0),
                const int fIndex = -1               
            );

            //-------------------------------------------------------------------------
            // Getters
            //-------------------------------------------------------------------------

            /**
            * @brief Get View Id
            *
            * @return View Id, returned as an integer.
            */
            int getViewId() const;

            /**
            * @brief Get frame index
            *
            * @return frame index, returned as an integer.
            */
            int getFrameIndex() const;

            /**
            * @brief Get camera pose
            *
            * @return Camera pose, returned as a cv::Matx44d.
            */
            cv::Matx44d getPose() const;

            /**
            * @brief Get view velocity and Bias
            *
            * @return View Velocity and Bias, returned as a std::pair<std::vector<double>,std::vector<double>>.
            */
            std::pair<std::vector<double>, std::vector<double>> getVelocityAndBias();

            /**
            * @brief Get camera orientation
            *
            * @return Camera orientation, returned as a cv::Matx33d.
            */
            cv::Matx33d getOrientation() const;

            /**
            * @brief Get camera location
            *
            * @return Camera location, returned as a cv::Vec3d.
            */
            cv::Vec3d getLocation() const;

            /**
            * @brief Get all the feature points
            *
            * @return Feature points, returned as a vector of cv::KeyPoint.
            */
            std::vector<cv::KeyPoint> getFeaturePoints() const;

            /**
            * @brief Get feature points by indices
            *
            * @param[in] idx, a vector of integers representing the indices.
            * @return Feature points, returned as a vector of cv::KeyPoint.
            */
            std::vector<cv::KeyPoint> getFeaturePoints(const std::vector<int>& idx) const;

            /**
            * @brief Get a feature point by index
            *
            * @param[in] idx, an integer representing the index.
            * @return Feature point, returned as a cv::KeyPoint.
            */
            cv::KeyPoint getFeaturePoints(const int idx) const;

            /**
            * @brief Get all the feature descriptors
            *
            * @return Feature descriptors, returned as a cv::Mat.
            */
            cv::Mat getFeatureDescriptors() const;

            /**
            * @brief Get feature descriptors by indices
            *
            * @param[in] idx, a vector of integers representing the indices.
            * @return Feature descriptors, returned as a cv::Mat.
            */
            cv::Mat getFeatureDescriptors(const std::vector<int>& idx);

            /**
            * @brief Get a feature descriptor by index
            *
            * @param[in] idx, an integer representing the index.
            * @return Feature descriptor, returned as a 1-by-32 cv::Mat.
            */
            cv::Mat getFeatureDescriptors(const int idx);

            /**
            * @brief Get connected views
            *
            * @param[in] minNumMatches minimum number of matched feature points between two views
            *           for the views to be considered as connected, specified as an integer.
            * @return Connected views, returned as a vector of view Ids.
            */
            std::vector<int> getConnectedViews(const int minNumMatches = 0) const;

            /**
            * @brief Check if the given view is connected to the object view.
            *
            * @param[in] viewId Id of the view, specified as an integer.
            * @param[in] minNumMatches Minimum number of matched feature 
            *            points for a connection to be valid.
            * @return true if given view is connected, otherwise false.
            */
            bool isConnectedView(const int viewId, const int minNumMatches = 0) const;

            /**
            * @brief Get connected world points in view
            *
            * @return Ids of world points in the view and the associated feature indices.
            */
            std::unordered_map<int, int> getWorldPointsInView() const;

            /**
            * @brief Get IMU measurements of the view
            *
            */
            std::pair<std::vector<double>, std::vector<double>> getIMU() const;

            //-------------------------------------------------------------------------
            // Setters
            //-------------------------------------------------------------------------

            /**
            * @brief Set camera pose
            *
            * @param[in] ori camera orientation, specified as a cv::Matx33d.
            * @param[in] loc camera location, specified as a cv::Vec3d.
            */
            void setPose(const cv::Matx33d& ori, const cv::Vec3d& loc);

            /**
            * @brief Set camera orientation
            *
            * @param[in] ori camera orientation, specified as a cv::Matx33d.
            */
            void setOrientation(const cv::Matx33d& ori);

            /**
            * @brief Set camera location
            *
            * @param[in] loc camera location, specified as a cv::Vec3d.
            */
            void setLocation(const cv::Vec3d& loc);

            /**
            * @brief Set view velocity and bias
            *
            * @param[in] view velocity and bias, specified both as a std::vector<double>.
            */
            void setVelocityAndBias(const std::vector<double>& velo, const std::vector<double>& bs);

            /**
            * @brief Set IMU measurements of the view
            *
            * @param[in] gyroscope and accelerometer measurements, specified both as a std::vector<double>.
            */
            void setIMU(const std::vector<double>& gyro, const std::vector<double>& accel);

            //-------------------------------------------------------------------------
            // Co-visibility
            //-------------------------------------------------------------------------

            /**
            * @brief Add a connected view
            *
            * @param[in] viewId Id of a connected view.
            * @param[in] numMatches number of matched features, specified by an integer.
            */
            void addConnectedView(const int viewId, const int numMatches = 0);

            /**
            * @brief Update a connected view
            *
            * @param[in] viewId Id of a connected view.
            * @param[in] numMatches number of matched features, specified by an integer.
            */
            void updateConnectedView(const int viewId, const int numMatches = 1);


            /**
            * @brief Delete a connected view
            *
            * @param[in] viewId Id of a connected view.
            */
            void removeConnectedView(const int viewId);

            /**
            * @brief Add a world point observed in the view
            *
            * @param[in] pointId Id of a world point observed in the view.
            * @param[in] featureIdx index of feature point corresponding to the world point, specified by an integer.
            */
            void addCorrespondence(const int pointId, const int featureIdx);

            /**
            * @brief Get the feature index correspondence associated with the world.
            *
            * @param[in] pointId Id of a world point observed in the view.
            * @return Index of feature point corresponding to the world point, specified by an integer.
            */
            int getCorrespondence(const int pointId);

            /**
            * @brief Remove a world point observed in the view
            *
            * @param[in] pointId Id of a world point observed in the view.
            */
            void removeCorrespondence(const int pointId);

            /**
            * @brief Destructor
            */
            ~View();

        private:

            /**
            * View Id
            */
            int viewId;

            /**
            * Frame Index
            */
            int frameIndex;

            /**
            * Camera orientation and location
            */
            cv::Matx33d orientation;
            cv::Vec3d location;

            /**
            * View velocity and bias
            */
            std::vector<double> velocity{0.0, 0.0, 0.0};
            std::vector<double> bias{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            /**
            * View gyroscope and accelerometer measurements
            */
            std::pair< std::vector<double>, std::vector<double> > imuViewMeasurements;

            /**
            * Feature points and descriptors
            */
            const std::vector<cv::KeyPoint> featurePoints;
            const cv::Mat featureDescriptors;

            /**
            * Ids of connected views and number of matches
            */
            std::unordered_map<int, int> neighborViewsWithMatches;

            /**
            * Ids of world points observed in the view and the indices of associated feature points
            */
            std::unordered_map<int, int> worldPointsInView;

        };
    }// namespace vSLAM
}// namespace vision

#endif // VIEW_HPP
