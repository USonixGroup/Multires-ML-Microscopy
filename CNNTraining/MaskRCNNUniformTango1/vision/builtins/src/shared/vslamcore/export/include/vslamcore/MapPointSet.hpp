////////////////////////////////////////////////////////////////////////////////
// Object storing correspondences between 3-D world points and 2-D image points 
// across camera views
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef MAPPOINTS_HPP
#define MAPPOINTS_HPP

#include <memory>
#include <vector>
#include <unordered_map>
#include <limits>
#include <mutex>

#include "opencv2/core.hpp"

namespace vision {
    namespace vslam {
        class WorldPoint;
        class KeyFrameSet;

        class MapPointSet {

        public:
            /**
            * @brief Constructor
            */
            MapPointSet() = default;

            //-------------------------------------------------------------------------
            // Getters
            //-------------------------------------------------------------------------

            /**
            * @brief Find views that observe a world point 
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @return a map between views and feature index.
            */ 
            std::unordered_map<int, int> findViewsOfWorldPoint(const int pointId);

            /**
            * @brief Find a world point by Id
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @return a pointer to WorldPoint.
            */ 
            std::shared_ptr<WorldPoint> getWorldPoint(const int pointId);
            
  
            /**
            * @brief Get all world points and their IDs 
            * 
            * @return a map of all the point Ids and pointers to all the world points
            */ 
            std::unordered_map<int, std::shared_ptr<WorldPoint>> getAllWorldPoints();

            /**
            * @brief Get location of a world point
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @return location of the world point, returned as a cv::Vec3d.
            */ 
            cv::Vec3d getLocation(const int pointId);

            /**
            * @brief Get location of all world points
            * 
            * @return locations of all world point, returned as a vector of cv::Vec3d.
            */ 
            std::vector<cv::Vec3d> getLocation();

            /**
            * @brief Get number of world points
            * 
            * @return number of world point, returned as an integer.
            */ 
            int getLastPointId();

            /**
            * @brief Get Id of the last added point 
            * 
            * @return Id of the last world point, returned as an integer.
            */ 
            int getCount();

            /**
            * @brief Check if a map point is a local map point 
            * 
            * @return flag indicating if a map point is in the local map
            */
            bool isLocal(const int pointId);

            /**
            * @brief Lock this object.
            */
            void lock() {
                internalMtx.lock();
            }
        
            /**
            * @brief Unlock this object.
            */
            void unlock() {
                internalMtx.unlock();
            }
        
            /**
            * @brief Try locking this object.
            *
            * @return true if locking successful, otherwise false.
            */
            bool tryLock() {
                return internalMtx.try_lock();
            }

            /**
            * @brief Add a world point
            * 
            * @param[in] point new world point, specified as a pointer to WorldPoint.
            */
            void addWorldPoint(std::shared_ptr<WorldPoint> point);

            /**
            * @brief Update a world point
            * 
            * @param[in] loc new world location, specified as a cv::Vec3d.
            */
            void updateWorldPoint(const int pointId, const cv::Vec3d& loc);

            /**
            * @brief Remove a world point
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @param[in] kfSet a keyFrameSet object containing views that observe the world point.
            */
            void removeWorldPoint(const int pointId, KeyFrameSet& kfSet);

            /**
            * @brief Add 3-D to 2-D correspondences between a world point and a view
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @param[in] viewId Id of the view observing the world point, specified as an integer.
            * @param[in] featureIdx index of the feature point corresponding to the world point,
            *           specified as an integer.
            * @param[in] kfSet a keyFrameSet object containing views that observe the world point.
            */
            void addCorrespondence(const int pointId, const int viewId, const int featureIdx, KeyFrameSet& kfSet);

            /**
            * @brief Remove 3-D to 2-D correspondences between a world point and a view
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @param[in] viewId Id of the view observing the world point, specified as an integer.
            * @param[in] kfSet a keyFrameSet object containing views that observe the world point.
            */
            void removeCorrespondence(const int pointId, const int viewId, KeyFrameSet& kfSet);

            /**
            * @brief Remove all 3-D to 2-D correspondences of a world point 
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            */
            void removeCorrespondence(const int pointId, KeyFrameSet& kfSet);
            
            /**
            * @brief Updates the distance limits and viewing direction of a world point
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @param[in] kfSet a keyFrameSet object containing views that observe the world point.
            */
            void updateLimitsAndDirection(const int pointId, KeyFrameSet& kfSet);

            /**
            * @brief Updates the representative view ID and corresponding feature index of a world point 
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @param[in] kfSet a keyFrameSet object containing views that observe the world point.
            */
            void updateRepresentativeView(const int pointId, KeyFrameSet& kfSet);

            /**
            * @brief Set if a map point is a local map point 
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @param[in] flag a boolean value indicating if a map point is in the local map.
            */
            void setLocal(const int pointId, const bool flag);

            /**
            * @brief Set a map point to be invalid. The correspondence between the world point and
            *       the key frames observing it are deleted.
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @param[in] kfSet a keyFrameSet object containing views that observe the world point.
            */
            void setInvalid(const int pointId, KeyFrameSet& kfSet);

            /**
            * @brief Destructor
            */
            ~MapPointSet() = default;

        private:
            /**
            * Point Id and pointer to world point
            */
            std::unordered_map<int, std::shared_ptr<WorldPoint>> worldPoints;

            /*
            * Flags used for querying local map points
            */
            std::unordered_map<int, bool> isInLocalMap;

            /*
            * Id of the last added point 
            */
            int lastPointId;

            /**
            * Mutex for thread safety
            */
            std::recursive_mutex internalMtx;

            /**
            * @brief Check if a world point exists.
            * 
            * @param[in] pointId Id of world point, specified as an integer.
            * @return true if the point exists, otherwise false.
            */ 
            bool isPointInSet(const int pointId);
        };
    }// namespace vslam
}// namespace vision

#endif // MAPPOINTS_HPP

