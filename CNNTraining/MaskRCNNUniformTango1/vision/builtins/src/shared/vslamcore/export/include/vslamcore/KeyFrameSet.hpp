////////////////////////////////////////////////////////////////////////////////
// Object managing view attributes and pairwise connections between views 
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifndef KEYFRAMESET_HPP
#define KEYFRAMESET_HPP

#include <memory>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <mutex>

#include "opencv2/core.hpp"
#include "SafeX.hpp"

namespace vision {
    namespace vslam {
        class View;
        class Connection;
        class MapPointSet;

        struct pairHash {
            // Uses Cantor Pairing to uniquely hash a pair of integers
            size_t operator()(const std::pair<int, int>& p) const {
                auto hashKey = (p.first + p.second) * (p.first + p.second + 1) / 2 + p.second;
                return hashKey;
            }
        };

        class KeyFrameSet {

        public:
            /**
            * @brief Constructor
            */
            KeyFrameSet() = default;

            //-------------------------------------------------------------------------
            // Getters
            //-------------------------------------------------------------------------

            /**
            * @brief Find a view by Id
            *
            * @param[in] viewId Id of the view, specified as an integer.
            * @return a pointer to the view.
            */
            std::shared_ptr<View> findView(const int viewId);

            /**
            * @brief Get all the views
            *
            * @return a vectors of pointers to the views.
            */
            std::vector<std::shared_ptr<View>> findView();

            /**
            * @brief Find a connection by Id
            *
            * @param[in] viewId1 Id of view1 in the connection, specified as an integer.
            * @param[in] viewId2 Id of view2 in the connection, specified as an integer.
            * @return a pointer to the connection.
            */
            std::shared_ptr<Connection> findConnection(const int viewId1, const int viewId2);

            /**
            * @brief Find a connection by Id
            *
            * @param[in] pairId Id of view1 and view2  in the connection, specified as a pair of integers.
            * @return a pointer to the connection.
            */
            std::shared_ptr<Connection> findConnection(const std::pair<int, int> pairId);

            
            /**
            * @brief Get all the connections
            *
            * @return a vectors of pointers to the connections.
            */
            std::vector<std::shared_ptr<Connection>> findConnection();

            /**
            * @brief Find world points observed in a view
            *
            * @param[in] viewId Id of the view, specified as an integer.
            * @return a map between world points and the corresponding feature index.
            */
            std::unordered_map<int, int> findWorldPointsInView(const int viewId);

            /**
            * @brief Find connected views of a view
            *
            * @param[in] viewId Id of the view, specified as an integer.
            * @param[out] views connected views, returned as a vector of view Ids.
            * @param[out] distances distances associated with the connected views, returned as a vector of integers.
            * @param[in] maxDistance maximum distance from the query view to the connected views, specified as an integer.
            * @param[in] minNumMatches minimum number of matched features between two views for the views to be considered
            *           as connected, specified as an integer.
            */
            void connectedViews(
                const int viewId,
                std::vector<int>& viewIds,
                std::vector<int>& distances,
                const int maxDistance = 1,
                const int minNumMatches = 0);

            /**
            * @brief Get poses of all views
            *
            * @return a pair of vectors containing the view Ids and the view poses, respectively.
            */
            std::pair<std::vector<int>, std::vector<cv::Matx44d>> getPoses();

            /**
            * @brief Get all the views Ids 
            *
            * @return a vector containing the view Ids.
            */
            std::vector<int> getViewIds();

            /**
            * @brief Get number of views
            *
            * @return number of views, returned as an integer.
            */
            int getNumViews();

            /**
            * @brief Get number of connections
            *
            * @return number of connections, returned as an integer.
            */
            int getNumConnections();

            /**
            * @brief Get the maximum view Id
            *
            * @return maximum view Id, returned as an integer.
            */
            int getMaxViewId();

            /**
            * @brief Get frame index associated with view Id
            *
            * @return frame index, returned as an integer.
            */
            int getFrameIndexByViewId(const int viewId);

            /**
            * @brief Get all the IMU measurements
            *
            * @return all IMU measurements, returned as a map (KF-viewIds), of pairs (frame) of vectors (IMU).
            */
            std::unordered_map<int, std::pair< std::vector<double>, std::vector<double> >> getIMUMeasurements();

            /**
            * @brief Get key frame index (not viewId)
            *
            * @return list of key frame indices.
            */
            std::vector<int> getKeyFrameIndex() const;

            //-------------------------------------------------------------------------
            // Setters
            //-------------------------------------------------------------------------

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
            * @brief Add a view
            *
            * @param[in] view view to be added, specified as a pointer to View.
            */
            void addView(std::shared_ptr<View> view);

            /**
            * @brief Add a connection
            *
            * @param[in] conn connection to be added, specified as a pointer to Connection.
            */
            void addConnection(std::shared_ptr<Connection> conn);

            /**
            * @brief Update pose of a view
            *
            * @param[in] viewId view Id, specified as an integer.
            * @param[in] ori new orientation, specified as a cv::Matx33d.
            * @param[in] loc new location, specified as a cv::Vec3d.
            */
            void updateViewPose(const int viewId,
                const cv::Matx33d& ori,
                const cv::Vec3d& loc);

            /**
            * @brief Update velocity and bias of a view
            *
            * @param[in] viewId view Id, specified as an integer.
            * @param[in] velo new velocity, specified as a std::vector<double>.
            * @param[in] bs new bias, specified as a std::vector<double>.
            */
            void updateVelocityAndBias(const int viewId,
                const std::vector<double>& velo,
                const std::vector<double>& bs);

            /**
            * @brief Update relative pose of a connection
            *
            * @param[in] viewId1 Id of view1, specified as an integer.
            * @param[in] viewId1 Id of view2, specified as an integer.
            * @param[in] rot new relative rotation, specified as a cv::Matx33d.
            * @param[in] trans new relative translation, specified as a cv::Vec3d.
            */
            void updateConnectionPose(const int viewId1,
                const int viewId2,
                const cv::Matx33d& rot,
                const cv::Vec3d& trans);

            /**
            * @brief Add matched feature pairs in a connection
            *
            * @param[in] viewId1 Id of view1, specified as an integer.
            * @param[in] viewId1 Id of view2, specified as an integer.
            * @param[in] matches matched feature pairs specified as a vector of integer pairs.
            */
            void addConnectionMatches(const int viewId1,
                const int viewId2,
                const std::vector<std::pair<int, int>> matches);

            /**
            * @brief Delete a view
            *
            * @param[in] viewId Id of the view to be deleted, specified as an integer.
            */
            void deleteView(const int viewId);

            /**
            * @brief Delete a connection
            *
            * @param[in] viewId1 Id of view1 in the connection, specified as an integer.
            * @param[in] viewId2 Id of view2 in the connection, specified as an integer.
            */
            void deleteConnection(const int viewId1, const int viewId2);

            /**
            * @brief Destructor
            */
            ~KeyFrameSet() = default;
        private:
            /**
            * View Id and views
            */
            std::map<int, std::shared_ptr<View>> views; 

            /**
            * View Ids and connections
            */
            std::unordered_map<std::pair<int, int>, std::shared_ptr<Connection>, pairHash> connections;

            /**
            * Maximum view Id
            */
            int maxViewId = 0;

            /**
            * Flags used for computing distance-N views using DFS
            */
            std::unordered_map<int, bool> isVisited;

            /**
            * List of key frame indices
            */
            std::vector<int> keyFrameIndex;

            /**
            * Mutex for thread safety
            */
            std::recursive_mutex internalMtx;
        };
    }// namespace vslam
}// namespace vision

#endif // KEYFRAMESET_HPP
