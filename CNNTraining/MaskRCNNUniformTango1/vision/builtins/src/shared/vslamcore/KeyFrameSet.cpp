////////////////////////////////////////////////////////////////////////////////
// Object managing view attributes and pairwise connections between views 
// 
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/View.hpp"
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
#else
    #include "View.hpp"
    #include "Connection.hpp"
    #include "MapPointSet.hpp"
    #include "KeyFrameSet.hpp"
#endif

namespace vision {
    namespace vslam {

        std::shared_ptr<View> KeyFrameSet::findView(const int viewId) {
            
            return views.at(viewId);
        }

        std::vector<std::shared_ptr<View>> KeyFrameSet::findView() {
            
            std::vector<std::shared_ptr<View>> allViews;
            allViews.reserve(views.size());
            for (const auto& ele : views) {
                allViews.push_back(ele.second);
            }
            return allViews;
        }

        std::shared_ptr<Connection> KeyFrameSet::findConnection(const int viewId1,
            const int viewId2) {
            
            return findConnection(std::make_pair(viewId1, viewId2));
        }

        std::shared_ptr<Connection> KeyFrameSet::findConnection(const std::pair<int, int> pair) {
            
            return connections.at(pair);
        }

        std::vector<std::shared_ptr<Connection>> KeyFrameSet::findConnection() {
            
            std::vector<std::shared_ptr<Connection>> allConns;
            allConns.reserve(connections.size());
            for (const auto& ele : connections) {
                allConns.push_back(ele.second);
            }
            return allConns;
        }

        std::unordered_map<int, int> KeyFrameSet::findWorldPointsInView(
            const int viewId) {
            
            return findView(viewId)->getWorldPointsInView();
        }

        std::pair<std::vector<int>, std::vector<cv::Matx44d>> KeyFrameSet::getPoses() {
            
            std::vector<cv::Matx44d> viewPoses;
            std::vector<int> viewIds;
            viewPoses.reserve(getNumViews());
            viewIds.reserve(getNumViews());

            for (const auto ele : views) {
                viewPoses.push_back(ele.second->getPose());
                viewIds.push_back(ele.first);
            }

            return std::make_pair(std::move(viewIds), std::move(viewPoses));
        }

        std::vector<int> KeyFrameSet::getViewIds() {

            std::vector<int> allViewIds;
            allViewIds.reserve(views.size());
            for (const auto& ele : views) {
                allViewIds.push_back(ele.first);
            }
            return allViewIds;
        }

        int KeyFrameSet::getNumViews() {
            
            return static_cast<int>(views.size());
        }

        int KeyFrameSet::getNumConnections() {
            
            return static_cast<int>(connections.size());
        }

        std::vector<int> KeyFrameSet::getKeyFrameIndex() const {
            return this->keyFrameIndex;
        }

        int KeyFrameSet::getFrameIndexByViewId(const int viewId) {
            auto kf = this->findView(viewId);
            int frameIndex = -1;
            if(kf != nullptr) {
                frameIndex = kf->getFrameIndex();
            }
            return frameIndex;
        }

        void KeyFrameSet::connectedViews(
            const int viewId,
            std::vector<int>& connViews,
            std::vector<int>& distances,
            const int maxDistance,
            const int minNumMatches) {
            

            assert(maxDistance == 1 || maxDistance == 2);

            if (views.find(viewId) == views.end())
                return;

            connViews.clear();
            distances.clear();

            // Find distance-1 views
            std::vector<int> dist1ViewIds = findView(viewId)->getConnectedViews(minNumMatches);

            connViews.insert(connViews.begin(), dist1ViewIds.begin(), dist1ViewIds.end());
            distances.resize(connViews.size(), 1); // Filled with 1

            if (maxDistance == 2) {
                isVisited[viewId] = true;
                for (const auto& Id1 : dist1ViewIds) {
                    isVisited[Id1] = true;
                }

                // For each distance-1 view, find its connected views which are potentially distance-2 views
                for (const auto& Id1 : dist1ViewIds) {
                    std::vector<int> connectedViewIds = findView(Id1)->getConnectedViews(minNumMatches);

                    for (const auto& Id2 : connectedViewIds) {
                        if (!isVisited[Id2]) { // If the view is not visited, it is a distance-2 view
                            isVisited[Id2] = true;
                            connViews.push_back(Id2);
                        }
                    }
                }

                distances.resize(connViews.size(), 2); // fill with 2

                // Set flag back
                for (const auto& Id : connViews) {
                    isVisited[Id] = false;
                }

                isVisited[viewId] = false;
            }
        }

        int KeyFrameSet::getMaxViewId() {
            
            return maxViewId;
        }


        std::unordered_map<int, std::pair< std::vector<double>, std::vector<double> >> KeyFrameSet::getIMUMeasurements() {

            std::unordered_map<int, std::pair< std::vector<double>, std::vector<double> >> allIMUMeasurements;

            for (const auto& view : views) {
                allIMUMeasurements[view.first] = view.second->getIMU();
            }


            return allIMUMeasurements;
        }

        void KeyFrameSet::addView(std::shared_ptr<View> view) {
            
            int viewId = view->getViewId();

            this->keyFrameIndex.push_back(view->getFrameIndex());
            
            views[viewId] = std::move(view);
            isVisited[viewId] = false;

            if (viewId > maxViewId)
                maxViewId = viewId;
        }

        void KeyFrameSet::addConnection(std::shared_ptr<Connection> conn) {
            
            auto pair = conn->getViewIdPair();
            int numMatches = static_cast<int>(conn->getMatches().size());
            findView(pair.first)->addConnectedView(pair.second, numMatches);
            findView(pair.second)->addConnectedView(pair.first, numMatches);
            connections[pair] = std::move(conn);
        }

        void KeyFrameSet::updateViewPose(const int viewId,
            const cv::Matx33d& ori,
            const cv::Vec3d& loc) {

            auto view = findView(viewId);
            view->setOrientation(ori);
            view->setLocation(loc);
        }

        void KeyFrameSet::updateVelocityAndBias(const int viewId,
            const std::vector<double>& velo,
            const std::vector<double>& bs) {

            auto view = findView(viewId);
            view->setVelocityAndBias(velo, bs);
        }

        void KeyFrameSet::updateConnectionPose(const int viewId1,
            const int viewId2,
            const cv::Matx33d& rot,
            const cv::Vec3d& trans) {
            

            auto conn = findConnection(viewId1, viewId2);
            conn->setRelRotation(rot);
            conn->setRelTranslation(trans);
        }

        void KeyFrameSet::addConnectionMatches(const int viewId1,
            const int viewId2,
            const std::vector<std::pair<int, int>> matches) {
            
            findConnection(viewId1, viewId2)->addMatches(matches);
        }

        void KeyFrameSet::deleteView(const int viewId) {
            
            if (views.find(viewId) == views.end())
                return;

            // First delete the connection associated with the view 
            std::vector<int> connViews = findView(viewId)->getConnectedViews();
            for (const auto& viewId2 : connViews) {
                deleteConnection(viewId, viewId2);
            }

            // Then delete the view from the set
            views.erase(viewId);
            isVisited.erase(viewId);
        }

        void KeyFrameSet::deleteConnection(const int viewId1, const int viewId2) {
            
            auto pair1 = std::make_pair(viewId1, viewId2), pair2 = std::make_pair(viewId2, viewId1);
            if (connections.find(pair1) != connections.end()) {
                connections.erase(pair1);
            }

            if (connections.find(pair2) != connections.end()) {
                connections.erase(pair2);
            }

            findView(viewId1)->removeConnectedView(viewId2);
            findView(viewId2)->removeConnectedView(viewId1);
        }
    } // namespace vslam
} // namespace vision
