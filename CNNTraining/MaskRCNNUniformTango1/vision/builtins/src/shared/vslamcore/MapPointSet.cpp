////////////////////////////////////////////////////////////////////////////////
// Object storing correspondences between 3-D world points and 2-D image points 
// across camera views
// 
// Copyright 2022-23 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/WorldPoint.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
#else
    #include "MapPointSet.hpp"
    #include "WorldPoint.hpp"
    #include "KeyFrameSet.hpp"
#endif

namespace vision {
    namespace vslam {

        std::shared_ptr<WorldPoint> MapPointSet::getWorldPoint(const int pointId) {
            
            if (!this->isPointInSet(pointId)) {
                throw "Invalid point Id";    
            }
            return worldPoints.at(pointId);
        }
        

        std::unordered_map<int, std::shared_ptr<WorldPoint>> MapPointSet::getAllWorldPoints() {
            return worldPoints;
        }

        bool MapPointSet::isPointInSet(const int pointId) {
            
            return worldPoints.find(pointId) != worldPoints.end();
        }

        std::unordered_map<int, int> MapPointSet::findViewsOfWorldPoint(const int pointId) {
            
            if (!this->isPointInSet(pointId))
                return std::unordered_map<int, int>();

            return getWorldPoint(pointId)->getViewsOfWorldPoint();
        }

        cv::Vec3d MapPointSet::getLocation(const int pointId) {
            
            if (!this->isPointInSet(pointId))
                return cv::Vec3d();

            return getWorldPoint(pointId)->getLocation();
        }

        std::vector<cv::Vec3d> MapPointSet::getLocation() {
            
            std::vector<cv::Vec3d> locations;
            locations.reserve(worldPoints.size());
            for (const auto& ele : worldPoints) {
                if (ele.second->isValid())
                    locations.push_back(ele.second->getLocation());
            }
            return locations;
        }

        int MapPointSet::getLastPointId() {
            
            return lastPointId;
        }

        int MapPointSet::getCount() {
            return static_cast<int>(worldPoints.size());
        }

        bool MapPointSet::isLocal(const int pointId) {
            
            return isInLocalMap.at(pointId);
        }

        void MapPointSet::addWorldPoint(std::shared_ptr<WorldPoint> point) {
            

            int pointId = point->getPointId();

            worldPoints[pointId] = std::move(point);
            lastPointId = pointId;
            isInLocalMap[pointId] = false;
        }

        void MapPointSet::updateWorldPoint(const int pointId, const cv::Vec3d& loc) {
            
            getWorldPoint(pointId)->updateLocation(loc);
        }

        void MapPointSet::removeWorldPoint(const int pointId, KeyFrameSet& kfSet) {
            
            removeCorrespondence(pointId, kfSet);
            isInLocalMap.erase(pointId);
            worldPoints.erase(pointId);
        }

        void MapPointSet::addCorrespondence(const int pointId, const int viewId, const int featureIdx, KeyFrameSet& kfSet) {
            
            getWorldPoint(pointId)->addCorrespondence(viewId, featureIdx);
            kfSet.findView(viewId)->addCorrespondence(pointId, featureIdx);
        }

        void MapPointSet::removeCorrespondence(const int pointId, const int viewId, KeyFrameSet& kfSet) {
            
            getWorldPoint(pointId)->removeCorrespondence(viewId);
            kfSet.findView(viewId)->removeCorrespondence(pointId);
        }

        void MapPointSet::removeCorrespondence(const int pointId, KeyFrameSet& kfSet) {
            
            auto wp = getWorldPoint(pointId);
            std::unordered_map<int, int> viewsOfPoint = wp->getViewsOfWorldPoint();
            for (const auto& view : viewsOfPoint) {
                wp->removeCorrespondence(view.first);
                kfSet.findView(view.first)->removeCorrespondence(pointId);
            }
        }

        void MapPointSet::updateLimitsAndDirection(const int pointId, KeyFrameSet& kfSet) {
            
            getWorldPoint(pointId)->updateLimitsAndDirection(kfSet);
        }

        void MapPointSet::updateRepresentativeView(const int pointId, KeyFrameSet& kfSet) {
            
            getWorldPoint(pointId)->updateRepresentativeView(kfSet);
        }

        void MapPointSet::setLocal(const int pointId, const bool flag) {
            
            isInLocalMap[pointId] = flag;
        }

        void MapPointSet::setInvalid(const int pointId, KeyFrameSet& kfSet) {
            
            auto wp = getWorldPoint(pointId);
            std::unordered_map<int, int> viewsOfPoint = wp->getViewsOfWorldPoint();
            for (const auto& view : viewsOfPoint) {
                kfSet.findView(view.first)->removeCorrespondence(pointId);
            }
            wp->setInvalid();
        }
    } // namespace vslam
} // namespace vision