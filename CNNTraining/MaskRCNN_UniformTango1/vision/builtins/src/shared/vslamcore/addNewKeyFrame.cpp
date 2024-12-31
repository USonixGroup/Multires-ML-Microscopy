///////////////////////////////////////////////////////////////////////////
// Add a new key frame to the data management object
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/View.hpp"
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/WorldPoint.hpp"
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
    #include "vslamcore/addNewKeyFrame.hpp" 
    #include "vslamcore/converter.hpp"
#else
    #include "View.hpp"
    #include "Connection.hpp"
    #include "WorldPoint.hpp"
    #include "MapPointSet.hpp"
    #include "KeyFrameSet.hpp"
    #include "addNewKeyFrame.hpp" 
    #include "converter.hpp"
#endif

namespace vision {
    namespace vslam {

        void addNewKeyFrame(
            MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const QueueInstanceBase& qObj,
            const Configuration& config) {

            // Add view
            cv::Matx33d absPose_R;
            cv::Vec3d absPose_t;
            extr2pose(qObj.currExtrinsics_R, qObj.currExtrinsics_t, absPose_R, absPose_t);
            std::shared_ptr<View> currView = std::make_shared<View>(qObj.currKeyFrameId, std::move(qObj.currPoints), qObj.currFeatures, absPose_R, absPose_t, qObj.currFrameIndex);
            kfSet.addView(currView);

            // Find matches of connected views
            std::unordered_map<int, std::vector<std::pair<int, int>>> viewsAndMatches;
            viewsAndMatches.reserve(qObj.localKeyFrameIds.size());

            config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NumTrackedMapPointsAddedFrame", {std::to_string(static_cast<int>(qObj.trackedMapPointIds.size()))});

            for (std::size_t i = 0; i != qObj.trackedMapPointIds.size(); ++i) {
                auto views = mpSet.getWorldPoint(qObj.trackedMapPointIds[i])->getViewsOfWorldPoint();
                for (const auto& view : views) {
                    viewsAndMatches[view.first].push_back(std::make_pair(view.second, qObj.trackedFeatureIndices[i]));

                    // Add correspondence after querying the views of the world point
                    mpSet.addCorrespondence(qObj.trackedMapPointIds[i], qObj.currKeyFrameId, qObj.trackedFeatureIndices[i], kfSet);
                }
            }

            // Add connections
            cv::Matx33d preOrientationT, relPose_R;
            cv::Vec3d relPose_t;
            for (const auto& ele : viewsAndMatches) {
                if (static_cast<int>(ele.second.size()) >= config.baseParams.minNumMatchesNewConnection) {// Skip weak connections
                    auto preKeyFrameId = ele.first;
                    auto preKF = kfSet.findView(preKeyFrameId);
                    int viewId = preKF->getFrameIndex();
                    
                    config.loggerPtr->logMessageV2("vision::vslamVerboseCpp::NewConnectionWithPoints",
                                                    {std::to_string(qObj.currFrameIndex + 1), std::to_string(viewId + 1), // 1-based indexing
                                                     std::to_string(static_cast<int>(ele.second.size()))});
                    
                    // Compute relative pose
                    preOrientationT = preKF->getOrientation().t();
                    relPose_R = preOrientationT * absPose_R;
                    relPose_t = preOrientationT * (absPose_t - preKF->getLocation());
                    std::shared_ptr<Connection> conn = std::make_shared<Connection>(preKeyFrameId, qObj.currKeyFrameId, relPose_R, relPose_t, ele.second);
                    kfSet.addConnection(conn);
                }
            }
        }
    }
}