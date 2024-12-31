////////////////////////////////////////////////////////////////////////////////
// Function to correct loop using the given the loop candidates
// 
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#include <vector>
#include <iostream>

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/correctLoop.hpp"
    #include "vslamcore/optimizePoseGraph.hpp"
    #include "vslamcore/Connection.hpp"
    #include "vslamcore/converter.hpp"
#else
    #include "correctLoop.hpp"
    #include "optimizePoseGraph.hpp"
    #include "Connection.hpp"
    #include "converter.hpp"
#endif

namespace vision {
    namespace vslam {

        bool correctLoop(MapPointSet& mpSet, 
                         KeyFrameSet& kfSet,
                         LoopClosureDatabase& database,
                         const cv::Matx33d& intrinsics,
                         const int currKeyFrameId,
                         const std::vector<cv::Mat>& currFeaturesVec,
                         const std::vector<std::vector<int>>& loopKeyFrameIds,
                         const Configuration& config) {

            bool isLoopClosed{ false };
            std::vector<std::pair<std::shared_ptr<Connection>, double>> loopConnections;

            // Add loop connections
            database.addLoopConnections(mpSet, kfSet, intrinsics, currKeyFrameId, loopKeyFrameIds, config, loopConnections);
            
            isLoopClosed = !loopConnections.empty();
            if (isLoopClosed) {

                // Optimize graph to get scales
                std::vector<double> optimScales;
                auto viewIdsAndPosesOld(kfSet.getPoses());
                auto viewIdsAndPosesNew(kfSet.getPoses());
                auto odometryConnections(kfSet.findConnection());

                if (config.isMono) {
                    optimizePoseGraph(
                        viewIdsAndPosesNew,
                        odometryConnections,
                        loopConnections,
                        optimScales,
                        config);
                }
                else {
                    std::vector<std::shared_ptr<Connection>> loopConnsStereo; 
                    loopConnsStereo.reserve(loopConnections.size());
                    for (const auto& conn : loopConnections) {
                        loopConnsStereo.push_back(conn.first); // Remove the scale
                    }

                    optimizePoseGraph(
                        viewIdsAndPosesNew,
                        odometryConnections,
                        loopConnsStereo,
                        config);
                }

                // Update view poses
                auto allViewIds = viewIdsAndPosesNew.first; // sorted
                cv::Matx33d rot;
                cv::Vec3d tran;
                for (int i = 0; i < static_cast<int>(allViewIds.size()); i++) {
                    pose2rt(viewIdsAndPosesNew.second[i], rot, tran);
                    kfSet.updateViewPose(allViewIds[i], rot, tran);
                }

                // Update map points
                cv::Matx44d tform;
                for (const auto& points : mpSet.getAllWorldPoints()) {
                    const int pointId = points.first;
                    if (mpSet.getWorldPoint(pointId)->isValid()) {
                        auto locOld3d(mpSet.getLocation(pointId));
                        cv::Vec4d locOld4d(locOld3d[0], locOld3d[1], locOld3d[2], 1.0);

                        // Get the representative view of the map point
                        int viewIdRepr = mpSet.getWorldPoint(pointId)->getRepresentativeView();
                        auto pos = std::distance(allViewIds.begin(), std::find(allViewIds.begin(), allViewIds.end(), viewIdRepr));

                        tform = viewIdsAndPosesNew.second[pos] * viewIdsAndPosesOld.second[pos].inv();
                        cv::Vec4d locNew4d(tform * locOld4d);
                        mpSet.updateWorldPoint(pointId, { locNew4d[0], locNew4d[1], locNew4d[2] });
                        mpSet.updateLimitsAndDirection(pointId, kfSet);
                    }
                }
            }

            return isLoopClosed;
        }

    } // namespace vslam
} // namespace vision
