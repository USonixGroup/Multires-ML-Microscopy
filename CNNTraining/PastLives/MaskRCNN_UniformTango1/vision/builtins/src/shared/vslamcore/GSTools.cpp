////////////////////////////////////////////////////////////////////////////////
// Tools for handeling scale and gravity alignement estimation
// 
// Copyright 2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////


#ifdef BUILDING_LIBMWVSLAMCORE
#include "vslamcore/View.hpp"
#include "vslamcore/WorldPoint.hpp"
#include "vslamcore/MapPointSet.hpp"
#include "vslamcore/KeyFrameSet.hpp"
#include "vslamcore/converter.hpp"
#include "vslamcore/IMUInfo.hpp"
#else
#include "View.hpp"
#include "WorldPoint.hpp"
#include "MapPointSet.hpp"
#include "KeyFrameSet.hpp"
#include "converter.hpp"
#include "IMUInfo.hpp"
#endif


namespace vision {
    namespace vslam {

        void applyGravityRotationAndScale(KeyFrameSet& keyFrames,
                                          MapPointSet& mapPoints,
                                          IMUInfo& imuInfo) {


            keyFrames.lock();

            auto IdsAndPoses = keyFrames.getPoses();
            std::vector<int> allViewIds = IdsAndPoses.first;
            std::vector<cv::Matx44d> allPoses = IdsAndPoses.second;

            // Update camera poses
            for (int i = 0; i < static_cast<int>(allViewIds.size()); ++i) {

                cv::Matx44d Tform = imuInfo.gravityRotationTransform * allPoses[i];

                for (int k = 0; k < 3; ++k) {
                    Tform(k, 3) *= imuInfo.poseScale;
                }

                cv::Matx33d rot;
                cv::Vec3d tran;
                pose2rt(Tform, rot, tran);

                keyFrames.updateViewPose(allViewIds[i], rot, tran);

            }

            // Update point locations
            const std::unordered_map<int, std::shared_ptr<WorldPoint>> allWorldPoints = mapPoints.getAllWorldPoints();

            cv::Matx33d gRotMtx;
            cv::Vec3d gRotT;
            pose2rt(imuInfo.gravityRotationTransform, gRotMtx, gRotT);

            for (const auto& worldPoint : allWorldPoints) {

                const cv::Vec3d xyzLocBefore = mapPoints.getLocation(worldPoint.first);
                const cv::Vec3d xyzLocAfter = gRotMtx * xyzLocBefore + gRotT;

                mapPoints.updateWorldPoint(worldPoint.first, imuInfo.poseScale * xyzLocAfter);

            }

            imuInfo.isIMUAligned = true;

            keyFrames.unlock();

        }

    } // namespace vslam
} // namespace vision
