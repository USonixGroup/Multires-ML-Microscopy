///////////////////////////////////////////////////////////////////////////
// Create new map points from stereo reconstruction using disparity
// 
// Copyright 2023 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/View.hpp"
    #include "vslamcore/WorldPoint.hpp"
    #include "vslamcore/MapPointSet.hpp"
    #include "vslamcore/KeyFrameSet.hpp"
    #include "vslamcore/addStereoReconstructionPoints.hpp"
    #include "vslamcore/converter.hpp"
#else
    #include "View.hpp"
    #include "WorldPoint.hpp"
    #include "MapPointSet.hpp"
    #include "KeyFrameSet.hpp"
    #include "addStereoReconstructionPoints.hpp"
    #include "converter.hpp"
#endif

namespace vision {
    namespace vslam {

        std::unordered_map<int, int> addStereoReconstructionPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const StereoReconstructor& reconstructor,
            QueueInstanceStereo& qObj,
            const ConfigurationStereo& config,
            NodeIDGenerator& generator) {

            constexpr int maxOctaveDiff{ 0 };
            constexpr float maxRatioStereo{ 1.f };
            std::vector<std::pair<int, int>> matchedPairs = matchFeatures(
                qObj.currFeatures, qObj.featuresR, qObj.currPoints, qObj.pointsR, config.baseParams.matchThreshold, maxRatioStereo, maxOctaveDiff);

            // Reconstruct 3-D points using disparity
            std::vector<cv::Vec3d> xyzPointsL;
            reconstructor.reconstruct(qObj, matchedPairs, xyzPointsL, config);

            // Add new 3-D points and correspondences
            const size_t numPoints = matchedPairs.size();
            std::unordered_map<int, int> stereoCorrespondence;
            stereoCorrespondence.reserve(numPoints);
            const std::shared_ptr<const View> currView = kfSet.findView(qObj.currKeyFrameId);
            const cv::Matx33d& rot = currView->getOrientation();
            const cv::Vec3d& trans = currView->getLocation();

            // Find untracked points
            std::sort(qObj.trackedFeatureIndices.begin(), qObj.trackedFeatureIndices.end());
            std::vector<bool> isUntrackedIdx = findUntrackedFeatures(matchedPairs, qObj.trackedFeatureIndices);

            for (size_t i = 0; i < numPoints; ++i) {
                if (isUntrackedIdx[i]) {
                    // Create new 3-D points for untracked features
                    const int currPointId = generator.newIDs(NodeType::POINT_XYZ).identifier;
                    mpSet.addWorldPoint(std::make_shared<WorldPoint>(currPointId, rot*xyzPointsL[i]+trans));

                    // Update WorldPoint attributes
                    mpSet.addCorrespondence(currPointId, qObj.currKeyFrameId, matchedPairs[i].first, kfSet);
                    mpSet.updateLimitsAndDirection(currPointId, kfSet);
                    mpSet.updateRepresentativeView(currPointId, kfSet);

                    // Add stereo correspondence
                    stereoCorrespondence[matchedPairs[i].first] = currPointId;
                }
            }

            return stereoCorrespondence;
        }
    }
}
