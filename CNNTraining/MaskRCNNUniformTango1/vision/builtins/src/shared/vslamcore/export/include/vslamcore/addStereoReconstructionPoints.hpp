///////////////////////////////////////////////////////////////////////////////////
// Perform stereo reconstruction from disparity and update map with new 3-D points.
//
// Copyright 2023 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////////////

#ifndef ADDSTEREORECONSTRUCTIONPOINTS_HPP
#define ADDSTEREORECONSTRUCTIONPOINTS_HPP

#include "MapPointSet.hpp"
#include "KeyFrameSet.hpp"
#include "StereoReconstructor.hpp"
#include "matchFeatures.hpp"
#include "NodeIDGenerator.hpp"
#include "QueueInstance.hpp"

namespace vision {
    namespace vslam {
        
        /**
        * @brief Identify untracked indices in matchedPairs
        *
        * @param[in] matchedPairs, indices of matched features, specified as a vector of pairs<T,T> sorted least to greatest.
        * @param[in] trackedFeatureIndices, Indices of tracked feature points, specified as a vector<T> sorted least to greatest.
        * @return vector<bool> with TRUE values indicating untracked indices in matchedPairs.
        */
        template<class T>
        std::vector<bool> findUntrackedFeatures(const std::vector<std::pair<T,T>>& matchedPairs, const std::vector<T>& trackedFeatureIndices) {
            const size_t numPairs = matchedPairs.size(), numTracked = trackedFeatureIndices.size();
            std::vector<bool> isUntrackedIdx(numPairs, true);

            for (size_t i = 0, j = 0; i < numPairs && j < numTracked;) {
                if (matchedPairs[i].first < trackedFeatureIndices[j])
                    ++i;
                else {
                    if (matchedPairs[i].first == trackedFeatureIndices[j])
                        isUntrackedIdx[i++] = false;
                    ++j;
                }
            }

            return isUntrackedIdx;
        }

        class MapPointSet;
        class KeyFrameSet;
        class StereoReconstructor;

        /**
        * @brief Perform stereo reconstruction from disparity and update map with new 3-D points
        *
        * @param[in/out] mpSet, a MapPointSet object. If map initialization is successful, mpSet will contain the 
        *           triangulated 3-D world points found.
        * @param[in/out] kfSet, a KeyFrameSet object containing camera poses.
        * @param[in] reconstructor StereoReconstructor object used to construct 3-D world points.
        * @param[in/out] qObj, a QueueInstance Base struct containing data for the current frame.
        * @param[in] config, a vision::vslam::Configuration struct.
        * @param[in] generator, an ID generator used to gerenate pointIds and viewIds.
        * @return map linking 2-D image point indices in the current frame to 3-D world point IDs.
        *
        * Note: the function writes to the following members of qObj:
        *   qObj.trackedFeatureIndices Indices of tracked feature points in the last key frame, returned as a vector of integers.
        */
        std::unordered_map<int, int> addStereoReconstructionPoints(MapPointSet& mpSet,
            KeyFrameSet& kfSet,
            const StereoReconstructor& reconstructor,
            QueueInstanceStereo& qObj,
            const ConfigurationStereo& config,
            NodeIDGenerator& generator);
    }
}
#endif // ADDSTEREORECONSTRUCTIONPOINTS_HPP
