//////////////////////////////////////////////////////////////////////////////////
// Utility class for loop closure detection and corresponding map correction.
//
// Copyright 2022-23 The MathWorks, Inc.
//////////////////////////////////////////////////////////////////////////////////

#ifndef LOOP_CLOSURE_DATABASE_HPP
#define LOOP_CLOSURE_DATABASE_HPP

#include <string>
#include <unordered_map>
#include <vector>

#include "GeoTransform3DEstimatorCallback.hpp"
#include "InvertedImageIndex.hpp"
#include "KeyFrameSet.hpp"
#include "MapPointSet.hpp"
#include "Configuration.hpp"

namespace vision {
    namespace vslam {
        
        class LoopClosureDatabase {

            public:

            /**
             * @brief Constructor.
             *
             * @param[in] orbVocabularyFilename Name of the file containing OrbVocabulary. File 
             *            extension is .yml.gz created using class OrbVocabulary from 3p/DBoW2.
             */
            LoopClosureDatabase(const std::string& orbVocabularyFilename) : database(orbVocabularyFilename) {}

            /**
             * @brief Convert feature descriptor data structure as required by DBoW2.
             *
             * @param[in] plainFeatures feature descriptors of current key frame.
             * @param[out] outFeatures feature descriptors of current key frame in vector form.
             */
            void changeFeaturesStructure(const cv::Mat& plainFeatures, std::vector<cv::Mat>& outFeatures); 

            /**
             * @brief Find loop candidate key frames, which are visually 
             *        similar to but not connected to the current key frame.
             *
             * @param[in] kfSet keyFrameSet object containing views that observe the world point.
             * @param[in] currKeyFrameId frame ID of the current key frame, specified as an integer.
             * @param[in] currFeatures feature descriptors of current key frame.
             * @param[in] config configuration parameters of the system.
             * @param[out] loopKeyFrameIds Groups of consecutive keyframe IDs forming loop with current key frame.
             * @return true if a loop is detected, otherwise false.
             */
            bool checkLoopClosure(KeyFrameSet& kfSet,
                                  const int currKeyFrameId,
                                  const std::vector<cv::Mat>& currFeatures,
                                  const Configuration& config,
                                  std::vector<std::vector<int>>& loopKeyFrameIds);

            /**
             * @brief Add connections between the current key frame and the valid loop candidate 
             *        key frames.
             *
             * @param[in/out] mpSet a MapPointSet object containing 3-D world points.
             * @param[in/out] kfSet keyFrameSet object containing views that observe the world point.
             * @param[in] intrinsics camera intrinsics matrix, specified as a cv::Matx33d
             * @param[in] currKeyFrameId frame ID of the current key frame, specified as an integer.
             * @param[in] loopKeyFrameIds Groups of consecutive keyframe IDs forming loop with current key frame.
             * @param[in] config configuration parameters of the system.
             * @param[out] loopConnections loop closure connections, specified as a vector of pairs. Each pair contains
             *            a pointer to Connection, which contains the relative orientation R and relative translation t,  
             *            and the corresponding scale s. The relative pose of a loop closure connection is represented 
             *            as a 3-D similarity transform:
             *            | s*R, t |
             *            | 0,   1 |
             * @return true if the loop connection is successfully added, otherwise false.
             */
            bool addLoopConnections(MapPointSet& mpSet,
                                    KeyFrameSet& kfSet,
                                    const cv::Matx33d& intrinsics,
                                    const int currKeyFrameId,
                                    const std::vector<std::vector<int>>& loopKeyFrameIds,
                                    const Configuration& config,
                                    std::vector<std::pair<std::shared_ptr<Connection>, double>>& loopConnections);

            /**
             * @brief Add features of image to index database.
             *
             * @param[in]  currKeyFrameId frame ID of the current key frame.
             * @param[in]  imageFeatures  vector of features of an image.
             * @return Internal DBoW2 id for the current frame.
             */
            unsigned int addImageFeatures(const int currKeyFrameId, const std::vector<cv::Mat>& imageFeatures);
    
            /**
             * @brief Destructor.
             */
            ~LoopClosureDatabase() = default;

            private:
            
            /*
             * @brief Bag of words (BoW) database.
             */
            InvertedImageIndex database;

            /*
             * @brief Mapping from internal BoW IDs to keyframe IDs.
             */
            std::unordered_map<DBoW2::EntryId, int> bowIdToViewId;
        };
        
    }// namespace vslam
}// namespace vision

#endif // LOOP_CLOSURE_DATABASE_HPP
