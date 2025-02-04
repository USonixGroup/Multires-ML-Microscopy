////////////////////////////////////////////////////////////////////////////////
// Find matched features between two groups of ORB feature descriptors
// 
// Copyright 2022-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/matchFeatures.hpp"
#else
    #include "matchFeatures.hpp"
#endif

namespace vision {
    namespace vslam {

        std::vector<std::pair<int, int>> matchFeatures(
            const cv::Mat& features1,
            const cv::Mat& features2,
            const std::vector<cv::KeyPoint>& points1,
            const std::vector<cv::KeyPoint>& points2,
            const int matchThreshold,
            const float maxRatio,
            const int maxOctaveDiff) {

            CV_Assert(features1.rows == static_cast<int>(points1.size()) && features2.rows == static_cast<int>(points2.size()));

            // Compute Hamming distance of feature descriptors
            cv::Mat featureDist; // int
            cv::batchDistance(features1, features2, featureDist, CV_32S, cv::noArray(), cv::NORM_HAMMING);

            // Matched pair = { index , matchIndex[index] }
            std::vector<int> matchIndex(featureDist.rows, -1); // matchIndex[i] = -1 indicates no valid match for feature1[i]

            const float matchThresholdF = static_cast<float>(matchThreshold);

            auto findMatches = [&](const cv::Range& range){
                // Find best match in features2 for each feature in features1
                for (int i = range.start; i < range.end; ++i) {
                    int minScore{ 256 }, min2Score = minScore; // Initialize the best matched and the second best match  
                    int minIndex{ 0 }, min2Index{ 0 };
                    const int* featureDistPtr = featureDist.ptr<int>(i,0);

                    for (int j = 0; j != featureDist.cols; ++j) {
                        if (std::abs(points1[i].octave - points2[j].octave) <= maxOctaveDiff) {
                            if (featureDistPtr[j] < minScore) {
                                min2Score = minScore;                                
                                minScore  = featureDistPtr[j];
                                min2Index = minIndex;
                                minIndex  = j;
                            }
                            else if (featureDistPtr[j] < min2Score) {
                                min2Score = featureDistPtr[j];
                                min2Index = j;
                            }
                        }
                    }

                    // Perform ratio check only when the best and the second best candidates are extracted from the same level
                    if (minScore <= matchThresholdF &&
                        (minScore <= maxRatio * min2Score || points2[min2Index].octave != points2[minIndex].octave)) {

                        // Enforce uniqueness by applying a bi-directional match constraint
                        bool isMutualMatch = true;
                        int row = 0;
                        const int bestScore{ featureDist.at<int>(i, minIndex) };

                        while (isMutualMatch && row != featureDist.rows) {
                            const int currScore{ featureDist.at<int>(row, minIndex) };
                            isMutualMatch = (currScore > bestScore || row == i ||
                                std::abs(points1[row].octave - points2[minIndex].octave) > maxOctaveDiff);
                            row++;
                        }

                        if (isMutualMatch) {
                            matchIndex[i] = minIndex;
                        }
                    }
                }
            };

            const cv::Range rowRange(0, featureDist.rows);
            constexpr int parallelThreshold{ 100 };
            if (rowRange.end > parallelThreshold) {
                cv::parallel_for_(rowRange, findMatches);
            } else { // cv::parallel_for_ unlikely to increase performance due to overhead
                findMatches(rowRange);
            }

            // Make valid matched pairs in order of features1 index
            std::vector<std::pair<int,int>> matchedPairs;
            matchedPairs.reserve(matchIndex.size());
            for (int i = 0; i < static_cast<int>(matchIndex.size()); ++i) {
                if (matchIndex[i] != -1) {
                    matchedPairs.push_back(std::make_pair(i, matchIndex[i]));
                }
            }

            return matchedPairs;
        }

        std::vector<std::pair<int, int>> matchFeaturesInRadius(
            const cv::Mat& features1,
            const cv::Mat& features2,
            const std::vector<cv::KeyPoint>& points1,
            const std::vector<cv::KeyPoint>& points2,
            const cv::Mat& centerPoints,
            const std::vector<float>& searchRadius,
            const int matchThreshold,
            const float maxRatio,
            const int maxOctaveDiff) {

            CV_Assert(features1.rows == static_cast<int>(searchRadius.size()) &&
                features1.rows == static_cast<int>(points1.size()) &&
                features2.rows == static_cast<int>(points2.size()));

            CV_Assert((centerPoints.rows == static_cast<int>(points1.size()) && centerPoints.cols == 2));

            // Convert from cv::KeyPoint to cv::Mat
            cv::Mat candidatePoints(points2.size(), 2, CV_32F);
            for (std::size_t i = 0; i != points2.size(); ++i) {
                candidatePoints.at<float>(i, 0) = points2[i].pt.x;
                candidatePoints.at<float>(i, 1) = points2[i].pt.y;
            }

            cv::Mat spatialDist, featureDist; // float, int

            // Compute spatial distance of feature points in pixels
            cv::batchDistance(centerPoints, candidatePoints, spatialDist, CV_32F, cv::noArray(), cv::NORM_L2);

            // Compute Hamming distance of feature descriptors
            cv::batchDistance(features1, features2, featureDist, CV_32S, cv::noArray(), cv::NORM_HAMMING);

            // Matched pair = { index , features1Matches[index] }
            std::vector<int> features1Matches(features1.rows, -1); // features1Matches[i] = -1 indicates no valid match for feature1[i]

            const float matchThresholdF = static_cast<float>(matchThreshold);

            auto findMatches = [&](const cv::Range& range){
                // Find best match in features2 for each feature in features1
                for (int i = range.start; i < range.end; ++i) {
                    int minScore{ 256 }, min2Score = minScore; // Initialize the best match and the second best match
                    int minIndex{ 0 };
                    const int* featureDistPtr = featureDist.ptr<int>(i,0);
                    const float* spatialDistPtr = spatialDist.ptr<float>(i,0);
                    for (int j = 0; j != features2.rows; ++j) {
                        if (std::abs(points1[i].octave - points2[j].octave) <= maxOctaveDiff &&
                                spatialDistPtr[j] <= searchRadius[i]) {

                            const int currScore = featureDistPtr[j];

                            if (currScore < minScore) {
                                min2Score = minScore;
                                minScore = currScore;
                                minIndex = j;
                            }
                            else if (currScore < min2Score) {
                                min2Score = currScore;
                            }
                        }
                    }

                    // Check matching threshold and maximum ratio constraints
                    if (minScore <= std::min(maxRatio * min2Score, matchThresholdF)) {
                        features1Matches[i] = minIndex;
                    }
                }
            };

            const cv::Range rowRange(0, features1.rows);
            constexpr int parallelThreshold{ 75 };
            if (rowRange.end > parallelThreshold) {
                cv::parallel_for_(rowRange, findMatches);
            } else { // cv::parallel_for_ unlikely to increase performance due to overhead
                findMatches(rowRange);
            }

            // Enforce uniqueness: features in features2 must have a maximum of one match.
            // features2Matches[features2Index] = {features1Index, score}
            std::vector<std::pair<int,int>> features2Matches(features2.rows, {-1,INT_MAX}); // features2Matches[i].first == -1 indicates invalid
            for (int i = 0; i < features1.rows; ++i) {
                if (features1Matches[i] == -1) { // features1[i] has no valid match
                    continue;
                }

                const int prevScore = features2Matches[features1Matches[i]].second;
                const int currScore = featureDist.at<int>(i, features1Matches[i]);

                if (currScore < prevScore) {
                    // Update features2Matches with best match and score
                    features2Matches[features1Matches[i]].first = i;
                    features2Matches[features1Matches[i]].second = currScore;
                }
                else if (currScore == prevScore) {
                    // Invalidate match with tie score
                    features2Matches[features1Matches[i]].first = -1;
                }
            }

            // Make valid matched pairs in order of features1 index
            std::vector<std::pair<int, int>> matchedPairs;
            matchedPairs.reserve(std::min(features1.rows, features2.rows));
            for (int i = 0; i < features1.rows; ++i) {
                const int j = features1Matches[i];
                if (j != -1 && features2Matches[j].first == i) {
                    matchedPairs.push_back(std::make_pair(i, j));
                }
            }

            return matchedPairs;
        }
    } // namespace vslam
} // namespace vision