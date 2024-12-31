////////////////////////////////////////////////////////////////////////////////
// Detect and extract uniformly-distributed ORB features from a grayscale image
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/detectAndExtractFeatures.hpp"
#else
    #include "detectAndExtractFeatures.hpp"
#endif

namespace vision {
    namespace vslam {

        //TODO: share code with toolbox/vision/builtins/src/vision/selectUniformPoints.cpp
        std::vector<int>  selectUniformImpl(
            const std::vector<cv::KeyPoint>& pointsIn,
            const int nRows,
            const int nCols,
            const int numPointsOut,
            std::vector<bool>& isSelected) {
            
            // Create a grid of square cell such that the number of cells is as close to N as possible. 
            // Then pick the strongest point in each cell.
            const double aspectRatio = nCols * 1.0 / nRows;
            const int h = std::max<int>(static_cast<int>(std::sqrt(numPointsOut / aspectRatio)), 1);
            const int w = std::max<int>(static_cast<int>(h * aspectRatio), 1);

            const double gridStepW = nCols / w;
            const double gridStepH = nRows / h;
            std::vector<std::vector<int> > binIdx(w);
            std::for_each(binIdx.begin(), binIdx.end(),
                [h](std::vector<int>& v) {v.resize(h); }); // Initialize indices as zeros

            for (std::vector<bool>::size_type i = 0; i != isSelected.size(); ++i)
            {
                if (isSelected[i]) // Selected in the previous round
                    continue;

                int whichBinW = std::min<int>(static_cast<int>(pointsIn[i].pt.x / gridStepW), w-1);
                int whichBinH = std::min<int>(static_cast<int>(pointsIn[i].pt.y / gridStepH), h-1);
                
                int idx = binIdx[whichBinW][whichBinH];

                // Select the point if the cell is empty or the point has a stronger response than the 
                // existing point in the cell
                if (idx < 1 || pointsIn[idx-1].response < pointsIn[i].response)  
                {
                    binIdx[whichBinW][whichBinH] = static_cast<int>(i+1);
                }
            }

            std::vector<int> pointsIdx;
            for (const auto& col : binIdx)
            {
                for (const auto& rowIdx : col)
                {   
                    if (static_cast<int>(pointsIdx.size()) == numPointsOut)
                        return pointsIdx;

                    if (rowIdx > 0) {
                        pointsIdx.push_back(rowIdx-1);
                        isSelected[rowIdx-1] = true;
                    }
                }
            }
            return pointsIdx;
        }

        std::vector<int>  selectUniform(
            const std::vector<cv::KeyPoint>& pointsIn,
            const int nRows,
            const int nCols,
            const int numPointsOut) {

            std::vector<int> pointIndex;
            pointIndex.reserve(numPointsOut);

            std::vector<bool> isPointSelected(pointsIn.size(), false);
            int partNumPointsOut{ numPointsOut };
            std::vector<int> partIndex;
            while (static_cast<int>(pointIndex.size()) < numPointsOut) {
                partIndex = selectUniformImpl(pointsIn, nRows, nCols, partNumPointsOut, isPointSelected);
                pointIndex.insert(pointIndex.end(), partIndex.begin(), partIndex.end());
                partNumPointsOut = numPointsOut - pointIndex.size();
            }
            return pointIndex;
        }

        void detectAndExtractFeatures(
            const cv::Mat& frame,
            const float scaleFactor,
            const int numLevels,
            const int numFeatures,
            std::vector<cv::KeyPoint>& points,
            cv::Mat& features) {

            CV_Assert( frame.channels() == 1); // Grayscale image only

            cv::Ptr<cv::ORB>  orbDetector = cv::ORB::create(
                frame.rows * frame.cols, scaleFactor, numLevels);

            std::vector<cv::KeyPoint> pointsAll;

            // Detect keypoints
            orbDetector->detect(frame, pointsAll, cv::Mat());

            points.clear();
            points.reserve(numFeatures);

            // Select uniform points
            if (static_cast<int>(pointsAll.size()) <= numFeatures) {
                    points = std::move(pointsAll);
            }
            else {
                std::vector<int>  uniformIdx = selectUniform(pointsAll, frame.rows, frame.cols, numFeatures);
                for (const auto& idx : uniformIdx) {
                    points.push_back(pointsAll[idx]);
                }
            }

            // Extract features
            orbDetector->compute(frame, points, features);
        }
    } // namespace vslam
} // namespace vision
