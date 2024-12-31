////////////////////////////////////////////////////////////////////////////////
// Utility function to filter out outlier 3-D world points
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/filterTriangulatedPoints.hpp"
#else
    #include "filterTriangulatedPoints.hpp"
#endif

namespace vision {
    namespace vslam {

        int filterTriangulatedPointsImpl(const cv::Vec3d& relPose_t, const cv::Mat& triangulatedPoints, const cv::Mat_<bool>& mask,
                          const std::vector<double>& errorsInCamera1, const std::vector<double>& errorsInCamera2, const double& minCosParallax, const double& maxReprojError,
                          std::vector<std::pair<int, int>>& matchedPairs, std::vector<cv::Vec3d>& xyzPoints) {

            const int numPoints = matchedPairs.size();
            xyzPoints.reserve(numPoints);
            std::vector<std::pair<int, int>> inlierMatchedPairs;
            inlierMatchedPairs.reserve(numPoints);

            int hasLargeParallax{ 0 };
            const bool* ptrMask = mask.ptr<bool>(0,0);
            for (int i = 0; i != numPoints; ++i) {

                if (ptrMask[i] && std::max(errorsInCamera1[i], errorsInCamera2[i]) <= maxReprojError) { 

                    cv::Vec3d singlePoint3d = cv::Vec3d(triangulatedPoints.col(i));

                    double cosAngle = singlePoint3d.dot(singlePoint3d - relPose_t) / (cv::norm(singlePoint3d) * cv::norm(singlePoint3d - relPose_t));

                    hasLargeParallax += (cosAngle < minCosParallax && cosAngle >0);

                    inlierMatchedPairs.push_back(matchedPairs[i]);
                    xyzPoints.push_back(singlePoint3d);
                }

            }
            matchedPairs = std::move(inlierMatchedPairs);
            return hasLargeParallax;
        }

        void filterTriangulatedPoints(const cv::Vec3d& relPose_t, const cv::Mat& triangulatedPoints, const int numPosDepth, const cv::Mat_<bool>& hasPosDepth,
                          const std::vector<double>& errorsInCamera1, const std::vector<double>& errorsInCamera2, const ConfigurationMono& config,
                          std::vector<std::pair<int, int>>& matchedPairs, std::vector<cv::Vec3d>& xyzPoints, bool& isMapInitialized, double& medianDepth) {

            if (numPosDepth < config.baseParams.minNumWorldPoints) {
                isMapInitialized = false;
                return;
            }

            int hasLargeParallax = filterTriangulatedPointsImpl(relPose_t, triangulatedPoints, hasPosDepth, errorsInCamera1, errorsInCamera2,
                                                                 config.monoParams.minCosParallaxInit, config.monoParams.maxReprojErrorMapInit, 
                                                                 matchedPairs, xyzPoints);

            // check if enough 3-D points with large parallax are reconstructed
            isMapInitialized = hasLargeParallax >= config.baseParams.minNumWorldPoints ;
            if (isMapInitialized) {
                // Find median depth of inlier points
                std::vector<double> depthOfPoint3d;
                depthOfPoint3d.reserve(xyzPoints.size());
                std::for_each(xyzPoints.begin(), xyzPoints.end(), [&](const cv::Vec3d point){ depthOfPoint3d.push_back(point[2]); });
                std::nth_element(depthOfPoint3d.begin(), depthOfPoint3d.begin() + depthOfPoint3d.size() / 2, depthOfPoint3d.end());
                medianDepth = depthOfPoint3d[depthOfPoint3d.size() / 2];
            }
        }
    } // namespace vslam
} // namespace vision