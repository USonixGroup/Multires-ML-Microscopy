////////////////////////////////////////////////////////////////////////////////
// World point class storing location, distance limits, viewing direction and 
// representative view.
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
#include "vslamcore/WorldPoint.hpp"
#else
#include "WorldPoint.hpp"
#endif

namespace vision {
    namespace vslam {

        WorldPoint::WorldPoint(const int Id, const cv::Vec3d& loc) :
            location(loc), pointId(Id) {}

        WorldPoint::~WorldPoint() {}

        int WorldPoint::getPointId() const {
            return pointId;
        }

        cv::Vec3d WorldPoint::getLocation() const {
            return location;
        }

        cv::Vec3d WorldPoint::getViewingDirection() const {
            return viewingDirection;
        }

        std::pair<double, double> WorldPoint::getDistanceLimits() const {
            return distanceLimits;
        }

        int WorldPoint::getRepresentativeView() {
            return representativeView;
        }

        cv::Mat WorldPoint::getRepresentativeFeatureDescriptor(KeyFrameSet& kfSet) {
            return kfSet.findView(representativeView)->getFeatureDescriptors(viewsOfPoint[representativeView]);
        }

        cv::KeyPoint WorldPoint::getRepresentativeFeaturePoint(KeyFrameSet& kfSet) {
            return kfSet.findView(representativeView)->getFeaturePoints(viewsOfPoint[representativeView]);
        }

        std::unordered_map<int, int> WorldPoint::getViewsOfWorldPoint() const {
            return viewsOfPoint;
        }

        bool WorldPoint::isValid() const {
            return validity;
        }

        void WorldPoint::updateLocation(const cv::Vec3d& loc) {
            location = loc;
        }

        void WorldPoint::updateLimitsAndDirection(KeyFrameSet& kfSet) {
            if (!viewsOfPoint.empty()) {

                cv::Vec3d sumOfNormalizedDirectionVectors;
                std::vector<double> viewToPointDist;
                for (const auto ele : viewsOfPoint) {
                    cv::Vec3d vec = kfSet.findView(ele.first)->getLocation() - location;
                    cv::Vec3d normalizedVec;
                    cv::normalize(vec, normalizedVec);
                    sumOfNormalizedDirectionVectors += normalizedVec;
                    viewToPointDist.push_back(std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]));
                }

                // Update distance limits
                const auto minmax = std::minmax_element(viewToPointDist.begin(), viewToPointDist.end());
                distanceLimits = std::make_pair(*(minmax.first), *(minmax.second));

                // Update direction
                cv::normalize(sumOfNormalizedDirectionVectors / static_cast<double>(viewsOfPoint.size()), viewingDirection);
            }
        }

        void WorldPoint::updateRepresentativeView(KeyFrameSet& kfSet) {
            if (!viewsOfPoint.empty()) {

                int numFeatures = viewsOfPoint.size();

                if (numFeatures == 1) {
                    representativeView = viewsOfPoint.begin()->first;
                    return;
                }
                else if (numFeatures == 2) {
                    auto it = viewsOfPoint.begin();
                    representativeView = std::max(it->first, it++->first);
                    return;
                }

                constexpr int featureLength{ 32 };
                cv::Mat allFeatures = cv::Mat(numFeatures, featureLength, CV_8U);

                int rowNum{ 0 };
                std::vector<int> candidateViews(numFeatures);
                for (const auto ele : viewsOfPoint) {
                    kfSet.findView(ele.first)->getFeatureDescriptors(ele.second).copyTo(allFeatures.row(rowNum));
                    candidateViews[rowNum] = ele.first;
                    rowNum++;
                }

                int minIdx{ 0 };
                // Compute Hamming distance of feature descriptors
                cv::Mat featureDist;
                cv::batchDistance(allFeatures, allFeatures, featureDist, CV_32S, cv::noArray(), cv::NORM_HAMMING);

                cv::Mat rowSum;
                constexpr int dim{ 1 }; // Compute sum of each row
                featureDist.convertTo(featureDist, CV_64F); // cv::reduce does not support CV_32S
                cv::reduce(featureDist, rowSum, dim, cv::REDUCE_SUM, CV_64F);

                double minVal = rowSum.at<double>(minIdx, 0);
                for (int i = 1; i != rowSum.rows; ++i) {
                    if (rowSum.at<double>(i, 0) < minVal) {
                        minVal = rowSum.at<double>(i, 0);
                        minIdx = i;
                    }
                }

                representativeView = candidateViews[minIdx];
            }
        }

        void WorldPoint::addCorrespondence(const int viewId, const int featureIdx) {
            viewsOfPoint[viewId] = featureIdx;
        }

        void WorldPoint::removeCorrespondence(const int viewId) {
            viewsOfPoint.erase(viewId);
        }

        void WorldPoint::setInvalid() {
            validity = false;
            viewsOfPoint.clear();
        }
    } // namespace vslam
} // namespace vision