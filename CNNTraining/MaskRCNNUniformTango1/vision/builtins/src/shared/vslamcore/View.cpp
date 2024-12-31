////////////////////////////////////////////////////////////////////////////////
// View class storing camera pose, feature descriptors, and key points
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/View.hpp"
    #include "vslamcore/converter.hpp"
#else
    #include "View.hpp"
    #include "converter.hpp"
#endif

namespace vision {
    namespace vslam {

        View::View(const int Id,
            const std::vector<cv::KeyPoint>& pts,
            const cv::Mat& fs,
            const cv::Matx33d& ori,
            const cv::Vec3d& loc,
            const int fIndex) :
            viewId(Id), orientation(ori), location(loc), featurePoints(pts), featureDescriptors(fs) {
                frameIndex = (fIndex == -1) ? Id : fIndex;
            }

        View::~View() {}

        int View::getViewId() const {
            return viewId;
        }

        int View::getFrameIndex() const {
            return frameIndex;
        }

        cv::Matx44d View::getPose() const {
            return rt2pose(orientation, location);
        }

        std::pair<std::vector<double>, std::vector<double>> View::getVelocityAndBias()  {
            return { velocity, bias };
        }

        std::vector<cv::KeyPoint> View::getFeaturePoints() const {
            return featurePoints;
        }

        std::vector<cv::KeyPoint> View::getFeaturePoints(const std::vector<int>& idx) const {
            std::vector<cv::KeyPoint> outPoints;
            outPoints.reserve(idx.size());

            for (const auto ele : idx) {
                outPoints.push_back(featurePoints[ele]);
            }
            return outPoints;
        }

        cv::KeyPoint View::getFeaturePoints(const int idx) const {
            return featurePoints[idx];
        }

        cv::Mat View::getFeatureDescriptors() const {
            return featureDescriptors;
        }

        cv::Mat View::getFeatureDescriptors(const std::vector<int>& idx) {
            int numFeatures = idx.size();
            cv::Mat outFeatures = cv::Mat::zeros(numFeatures, featureDescriptors.cols, featureDescriptors.type());

            for (int i = 0; i != numFeatures; ++i) {
                featureDescriptors.row(idx[i]).copyTo(outFeatures.row(i));
            }
            return outFeatures;
        }

        cv::Mat View::getFeatureDescriptors(const int idx) {
            return featureDescriptors.row(idx);
        }

        cv::Matx33d View::getOrientation() const {
            return orientation;
        }

        cv::Vec3d View::getLocation() const {
            return location;
        }

        std::unordered_map<int, int> View::getWorldPointsInView() const {
            return worldPointsInView;
        }

        std::vector<int> View::getConnectedViews(const int minNumMatches) const {
            std::vector<int> outViews;
            outViews.reserve(neighborViewsWithMatches.size());

            for (const auto ele : neighborViewsWithMatches) {
                if (ele.second >= minNumMatches) {
                    outViews.push_back(ele.first);
                }
            }
            return outViews;
        }


        std::pair< std::vector<double>, std::vector<double> > View::getIMU() const {
            return imuViewMeasurements;
        }

        bool View::isConnectedView(const int viewId, const int minNumMatches) const {
            return neighborViewsWithMatches.find(viewId) != neighborViewsWithMatches.end() && neighborViewsWithMatches.at(viewId) >= minNumMatches;
        }

        void View::setPose(const cv::Matx33d& ori, const cv::Vec3d& loc) {
            setOrientation(ori);
            setLocation(loc);
        }

        void View::setOrientation(const cv::Matx33d& ori) {
            orientation = ori;
        }

        void View::setLocation(const cv::Vec3d& loc) {
            location = loc;
        }

        void View::setVelocityAndBias(const std::vector<double>& velo, const std::vector<double>& bs) {
            velocity = velo;
            bias = bs;
        }

        void View::setIMU(const std::vector<double>& gyro, const std::vector<double>& accel) {

            imuViewMeasurements.first.insert(imuViewMeasurements.first.end(), gyro.begin(), gyro.end());
            imuViewMeasurements.second.insert(imuViewMeasurements.second.end(), accel.begin(), accel.end());

        }

        void View::addConnectedView(const int viewId, const int numMatches) {
            neighborViewsWithMatches[viewId] = numMatches;
        }

        void View::removeConnectedView(const int viewId) {
            neighborViewsWithMatches.erase(viewId);
        }

        void View::updateConnectedView(const int viewId, const int numMatches) {
            neighborViewsWithMatches[viewId] += numMatches;
        }

        void View::addCorrespondence(const int pointId, const int featureIdx) {
            worldPointsInView[pointId] = featureIdx;
        }
        
        int View::getCorrespondence(const int pointId) {
            int retval(-1);
            auto obj(worldPointsInView.find(pointId));
            if(obj != worldPointsInView.end()){
                retval = obj->second;
            }
            return retval;
        }

        void View::removeCorrespondence(const int pointId) {
            worldPointsInView.erase(pointId);
        }
    } // namespace vslam
} // namespace vision