#ifndef GEOMETRY_HPP
#define GEOMETRY_HPP

#include <cmath>
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/core/core_c.h"

#include "KeyFrameSet.hpp"
#include "MapPointSet.hpp"

namespace vision {
    namespace vslam {
        template <typename T>
        static float distancePointLine(const cv::Point_<T>& point, const cv::Vec<T, 3>& line)
        {
            //Line is given as a*x + b*y + c = 0
            return std::fabs(line(0) * point.x + line(1) * point.y + line(2))
                / std::sqrt(line(0) * line(0) + line(1) * line(1));
        }

        template<typename T>
        bool checkParallax(
            const cv::Vec<T, 3>& point3d,
            const cv::Vec<T, 3>& camLocation1,
            const cv::Vec<T, 3>& camLocation2,
            const T threshold) {

            const cv::Vec<T, 3> ray1 = point3d - camLocation1, ray2 = point3d - camLocation1;
            T cosAngle = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

            return cosAngle > 0 && cosAngle <= threshold;
        }

        template <typename T>
        double getMaxDepth(MapPointSet& mpSet, KeyFrameSet& kfSet, const int currKeyFrameId, const cv::Matx<T,3,3>& R, const cv::Vec<T,3>& t) {
            double maxDepth = 0;
            // Get all points in current frame
            auto wpoints(kfSet.findWorldPointsInView(currKeyFrameId));
            if(!wpoints.empty()) {
                for (const auto& kv : wpoints) {
                    // Convert points from world frame to camera frame
                    cv::Vec<T,3> locInWorldFrame = mpSet.getLocation(kv.first);
                    cv::Vec<T,3> locInCameraFrame = R * locInWorldFrame + t;

                    // Get depth and compare from maxDepth
                    if(maxDepth < locInCameraFrame[2]) {
                        maxDepth = locInCameraFrame[2];
                    }
                }
            }
            return maxDepth;
        }

        bool checkParallax(
            const cv::Mat& point3d,
            const cv::Mat& camLocation1,
            const cv::Mat& camLocation2,
            const double threshold);
    }// namespace vslam
}// namespace vision

#endif // GEOMETRY_HPP
