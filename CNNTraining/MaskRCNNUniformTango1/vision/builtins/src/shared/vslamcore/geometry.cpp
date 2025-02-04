////////////////////////////////////////////////////////////////////////////////
// geometry.cpp checkParallax
// 
// Copyright 2023 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/geometry.hpp"
#else
    #include "geometry.hpp"
#endif

namespace vision {
    namespace vslam {
        bool checkParallax(
            const cv::Mat& point3d,
            const cv::Mat& camLocation1,
            const cv::Mat& camLocation2,
            const double threshold) {

            const cv::Mat ray1 = point3d - camLocation1, ray2 = point3d - camLocation1;
            double cosAngle = ray1.dot(ray2) / (cv::norm(ray1) * cv::norm(ray2));

            return cosAngle > 0 && cosAngle <= threshold;
        }
    }
}