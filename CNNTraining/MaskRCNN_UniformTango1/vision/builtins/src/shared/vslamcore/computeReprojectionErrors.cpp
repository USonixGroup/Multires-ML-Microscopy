////////////////////////////////////////////////////////////////////////////////
// Compute reprojection errors for 3-D world points.
// 
// Copyright 2022 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////

#ifdef BUILDING_LIBMWVSLAMCORE
    #include "vslamcore/computeReprojectionErrors.hpp"
#else
    #include "computeReprojectionErrors.hpp"
#endif

namespace vision {
    namespace vslam {

        std::vector<double> computeReprojectionErrors(
            const cv::Mat& points3d,
            const std::vector<cv::Point2f>& imagePoints,
            const cv::Matx33d& extrinsics_R,
            const cv::Vec3d& extrinsics_t,
            const cv::Matx33d& intrinsics)
        {
            std::vector<double> reprojErrors;
            reprojErrors.resize(imagePoints.size());

            // Convert the rotation matrix to a Rodrigues rotation vector
            cv::Vec3d extrinsics_R_vec;
            cv::Rodrigues(extrinsics_R, extrinsics_R_vec);

            cv::Mat projectedPoints, projectedPoints2d;
            const cv::Matx<double, 1, 4> distCoeffs = cv::Matx<double, 1, 4>::zeros(); // No lens distortion

            // Project 3-D world point to the image plane
            cv::projectPoints(points3d, extrinsics_R_vec, extrinsics_t,
                intrinsics, distCoeffs, projectedPoints);

            // Convert from M-by-1-by-2 to M-by-2-by-1
            projectedPoints2d = projectedPoints.reshape(1);

            for (std::vector<cv::Point2f>::size_type i = 0; i != imagePoints.size(); i++) {
                double dx = static_cast<double>(imagePoints[i].x) - projectedPoints2d.at<double>(i, 0);
                double dy = static_cast<double>(imagePoints[i].y) - projectedPoints2d.at<double>(i, 1);
                reprojErrors[i] = std::sqrt(dx * dx + dy * dy);
            }

            return reprojErrors;
        }

        double computeReprojectionErrors(
            const cv::Vec3d& points3d,
            const cv::Point2f& imagePoint,
            const cv::Matx33d& extrinsics_R,
            const cv::Vec3d& extrinsics_t,
            const cv::Matx33d& intrinsics)
        {
            // Convert the rotation matrix to a Rodrigues rotation vector
            cv::Vec3d extrinsics_R_vec;
            cv::Rodrigues(extrinsics_R, extrinsics_R_vec);

            cv::Mat projectedPoints, projectedPoints2d;
            const cv::Matx<double, 1, 4> distCoeffs = cv::Matx<double, 1, 4>::zeros(); // No lens distortion

            // Project 3-D world point to the image plane
            cv::projectPoints(points3d, extrinsics_R_vec, extrinsics_t,
                intrinsics, distCoeffs, projectedPoints);

            // Convert from M-by-1-by-2 to M-by-2-by-1
            projectedPoints2d = projectedPoints.reshape(1);

            double dx = static_cast<double>(imagePoint.x) - projectedPoints2d.at<double>(0, 0);
            double dy = static_cast<double>(imagePoint.y) - projectedPoints2d.at<double>(0, 1);
            return std::sqrt(dx * dx + dy * dy);
        }
    } // namespace vslam
} // namespace vision