// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018-2021 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------
// Altered from:
// @author Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2021 Ignacio Vizzo, Cyrill Stachniss, University of Bonn.
// ----------------------------------------------------------------------------

#define _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING

#include "generalizedICP.h"

#include "Eigen/Dense"
#include "unsupported/Eigen/MatrixFunctions"

#include "open3d/geometry/KDTreeSearchParam.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/utility/Eigen.h"

namespace open3d {
namespace pipelines {
namespace registration {

namespace {

/// Obtain the Rotation matrix that transform the basis vector e1 onto the
/// input vector x.
inline Eigen::Matrix3d GetRotationFromE1ToX(const Eigen::Vector3d &x) {
    const Eigen::Vector3d e1{1, 0, 0};
    const Eigen::Vector3d v = e1.cross(x);
    const double c = e1.dot(x);
    if (c < -0.99) {
        // Then means that x and e1 are in the same direction
        return Eigen::Matrix3d::Identity();
    }

    const Eigen::Matrix3d sv = utility::SkewMatrix(v);
    const double factor = 1 / (1 + c);
    return Eigen::Matrix3d::Identity() + sv + (sv * sv) * factor;
}

/// Compute the covariance matrix according to the original paper. If the input
/// has already pre-computed covariances returns inmediatly. If the input has
/// pre-computed normals but no covariances, compute the covariances from those
/// normals. If there is no covariances nor normals, compute each covariance
/// matrix following the original implementation of GICP using 20 NN.
std::shared_ptr<geometry::PointCloud> InitializePointCloudForGeneralizedICP(
        const geometry::PointCloud &pcd, double epsilon) {
    auto output = std::make_shared<geometry::PointCloud>(pcd);
    if (output->HasCovariances()) {
        return output;
    }
    if (!output->HasNormals()) {
        // Compute covariances the same way is done in the original GICP paper.
        output->EstimateNormals(open3d::geometry::KDTreeSearchParamKNN(20));
    }

    output->covariances_.resize(output->points_.size());
    const Eigen::Matrix3d C = Eigen::Vector3d(epsilon, 1, 1).asDiagonal();
#pragma omp parallel for schedule(static)
    for (int i = 0; i < (int)output->normals_.size(); i++) {
        const auto Rx = GetRotationFromE1ToX(output->normals_[i]);
        output->covariances_[i] = Rx * C * Rx.transpose();
    }
    return output;
}
}  // namespace

double TransformationEstimationForGeneralizedICP::ComputeRMSE(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &correspondences) const {
    if (correspondences.empty()) {
        return 0.0;
    }
    double error = 0.0;
    for (const auto &c : correspondences) {
        const Eigen::Vector3d &sourceVertices = source.points_[c[0]];
        const Eigen::Matrix3d &sourceColors = source.covariances_[c[0]];
        const Eigen::Vector3d &targetVertices = target.points_[c[1]];
        const Eigen::Matrix3d &targetColors = target.covariances_[c[1]];
        const Eigen::Vector3d d = sourceVertices - targetVertices;
        const Eigen::Matrix3d M = targetColors + sourceColors;
        const Eigen::Matrix3d W = M.inverse().sqrt();
        error += d.transpose() * W * d;
    }
    return std::sqrt(error / (double)correspondences.size());
}

Eigen::Matrix4d
TransformationEstimationForGeneralizedICP::ComputeTransformation(
        const geometry::PointCloud &source,
        const geometry::PointCloud &target,
        const CorrespondenceSet &correspondences) const {
    if (correspondences.empty() || !target.HasCovariances() ||
        !source.HasCovariances()) {
        return Eigen::Matrix4d::Identity();
    }

    auto computeJacobianAndResidual =
            [&](int i,
                std::vector<Eigen::Vector6d, utility::Vector6d_allocator> &J_r,
                std::vector<double> &r, std::vector<double> &w) {
                const Eigen::Vector3d &sourceVertices = source.points_[correspondences[i][0]];
                const Eigen::Matrix3d &sourceColors = source.covariances_[correspondences[i][0]];
                const Eigen::Vector3d &targetVertices = target.points_[correspondences[i][1]];
                const Eigen::Matrix3d &targetColors = target.covariances_[correspondences[i][1]];
                const Eigen::Vector3d d = sourceVertices - targetVertices;
                const Eigen::Matrix3d M = targetColors + sourceColors;
                const Eigen::Matrix3d W = M.inverse().sqrt();

                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = -utility::SkewMatrix(sourceVertices);
                J.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
                J = W * J;

                constexpr int numRows = 3;
                J_r.resize(numRows);
                r.resize(numRows);
                w.resize(numRows);
                for (size_t iCount = 0; iCount < numRows; ++iCount) {
                    r[iCount] = W.row(iCount).dot(d);
                    w[iCount] = kernel->Weight(r[iCount]);
                    J_r[iCount] = J.row(iCount);
                }
            };

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
    double r2 = -1.0;
    std::tie(JTJ, JTr, r2) =
            utility::ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
                    computeJacobianAndResidual, (int)correspondences.size());

    bool isSuccess = false;
    Eigen::Matrix4d extrinsic;
    std::tie(isSuccess, extrinsic) =
            utility::SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ, JTr);

    return isSuccess ? extrinsic : Eigen::Matrix4d::Identity();
}

RegistrationResult RegistrationGeneralizedICP(
        const geometry::PointCloud &moving,
        const geometry::PointCloud &fixed,
        double correspondenceMetric,
        const Eigen::Matrix4d &init /* = Eigen::Matrix4d::Identity()*/,
        const TransformationEstimationForGeneralizedICP
                &estimation /* = TransformationEstimationForGeneralizedICP()*/,
        const ICPConvergenceCriteria
                &criteria /* = ICPConvergenceCriteria()*/,
        bool useInlierRatio /*= false*/,
        bool verbose /*= false*/
        ) {
    return RegistrationICP(
            *InitializePointCloudForGeneralizedICP(moving, estimation.epsilon),
            *InitializePointCloudForGeneralizedICP(fixed, estimation.epsilon),
            correspondenceMetric, init, estimation, criteria, useInlierRatio,
            verbose);
            
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d