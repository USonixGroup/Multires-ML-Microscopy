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

#include "coloredICP.h"

#include <Eigen/Dense>

#include "open3d/geometry/KDTreeFlann.h"
#include "open3d/geometry/KDTreeSearchParam.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/pipelines/registration/RobustKernel.h"
#include "open3d/utility/Eigen.h"

#ifdef COMPILE_FOR_VISION_BUILTINS  // Used only for simulation 
// Below header file are used for error messages
#include <fl/except/MsgIDException.hpp>
#include "resources/vision/pointcloud.hpp"
#endif
#include <assert.h>
#include <stdio.h>

#if defined(NDEBUG)
#define VISION_ASSERT_MSG(EXPR, MSG) (void)0
#else
#define VISION_ASSERT_MSG(EXPR, MSG)            \
do {                                            \
        if (! (EXPR)) {                         \
                printf("%s \n",MSG);            \
                assert((EXPR));                 \
        }                                       \
} while (false)
#endif

namespace open3d {
namespace pipelines {
namespace registration {

namespace {

// Color gradient property addition into Open3D point cloud class
class PointCloudForColoredICP : public geometry::PointCloud {
public:
    std::vector<Eigen::Vector3d> colorGradient;
};

// Initialization of point clouds for color variant of point-to-plane ICP
// registration. Color gradient is calculated between color of the points 
// from the fixed point cloud and projection of color of the points from 
// the moving point cloud to the corresponding fixed point cloud point's 
// tangent's plane. This function can accept Search Hybrid parameters or 
// KNN search parameter for initializing the point cloud. Correspondences
// is calculated based on search parameter.
std::shared_ptr<PointCloudForColoredICP> InitializePointCloudForColoredICP(
        const geometry::PointCloud &fixed,
        const geometry::KDTreeSearchParam &searchParam) {

    geometry::KDTreeFlann tree;
    tree.SetGeometry(fixed);

    auto output = std::make_shared<PointCloudForColoredICP>();
    output->colors_ = fixed.colors_;
    output->normals_ = fixed.normals_;
    output->points_ = fixed.points_;

    size_t numPoints = output->points_.size();
    output->colorGradient.resize(numPoints, Eigen::Vector3d::Zero());

    for (size_t k = 0; k < numPoints; k++) {
        const Eigen::Vector3d &fixedVertices = output->points_[k];
        const Eigen::Vector3d &fixedNormals = output->normals_[k];
        double fixedColors = (output->colors_[k](0) + output->colors_[k](1) +
                     output->colors_[k](2)) /
                    3.0;

        std::vector<int> pointIdx;
        std::vector<double> squaredPointDistance;

        if (tree.Search(fixedVertices, searchParam, pointIdx, squaredPointDistance) >= 4) {
            // approximate image gradient of fixedVertices's tangential plane
            size_t numNeighbors = pointIdx.size();
            Eigen::MatrixXd A(numNeighbors, 3);
            Eigen::MatrixXd b(numNeighbors, 1);
            A.setZero();
            b.setZero();
            for (size_t i = 1; i < numNeighbors; i++) {
                int adjPointIdx = pointIdx[i];
                const Eigen::Vector3d &adjFixedVertex = output->points_[adjPointIdx];
                double adjFixedColors = (output->colors_[adjPointIdx](0) +
                                 output->colors_[adjPointIdx](1) +
                                 output->colors_[adjPointIdx](2)) /
                                3.0;
                A(i - 1, 0) = (adjFixedVertex(0) - fixedVertices(0));
                A(i - 1, 1) = (adjFixedVertex(1) - fixedVertices(1));
                A(i - 1, 2) = (adjFixedVertex(2) - fixedVertices(2));
                b(i - 1, 0) = (adjFixedColors - fixedColors);
            }
            // adds orthogonal constraint
            A(numNeighbors - 1, 0) = (numNeighbors - 1) * fixedNormals(0);
            A(numNeighbors - 1, 1) = (numNeighbors - 1) * fixedNormals(1);
            A(numNeighbors - 1, 2) = (numNeighbors - 1) * fixedNormals(2);
            b(numNeighbors - 1, 0) = 0;
            // solving linear equation
            bool isSuccess = false;
            Eigen::MatrixXd x;
            std::tie(isSuccess, x) = utility::SolveLinearSystemPSD(
                    A.transpose() * A, A.transpose() * b);
            if (isSuccess) {
                output->colorGradient[k] = x;
            }
        }
    }
    return output;
}
}  // namespace

// Estimation of transformation based on the correspondences between the 
// point clouds.
Eigen::Matrix4d TransformationEstimationForColoredICP::ComputeTransformation(
        const geometry::PointCloud &moving,
        const geometry::PointCloud &fixed,
        const CorrespondenceSet &correspondences) const {
    VISION_ASSERT_MSG(fixed.HasNormals(),"Point clouds initialization return null value");

    double sqrtLambdaGeometric = sqrt(lambdaGeometric);
    double lambdaPhotometric = 1.0 - lambdaGeometric;
    double sqrtLambdaPhotometric = sqrt(lambdaPhotometric);

    const auto &coloredFixed = (const PointCloudForColoredICP &)fixed;

    auto computeJacobianAndResidual =
            [&](int i,
                std::vector<Eigen::Vector6d, utility::Vector6d_allocator> &Jr,
                std::vector<double> &r, std::vector<double> &w) {
                size_t movingCorrespondence = correspondences[i][0];
                size_t fixedCorrespondence = correspondences[i][1];
                const Eigen::Vector3d &movingVertices = moving.points_[movingCorrespondence];
                const Eigen::Vector3d &fixedVertices = fixed.points_[fixedCorrespondence];
                const Eigen::Vector3d &fixedNormals = fixed.normals_[fixedCorrespondence];

                Jr.resize(2);
                r.resize(2);
                w.resize(2);

                Jr[0].block<3, 1>(0, 0) = sqrtLambdaGeometric * movingVertices.cross(fixedNormals);
                Jr[0].block<3, 1>(3, 0) = sqrtLambdaGeometric * fixedNormals;
                r[0] = sqrtLambdaGeometric * (movingVertices - fixedVertices).dot(fixedNormals);
                w[0] = kernel->Weight(r[0]);

                // project movingVertices into fixedVertices's tangential plane
                Eigen::Vector3d projectedMovingVertices = movingVertices - (movingVertices - fixedVertices).dot(fixedNormals) * fixedNormals;
                double movingColors = (moving.colors_[movingCorrespondence](0) + moving.colors_[movingCorrespondence](1) +
                             moving.colors_[movingCorrespondence](2)) /
                            3.0;
                double fixedColors = (fixed.colors_[fixedCorrespondence](0) + fixed.colors_[fixedCorrespondence](1) +
                             fixed.colors_[fixedCorrespondence](2)) /
                            3.0;
                const Eigen::Vector3d &fixedColorGradients = coloredFixed.colorGradient[fixedCorrespondence];
                double projectedMovingColors = (fixedColorGradients.dot(projectedMovingVertices - fixedVertices)) + fixedColors;

                const Eigen::Matrix3d &M =
                        Eigen::Matrix3d::Identity() - fixedNormals * fixedNormals.transpose();
                const Eigen::Vector3d &fixedColorGradientsMat = fixedColorGradients.transpose() * M;

                Jr[1].block<3, 1>(0, 0) =
                        sqrtLambdaPhotometric * movingVertices.cross(fixedColorGradientsMat);
                Jr[1].block<3, 1>(3, 0) = sqrtLambdaPhotometric * fixedColorGradientsMat;
                r[1] = sqrtLambdaPhotometric * (projectedMovingColors - movingColors);
                w[1] = kernel->Weight(r[1]);
            };

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
    double r2;
    std::tie(JTJ, JTr, r2) =
            utility::ComputeJTJandJTr<Eigen::Matrix6d, Eigen::Vector6d>(
                    computeJacobianAndResidual, (int)correspondences.size());

    bool isSuccess;
    Eigen::Matrix4d extrinsic;
    std::tie(isSuccess, extrinsic) =
            utility::SolveJacobianSystemAndObtainExtrinsicMatrix(JTJ, JTr);

    return isSuccess ? extrinsic : Eigen::Matrix4d::Identity();
}

// Estimation of inlier RMSEs based on the correspondences between the 
// point clouds.
double TransformationEstimationForColoredICP::ComputeRMSE(
        const geometry::PointCloud &moving,
        const geometry::PointCloud &fixed,
        const CorrespondenceSet &correspondences) const {
    double sqrtLambdaGeometric = sqrt(lambdaGeometric);
    double lambdaPhotometric = 1.0 - lambdaGeometric;
    double sqrtLambdaPhotometric = sqrt(lambdaPhotometric);
    const auto &coloredFixed = (const PointCloudForColoredICP &)fixed;

    double residual = 0.0;
    for (size_t i = 0; i < correspondences.size(); i++) {
        size_t movingCorrespondence = correspondences[i][0];
        size_t fixedCorrespondence = correspondences[i][1];
        const Eigen::Vector3d &movingVertices = moving.points_[movingCorrespondence];
        const Eigen::Vector3d &fixedVertices = fixed.points_[fixedCorrespondence];
        const Eigen::Vector3d &fixedNormals = fixed.normals_[fixedCorrespondence];
        Eigen::Vector3d projectedMovingVertices = movingVertices - (movingVertices - fixedVertices).dot(fixedNormals) * fixedNormals;
        double movingColors = (moving.colors_[movingCorrespondence](0) + moving.colors_[movingCorrespondence](1) +
                     moving.colors_[movingCorrespondence](2)) /
                    3.0;
        double fixedColors = (fixed.colors_[fixedCorrespondence](0) + fixed.colors_[fixedCorrespondence](1) +
                     fixed.colors_[fixedCorrespondence](2)) /
                    3.0;
        const Eigen::Vector3d &fixedColorGradients = coloredFixed.colorGradient[fixedCorrespondence];
        double projectedInitialMovingColors = (fixedColorGradients.dot(projectedMovingVertices - fixedVertices)) + fixedColors;
        double residualGeometric = sqrtLambdaGeometric * (movingVertices - fixedVertices).dot(fixedNormals);
        double residualPhotometric = sqrtLambdaPhotometric * (movingColors - projectedInitialMovingColors);
        residual += residualGeometric * residualGeometric +
                    residualPhotometric * residualPhotometric;
    }
    return residual;
};

RegistrationResult RegistrationColoredICP(
        const geometry::PointCloud &moving,
        const geometry::PointCloud &fixed,
        double correspondenceMetric,
        const Eigen::Matrix4d &init /* = Eigen::Matrix4d::Identity()*/,
        const TransformationEstimationForColoredICP &estimation
        /* = TransformationEstimationForColoredICP()*/,
        const ICPConvergenceCriteria
                &criteria /* = ICPConvergenceCriteria()*/,
        bool useInlierRatio /*= false*/,
        bool verbose /*= false*/) {
    VISION_ASSERT_MSG(fixed.HasNormals(),"Point clouds initialization return null value");
    
    RegistrationResult regICP;
    std::shared_ptr<PointCloudForColoredICP> coloredFixed;
    if (useInlierRatio) {
        coloredFixed = InitializePointCloudForColoredICP(
                fixed,
                geometry::KDTreeSearchParamKNN(30));
    } else {
        coloredFixed = InitializePointCloudForColoredICP(
                fixed,
                geometry::KDTreeSearchParamHybrid(correspondenceMetric * 2.0, 30)); 
    }

    if (coloredFixed) {
        regICP = RegistrationICP(moving,
                *coloredFixed,
                correspondenceMetric, init, estimation, criteria, useInlierRatio, verbose);       
    }   
    else {       
        #ifdef COMPILE_FOR_VISION_BUILTINS
        throw fl::except::MakeException(vision::pointcloud::invalidInitialization());
        #else
        VISION_ASSERT_MSG(true,"Unable to initialize target point cloud with color property");
        #endif
    };
     return regICP;
    
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
