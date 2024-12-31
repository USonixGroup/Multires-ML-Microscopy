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

#pragma once

#include <Eigen/Core>
#include <memory>

#include "registration.h"
#include "open3d/pipelines/registration/RobustKernel.h"
#include "open3d/pipelines/registration/TransformationEstimation.h"

namespace open3d {

namespace geometry {
class PointCloud;
}

namespace pipelines {
namespace registration {

class RegistrationResult;

class TransformationEstimationForColoredICP : public TransformationEstimation {
public:
    ~TransformationEstimationForColoredICP() override{};

    TransformationEstimationType GetTransformationEstimationType()
            const override {
        return estimationMethod;
    };
    /// \brief Constructor that takes as input a RobustKernel \params kernel Any
    /// of the implemented statistical robust kernel for outlier rejection.
    explicit TransformationEstimationForColoredICP(
            double lambda = 0.968,
            std::shared_ptr<RobustKernel> kern = std::make_shared<L2Loss>())
        : lambdaGeometric(lambda), kernel(std::move(kern)) {
        if (lambdaGeometric < 0 || lambdaGeometric > 1.0) {
            lambdaGeometric = 0.968;
        }
    }

public:
    double ComputeRMSE(const geometry::PointCloud &moving,
                       const geometry::PointCloud &fixed,
                       const CorrespondenceSet &correspondences) const override;
    Eigen::Matrix4d ComputeTransformation(
            const geometry::PointCloud &moving,
            const geometry::PointCloud &fixed,
            const CorrespondenceSet &correspondences) const override;

public:
    double lambdaGeometric = 0.968;
    /// shared_ptr to an Abstract RobustKernel that could mutate at runtime.
    std::shared_ptr<RobustKernel> kernel = std::make_shared<L2Loss>();

private:
    const TransformationEstimationType estimationMethod =
            TransformationEstimationType::ColoredICP;
};

/// \brief Function for Colored ICP registration.
///
/// This is implementation of following paper
/// J. Park, Q.-Y. Zhou, V. Koltun,
/// Colored Point Cloud Registration Revisited, ICCV 2017.
///
/// \param moving The moving point cloud.
/// \param fixed The fixed point cloud.
/// \param correspondenceMetric Maximum correspondence points-pair distance.
/// \param init Initial transformation estimation.
/// Default value: array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.],
/// [0., 0., 0., 1.]]).
/// \param estimation TransformationEstimationForColoredICP method. Can only
/// change the lambda value and the robust kernel used in the
/// optimization
/// \param criteria  Convergence criteria.
/// \param useInlierRatio a boolean flag to specify correspondenceMetric
///  as inlier ratio. Default value: false.
/// \param verbose a boolean flag to display progress information.
/// Default value: false.
RegistrationResult RegistrationColoredICP(
        const geometry::PointCloud &moving,
        const geometry::PointCloud &fixed,
        double correspondenceMetric,
        const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(),
        const TransformationEstimationForColoredICP &estimation =
                TransformationEstimationForColoredICP(),
        const ICPConvergenceCriteria &criteria = ICPConvergenceCriteria(),
        bool useInlierRatio = false,
        bool verbose = false);

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d