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
#define _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING

#include <Eigen/Core>
#include <tuple>
#include <vector>

#include "open3d/pipelines/registration/CorrespondenceChecker.h"
#include "open3d/pipelines/registration/TransformationEstimation.h"
#include "open3d/utility/Eigen.h"
#include "open3d/utility/Optional.h"

namespace open3d {

namespace geometry {
class PointCloud;
}

namespace pipelines {
namespace registration {
class Feature;

/// \class ICPConvergenceCriteria
///
/// \brief Class that defines the convergence criteria of ICP.
///
/// ICP algorithm stops if the relative change of fitness and rmse hit
/// \p relativeFitness and \p relativeRMSE individually, or the iteration
/// number exceeds \p maxIteration.
class ICPConvergenceCriteria {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param deltaRelFitness If relative change (difference) of fitness score
    /// is lower than deltaRelFitness, the iteration stops. \param
    /// deltaRelRMSE If relative change (difference) of inliner RMSE score is
    /// lower than deltaRelRMSE, the iteration stops. \param maxIter
    /// Maximum iteration before iteration stops.
    ICPConvergenceCriteria(double deltaRelFitness = 1e-6,
                           double deltaRelRMSE = 1e-6,
                           int maxIter = 30)
        : relativeFitness(deltaRelFitness),
          relativeRMSE(deltaRelRMSE),
          maxIteration(maxIter),
          useTransformationMetric(false),
          usePlaneToPlaneWithColor(false),
          relativeTranslation(0.01),
          relativeRotation(0.05),
          previousMaxIteration(0) {}

    ICPConvergenceCriteria(bool useMetric, 
                           double relTranslation = 0.01,
                           double relRotation = 0.05,
                           int maxIter = 30,
                           int prevMaxIter = 0,
                           bool usePlanetoplaneWithColor = false)
        : relativeFitness(1e-6), relativeRMSE(1e-6),
          maxIteration(maxIter),
          useTransformationMetric(useMetric),
          usePlaneToPlaneWithColor(usePlanetoplaneWithColor),
          relativeTranslation(relTranslation),
          relativeRotation(relRotation),
          previousMaxIteration(prevMaxIter){}

    ~ICPConvergenceCriteria() {}

public:
    /// If relative change (difference) of fitness score is lower than
    /// `deltaRelFitness`, the iteration stops.
    double relativeFitness;
    /// If relative change (difference) of inlier RMSE score is lower than
    /// `deltaRelRMSE`, the iteration stops.
    double relativeRMSE;
    /// Maximum iteration before iteration stops.
    int maxIteration;
    /// Boolean flag to specify convergence criteria using transformation 
    /// metrics such as relative translation and relative rotation.
    bool useTransformationMetric;
    /// Boolean flag to specify that current method used is plane-to-plane
    /// with color.
    bool usePlaneToPlaneWithColor;
    /// If relative change (difference) of translation and rotation are 
    /// lower than `relTranslation` and `relTranslation` 
    /// respectively, iteration stops.
    double relativeTranslation;
    double relativeRotation;
    /// For multi-scale registration used in point-to-plane and plane-to-plane 
    /// with color variants, the previous maximum iteration will serve as 
    /// the starting point for the following maximum iterations.
    int previousMaxIteration;
};

/// \class Correspondence
///
/// \brief Class to manage point correspondences between the two input point clouds
///
/// Correspondence class is used to manage the point correspondences during 
/// outlier rejection based on inlier-ratio. Specifically, this class is helpful 
/// during the partial-sort of correspondence indices based on inlier distances.
class Correspondence{
    public:
        Correspondence(int queryIdx, int resultIdx, double dist):
                       queryIndex(queryIdx), resultIndex(resultIdx),
                       distance(dist){}

        int queryIndex;
        int resultIndex;
        double distance;

        bool operator < (const Correspondence& rhs) const {
            return distance < rhs.distance;
        }
        bool operator > (const Correspondence& rhs) const {
            return distance > rhs.distance;
        }
        bool operator == (const Correspondence& rhs) const {
            return distance == rhs.distance;
        }
        bool operator <= (const Correspondence& rhs) const {
            return distance <= rhs.distance;
        }
        bool operator >= (const Correspondence& rhs) const {
            return distance >= rhs.distance;
        }
};

/// \class RegistrationResult
///
/// Class that contains the registration results.
class RegistrationResult {
public:
    /// \brief Parameterized Constructor.
    ///
    /// \param transformation The estimated transformation matrix.
    RegistrationResult(
            const Eigen::Matrix4d &tform = Eigen::Matrix4d::Identity())
        : transformation(tform), inlierRMSE(0.0), fitness(0.0),
          previousMaxIteration(0){}
    ~RegistrationResult() {}
    bool IsBetterRANSACThan(const RegistrationResult &other) const {
        return fitness > other.fitness || (fitness == other.fitness &&
                                             inlierRMSE < other.inlierRMSE);
    }

public:
    /// The estimated transformation matrix.
    Eigen::Matrix4d_u transformation;
    /// Correspondence set between moving and fixed point cloud.
    CorrespondenceSet correspondenceSet;
    /// RMSE of all inlier correspondences. Lower is better.
    double inlierRMSE;
    /// For ICP: the overlapping area (# of inlier correspondences / # of points
    /// in fixed). Higher is better.
    /// For RANSAC: inlier ratio (# of inlier correspondences / # of
    /// all correspondences)
    double fitness;
    /// For multi-scale registration used in point-to-plane and plane-to-plane 
    /// with color variants, the previous maximum iteration will serve as 
    /// the starting point for the following maximum iterations.
    int previousMaxIteration;
};

/// \brief Functions for ICP registration.
///
/// \param moving The moving point cloud.
/// \param fixed The fixed point cloud.
/// \param correspondenceMetric Maximum correspondence points-pair distance
/// or inlier ratio depending on useInlierRatio flag.
/// \param init Initial transformation estimation.
///  Default value: array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.],
///  [0., 0., 0., 1.]])
/// \param estimation Estimation method.
/// \param criteria Convergence criteria.
/// \param useInlierRatio a boolean flag to specify correspondenceMetric 
///  as inlier ratio. Default value: false.
/// \param verbose a boolean flag to display progress information.
 /// Default value: false.
RegistrationResult RegistrationICP(
        const geometry::PointCloud &moving,
        const geometry::PointCloud &fixed,
        double correspondenceMetric,
        const Eigen::Matrix4d &init = Eigen::Matrix4d::Identity(),
        const TransformationEstimation &estimation =
                TransformationEstimationPointToPoint(false),
        const ICPConvergenceCriteria &criteria = ICPConvergenceCriteria(),
        bool useInlierRatio = false,
        bool verbose = false);

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d
