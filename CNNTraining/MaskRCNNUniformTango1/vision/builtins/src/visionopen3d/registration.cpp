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

#define _SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING
#if defined(_MSC_VER)
    #pragma warning ( disable : 4267 )
#endif

#include "registration.h"

#ifdef COMPILE_FOR_VISION_BUILTINS  // Used only for simulation 
#include "i18n/MessageCatalog.hpp"
#include <fl/except/MsgIDException.hpp>
#include "resources/vision/pointcloud.hpp"

// Unicode streams for output to the command window
#include "services/io/unicode_stream.hpp"
#endif

#include "open3d/geometry/KDTreeFlann.h"
#include "open3d/geometry/PointCloud.h"
#include "open3d/pipelines/registration/Feature.h"
#include "open3d/utility/Helper.h"
#include "open3d/utility/Parallel.h"
/* TMW Edit: Added the following header for random number generation as per 
   changes related to Open3D 0.16.1 upgrade.*/
#include "open3d/utility/Random.h"

#include <algorithm>  // std::partial_sort
#include <iostream>   // std::cout

#define PRINTMESSAGE(msgIDObj) services::io::uout() << \
                               fl::i18n::MessageCatalog::get_message(msgIDObj) << \
                               std::endl;

namespace open3d {
namespace pipelines {
namespace registration {

// Helper function to find correspondences using distance metric and 
// to compute inlier_rmse.
static RegistrationResult GetRegistrationResultAndCorrespondencesUsingDistance(
        const Eigen::MatrixXd &moving,
        const geometry::KDTreeFlann &fixedKdtree,
        double maxCorrespondenceDistance,
        const Eigen::Matrix4d &transformation) {
    RegistrationResult result(transformation);
    if (maxCorrespondenceDistance <= 0.0) {
        return result;
    }

    double error = 0.0;

#pragma omp parallel
    {
        double errorPrivate = 0.0;
        CorrespondenceSet correspondenceSetPrivate;
#pragma omp for nowait
        for (int i = 0; i < (int)moving.cols(); i++) {
            std::vector<int> indices(1);
            std::vector<double> dists(1);
            const Eigen::VectorXd &point = moving.col(i);
            if (fixedKdtree.SearchHybrid(point, maxCorrespondenceDistance,
                                           1, indices, dists) > 0) {
                errorPrivate += dists[0];
                correspondenceSetPrivate.push_back(
                        Eigen::Vector2i(i, indices[0]));
            }
        }
#pragma omp critical(GetRegistrationResultAndCorrespondencesUsingDistance)
        {
            for (int i = 0; i < (int)correspondenceSetPrivate.size(); i++) {
                result.correspondenceSet.push_back(
                        correspondenceSetPrivate[i]);
            }
            error += errorPrivate;
        }
    }

    size_t numCorrespondences = result.correspondenceSet.size();
    if (numCorrespondences < 3) {
        #ifdef COMPILE_FOR_VISION_BUILTINS
        throw fl::except::MakeException(vision::pointcloud::increaseInlierDistance());
        #endif
    } else {
        result.fitness = (double)numCorrespondences / (double)moving.cols();
        result.inlierRMSE = std::sqrt(error / (double)numCorrespondences);
    }
    return result;
}

// Helper function to find correspondences using inlier ratio and 
// to compute inlier_rmse.
static RegistrationResult GetRegistrationResultAndCorrespondencesUsingInlierRatio(
        const Eigen::MatrixXd &moving,
        const geometry::KDTreeFlann &fixedKdtree,
        double inlierRatio,
        const Eigen::Matrix4d &transformation) {

    RegistrationResult result(transformation);
    if (inlierRatio <= 0.0) {
        return result;
    }

    int numMovingPoints = static_cast<int>(moving.cols());

    // Create and initialize a vector of correspondences. Initialization is 
    // required for the begin() and end() iterators to point to 0 and  
    // numMovingPoints locations respectively.
    int defaultIdx = 1;
    double defaultDistance = 0;
    Correspondence defaultCorrespondence {defaultIdx, defaultIdx, defaultDistance};
    std::vector<Correspondence> correspondences(numMovingPoints, defaultCorrespondence);

    // In this parallel for loop, data is written in the vector using array indexing 
    // instead of STL push_back or emplace_back to avoid data racing condition.
#pragma omp parallel
    {
#pragma omp for nowait
        for (int i = 0; i < numMovingPoints; i++) {
            const Eigen::VectorXd &point = moving.col(i);
            std::vector<int> resultIdx(1);
            std::vector<double> resultDistance(1);
            const int numNeighbors = 1;
            if (fixedKdtree.SearchKNN(point, numNeighbors, resultIdx, resultDistance) > 0) {
                correspondences[i] = Correspondence(i, resultIdx[0], resultDistance[0]);
            }
        }
    }

    int numInliers = std::max(1, static_cast<int>(round(inlierRatio * numMovingPoints)));
    if(numInliers != numMovingPoints){
        // Find k closest correspondences where k = numInliers.
        std::partial_sort(correspondences.begin(), correspondences.begin()+numInliers, correspondences.end());
    }
    
    double error = 0.0;
    for (int i = 0; i < numInliers; i++) {
        error += correspondences.at(i).distance;
        result.correspondenceSet.push_back(
                   Eigen::Vector2i(correspondences.at(i).queryIndex,
                                   correspondences.at(i).resultIndex));
    }

    size_t numCorrespondences = result.correspondenceSet.size();
    if (numCorrespondences < 3) {
        #ifdef COMPILE_FOR_VISION_BUILTINS
        throw fl::except::MakeException(vision::pointcloud::increaseInlierRatio());
        #endif
    } else {
        result.fitness = (double)numCorrespondences / (double)moving.cols();
        result.inlierRMSE = std::sqrt(error / (double)numCorrespondences);
    }

    correspondences.clear();
    return result;
}

// Wrapper function to switch between using inlier ratio and distance metrics 
// depending on useInlierRatio flag.
static RegistrationResult GetRegistrationResultAndCorrespondences(
        const Eigen::MatrixXd &moving,
        const geometry::KDTreeFlann &fixedKdtree,
        double metric,
        const Eigen::Matrix4d &transformation,
        bool useInlierRatio = false) {

    if(useInlierRatio){
        return GetRegistrationResultAndCorrespondencesUsingInlierRatio(
                    moving, fixedKdtree, metric, transformation);
    } else {
        return GetRegistrationResultAndCorrespondencesUsingDistance(
                    moving, fixedKdtree, metric, transformation);
    }
}

// Helper function to determine convergence using tolerance criteria
// on relative translation and relative rotation.
inline bool hasTransformationConverged(const int i, 
                                       const RegistrationResult &currResult,
                                       std::vector<Eigen::Vector3d> &ts,
                                       std::vector<Eigen::Quaterniond> &qs,
                                       const ICPConvergenceCriteria &criteria,
                                       bool verbose = false) {
    
    // Update the quaternion and translation buffers with the latest 
    // transformation estimate.
    Eigen::Matrix4d T = currResult.transformation;
    Eigen::Matrix3d rotationMatrix = T.block<3,3>(0,0);
    Eigen::Vector3d translationVector(T(0,3), T(1,3), T(2,3));
    
    qs[i+1] = Eigen::Quaterniond(rotationMatrix);
    ts[i+1] = translationVector;

    // Compute the average difference between the transformations
    // in the last three iterations.
    double dr = 0;
    double dt = 0;
    int count = 0;
    for (int j = std::max(i-2, 0); j <= i; j++, count++) {

        // Find change in translation.
        double tdiff = (ts[j+1] - ts[j]).norm();

        // Find change in rotation in degrees from the quaternion buffer.
        constexpr double pi = 3.14159265;
        Eigen::Quaterniond q1 = qs[j];
        Eigen::Quaterniond q2 = qs[j+1];
        double cosTheta = q1.dot(q2)/(q1.norm()*q2.norm());
        double rdiff = acos(std::min(1.0, std::max(-1.0, cosTheta))) * 180.0 / pi;

        // Accumulate changes.
        dr += rdiff;
        dt += tdiff;

        if (verbose && j == i){
            #ifdef COMPILE_FOR_VISION_BUILTINS
            PRINTMESSAGE(vision::pointcloud::checkConverge(std::to_string(tdiff),
                std::to_string(rdiff), std::to_string(currResult.inlierRMSE)));
            #endif
        }
    }
    dr /= count;
    dt /= count;

    // Check for convergence.
    bool hasRotationConverged = dr <= criteria.relativeRotation;
    bool hasTranslationConverged = dt <= criteria.relativeTranslation;
    return hasRotationConverged && hasTranslationConverged;
}

// Default helper function from Open3D to determine convergence using 
// tolerance criteria on relative fitness and relative rmse.
inline bool hasRegistrationResultConverged(const RegistrationResult &prevResult, 
                                           const RegistrationResult &currResult, 
                                           const ICPConvergenceCriteria &criteria) {    
    
    bool hasFitnessConverged = std::abs(prevResult.fitness - currResult.fitness)
                                            < criteria.relativeFitness;
    bool hasRMSEConverged = std::abs(prevResult.inlierRMSE - currResult.inlierRMSE)
                                            < criteria.relativeRMSE;

    return hasFitnessConverged && hasRMSEConverged;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Augment open3D point cloud into Eigen matrix point cloud based on number of dimensions
/////////////////////////////////////////////////////////////////////////////////////////
static Eigen::MatrixXd augmentPointCloudtoEigenMatrix(const geometry::PointCloud &ptCloud, size_t numDimensions)
{
    Eigen::MatrixXd augmentedPtCloud;
    size_t col = ptCloud.points_.size();
    if (numDimensions == 6) {
        // Augment open3D point cloud into Eigen matrix point cloud with color. (X Y Z L A B)
        augmentedPtCloud.resize(6,col);

        augmentedPtCloud.block(0,0,3,col) = Eigen::Map<const Eigen::MatrixXd>(
            (const double *)((const geometry::PointCloud &)ptCloud).points_.data(),
            3,((const geometry::PointCloud &)ptCloud).points_.size());

        augmentedPtCloud.block(3,0,3,col) = Eigen::Map<const Eigen::MatrixXd>(
            (const double *)((const geometry::PointCloud &)ptCloud).colors_.data(),
            3,((const geometry::PointCloud &)ptCloud).points_.size());
    } else {
        // Augment open3D point cloud into Eigen matrix point cloud. (X Y Z)
        augmentedPtCloud.resize(3,col);

        augmentedPtCloud.block(0,0,3,col) = Eigen::Map<const Eigen::MatrixXd>(
            (const double *)((const geometry::PointCloud &)ptCloud).points_.data(),
            3,((const geometry::PointCloud &)ptCloud).points_.size());
    }
    return augmentedPtCloud;
}

RegistrationResult RegistrationICP(
        const geometry::PointCloud &moving,
        const geometry::PointCloud &fixed,
        double correspondenceMetric,
        const Eigen::Matrix4d &init /* = Eigen::Matrix4d::Identity()*/,
        const TransformationEstimation &estimation
        /* = TransformationEstimationPointToPoint(false)*/,
        const ICPConvergenceCriteria
                &criteria /* = ICPConvergenceCriteria()*/,
        bool useInlierRatio /*= false*/,
        bool verbose /*= false*/) {
    
    Eigen::Matrix4d transformation = init;
    size_t kdtreeDimensions = 3;
    Eigen::MatrixXd augmentedMovingPtCloud;
    geometry::KDTreeFlann kdtree;
    RegistrationResult result;
    geometry::PointCloud pcd = moving;
    if (!init.isIdentity()) {
        pcd.Transform(init);
    };
    int currentIteration = 0;
    // Create and initialize buffers to store translation and rotation
    // (quaternion) vectors.
    int bufferSize = criteria.maxIteration + 1;
    std::vector<Eigen::Vector3d> ts(bufferSize, Eigen::Vector3d::Zero());
    std::vector<Eigen::Quaterniond> qs(bufferSize, Eigen::Quaterniond(1,0,0,0));
    
    if ((estimation.GetTransformationEstimationType() ==
                 TransformationEstimationType::GeneralizedICP) && 
        criteria.usePlaneToPlaneWithColor) {
        // Color variant of plane-to-plane registration.
        kdtreeDimensions = 6;
        Eigen::MatrixXd augmentedFixedPtCloud = augmentPointCloudtoEigenMatrix(fixed, kdtreeDimensions);
        kdtree.SetMatrixData(augmentedFixedPtCloud);

        augmentedMovingPtCloud = augmentPointCloudtoEigenMatrix(pcd, kdtreeDimensions);
        result = GetRegistrationResultAndCorrespondences(augmentedMovingPtCloud, kdtree,
                 correspondenceMetric, transformation, useInlierRatio);

        #ifdef COMPILE_FOR_VISION_BUILTINS
        fl::ustring msgTxt;
        #endif
        for (int i = 0; i < criteria.maxIteration; i++) {
            currentIteration = criteria.previousMaxIteration + i + 1;
            if(verbose){
                #ifdef COMPILE_FOR_VISION_BUILTINS
                std::cout << "\n--------------------------------------------\n";
                PRINTMESSAGE(vision::pointcloud::icpIteration(std::to_string(currentIteration)));
                #endif
            }
            Eigen::Matrix4d update = estimation.ComputeTransformation(
                    pcd, fixed, result.correspondenceSet);

            transformation = update * transformation;
            pcd.Transform(update);
            augmentedMovingPtCloud = augmentPointCloudtoEigenMatrix(pcd, kdtreeDimensions);
            RegistrationResult backup = result;
            result = GetRegistrationResultAndCorrespondences(augmentedMovingPtCloud, kdtree,
                 correspondenceMetric, transformation, useInlierRatio);

            bool hasConverged;
            if(criteria.useTransformationMetric){

                hasConverged = hasTransformationConverged(i, result, ts, qs,
                                                          criteria, verbose);
            } else {

                hasConverged = hasRegistrationResultConverged(backup, result,
                                                              criteria);
            }

            result.previousMaxIteration = currentIteration;

            if(hasConverged){
                if(verbose){
                    #ifdef COMPILE_FOR_VISION_BUILTINS
                    std::cout << "\n--------------------------------------------\n";
                    PRINTMESSAGE(vision::pointcloud::icpSummary(std::to_string(currentIteration)));
                    #endif
                }
                break;
            }
        }
    } else {
    // Color variant of point-to-plane registration and other non-colored 
    // variants of ICP registration.
    kdtree.SetGeometry(fixed);
    augmentedMovingPtCloud = augmentPointCloudtoEigenMatrix(pcd, kdtreeDimensions);
    result = GetRegistrationResultAndCorrespondences(augmentedMovingPtCloud, kdtree,
                 correspondenceMetric, transformation, useInlierRatio);

    #ifdef COMPILE_FOR_VISION_BUILTINS
    fl::ustring msgTxt;
    #endif
    for (int i = 0; i < criteria.maxIteration; i++) {
        currentIteration = criteria.previousMaxIteration + i + 1;
        if(verbose){
            #ifdef COMPILE_FOR_VISION_BUILTINS
            std::cout << "\n--------------------------------------------\n";
            PRINTMESSAGE(vision::pointcloud::icpIteration(std::to_string(currentIteration)));
            #endif
        }
        Eigen::Matrix4d update = estimation.ComputeTransformation(
                pcd, fixed, result.correspondenceSet);

        transformation = update * transformation;
        pcd.Transform(update);
        augmentedMovingPtCloud = augmentPointCloudtoEigenMatrix(pcd, kdtreeDimensions);
        RegistrationResult backup = result;
        result = GetRegistrationResultAndCorrespondences(augmentedMovingPtCloud, kdtree,
                 correspondenceMetric, transformation, useInlierRatio);

        bool hasConverged;
        if(criteria.useTransformationMetric){

            hasConverged = hasTransformationConverged(i, result, ts, qs, 
                                                      criteria, verbose);
        } else {

            hasConverged = hasRegistrationResultConverged(backup, result,
                                                          criteria);
        }

        result.previousMaxIteration = currentIteration;

        if(hasConverged){
            if(verbose){
                #ifdef COMPILE_FOR_VISION_BUILTINS
                std::cout << "\n--------------------------------------------\n";
                PRINTMESSAGE(vision::pointcloud::icpSummary(std::to_string(currentIteration)));
                #endif
            }
            break;
        }
    }
    }
    return result;
}

}  // namespace registration
}  // namespace pipelines
}  // namespace open3d