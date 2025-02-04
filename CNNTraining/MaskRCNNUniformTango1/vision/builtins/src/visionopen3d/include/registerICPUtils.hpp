///////////////////////////////////////////////////////////////////////////
//  registerICPUtils.hpp
//
//  This file contains utilities used by pcregistericp for Codegen.
//
//  Copyright 2023 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////
#ifndef COMPILE_FOR_VISION_BUILTINS

#include "registration.h"
#include "generalizedICP.h"
#include "coloredICP.h"

#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/KDTreeSearchParam.h"
#include "open3d/pipelines/registration/TransformationEstimation.h"

namespace o3dRegistration = open3d::pipelines::registration;
namespace o3dGeometry = open3d::geometry;
//////////////////////////////////////////////////////////////////////////////
// Read input point cloud locations from an array.
//////////////////////////////////////////////////////////////////////////////
template <typename T>
o3dGeometry::PointCloud readPointCloud(const T* locationPoints, const size_t count){

    o3dGeometry::PointCloud ptCloud;

    size_t xOffset(0);
    size_t yOffset(xOffset + count);
    size_t zOffset(yOffset + count);

    for (size_t i = 0; i < count; i++) {
        auto point = Eigen::Vector3d(locationPoints[i + xOffset],
                                     locationPoints[i + yOffset],
                                     locationPoints[i + zOffset]);
        ptCloud.points_.push_back(point);
    }
    return ptCloud;
}

//////////////////////////////////////////////////////////////////////////////
// Read normals from an array.
//////////////////////////////////////////////////////////////////////////////
template <typename T>
std::vector<Eigen::Vector3d> readNormals(const T* normalsData, const size_t count){

    std::vector<Eigen::Vector3d> normals;

    size_t xOffset(0);
    size_t yOffset(xOffset + count);
    size_t zOffset(yOffset + count);

    for (size_t i = 0; i < count; i++) {
        auto normal = Eigen::Vector3d(normalsData[i + xOffset],
                                      normalsData[i + yOffset],
                                      normalsData[i + zOffset]);
        normals.push_back(normal);
    }

    return normals;
}

//////////////////////////////////////////////////////////////////////////////
// Read normalized colors from an array.
//////////////////////////////////////////////////////////////////////////////
std::vector<Eigen::Vector3d> readNormalizedColors(const real64_T* colorsData, const size_t count){

    std::vector<Eigen::Vector3d> colors;
    size_t xOffset(0);
    size_t yOffset(xOffset + count);
    size_t zOffset(yOffset + count);

    for (size_t i = 0; i < count; i++) {
        auto color = Eigen::Vector3d(colorsData[i + xOffset],
                                     colorsData[i + yOffset],
                                     colorsData[i + zOffset]);
        colors.push_back(color);
    }
    return colors;
}

//////////////////////////////////////////////////////////////////////////////
// Read initial transformation from an array.
//////////////////////////////////////////////////////////////////////////////
template <typename T>
Eigen::Matrix4d readInitTform(const T* tform){

    Eigen::Matrix4d initTform;

    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++)
            initTform(j,i) = static_cast<double>(tform[i+4*j]); // (j,i) instead of (i,j) to transpose
    }

    return initTform;
}

//////////////////////////////////////////////////////////////////////////////
// Construct ICPConvergenceCriteria.
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::ICPConvergenceCriteria
readConvergenceCriteria(const int maxIteration, const double relativeRotation,
                        const double relativeTranslation){

    // This is a flag to inform ICPConvergenceCriteria present in Registration.cpp
    // to measure convergence based on transformation (R & t) instead of default
    // metrics like relative fitness and relative RMSE.
    bool useTransformationTolerance = true;

    return o3dRegistration::ICPConvergenceCriteria(
        useTransformationTolerance, relativeTranslation,
        relativeRotation, maxIteration);
}

//////////////////////////////////////////////////////////////////////////////
// Grid average downsampling and estimation of normals
//////////////////////////////////////////////////////////////////////////////
std::shared_ptr<open3d::geometry::PointCloud> gridAverageDownsampling(open3d::geometry::PointCloud ptCloud,double voxelSize)
{
    auto ptCloudDown = ptCloud.VoxelDownSample(voxelSize);
    ptCloudDown->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(
        voxelSize * 2.0, 30));
    return ptCloudDown;
}
//////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D's API for point-to-point ICP.
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult registerUsingPointToPointICP(o3dGeometry::PointCloud moving,
                                                                 o3dGeometry::PointCloud fixed,
                                                                 const double metric, Eigen::Matrix4d initTform,
                                                                 o3dRegistration::ICPConvergenceCriteria convergenceCriteria,
                                                                 const boolean_T useInlierRatio, const boolean_T verbose){

    bool doEstimateScale = false;
    auto estimation = o3dRegistration::TransformationEstimationPointToPoint(doEstimateScale);

    return o3dRegistration::RegistrationICP(moving, fixed, metric,
                                            initTform, estimation, convergenceCriteria, useInlierRatio, verbose);
}

//////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D's API for point-to-plane ICP.
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult registerUsingPointToPlaneICP(o3dGeometry::PointCloud moving,
                                                                 o3dGeometry::PointCloud fixed,                                                                 
                                                                 const double metric, Eigen::Matrix4d initTform,
                                                                 o3dRegistration::ICPConvergenceCriteria convergenceCriteria,
                                                                 const boolean_T useInlierRatio, const boolean_T verbose){
   
    auto estimation = o3dRegistration::TransformationEstimationPointToPlane();

    return o3dRegistration::RegistrationICP(moving, fixed, metric,
                                            initTform, estimation, convergenceCriteria, useInlierRatio, verbose);
}

// //////////////////////////////////////////////////////////////////////////////
// // Helper function to call Open3D's API for plane-to-plane ICP (Generalized-ICP).
// //////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult registerUsingPlaneToPlaneICP(o3dGeometry::PointCloud moving,
                                                                 o3dGeometry::PointCloud fixed,                                                                 
                                                                 const double metric, Eigen::Matrix4d initTform,
                                                                 o3dRegistration::ICPConvergenceCriteria convergenceCriteria,
                                                                 const boolean_T useInlierRatio, const boolean_T verbose){
    
    auto estimation = o3dRegistration::TransformationEstimationForGeneralizedICP();

    return o3dRegistration::RegistrationGeneralizedICP(moving, fixed, metric,
                                                       initTform, estimation, convergenceCriteria, useInlierRatio, verbose);
}

// //////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D's API for color variant of point-to-plane ICP.
// This algorithm is inspired from "Open3D v0.17.0 cpp/open3d/pipelines/
// registration/ColoredICP.cpp". The hyperparameters have been modified after
// performing experiments on a set of chosen datasets.
// //////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult registerUsingPointToPlaneWithColorICP(o3dGeometry::PointCloud moving,
                                                                          o3dGeometry::PointCloud fixed,
                                                                          const std::vector<double> voxelSizes,
                                                                          const std::vector<int> iterations,
                                                                          Eigen::Matrix4d initTform,
                                                                          o3dRegistration::ICPConvergenceCriteria convergenceCriteria,
                                                                          const boolean_T useInlierRatio, const boolean_T verbose,
                                                                          const double relativeRotation,
                                                                          const double relativeTranslation,
                                                                          const double metric){

    int prevMaxIteration = 0;
    Eigen::Matrix4d trans = initTform;
    o3dRegistration::RegistrationResult finalResult;
    bool useTransformationTolerance = true;
    // Multi-Scale Registration (Coarser registration to finer registration).
    // The result of a coarse registration step is fed to the next finer
    // registration step as initial transformation.
    for (int i = 0; i < 3; ++i) {
        double voxelSize = voxelSizes[i];

        auto movingDownsampled = gridAverageDownsampling(moving, voxelSize);
        auto fixedDownsampled = gridAverageDownsampling(fixed, voxelSize);

        auto estimation = o3dRegistration::TransformationEstimationForColoredICP();

        auto result = o3dRegistration::RegistrationColoredICP(*movingDownsampled, *fixedDownsampled, metric,
                                                              trans, estimation, o3dRegistration::ICPConvergenceCriteria(useTransformationTolerance,
                                                              relativeTranslation, relativeRotation, iterations[i], prevMaxIteration),
                                                              useInlierRatio, verbose);

        prevMaxIteration = result.previousMaxIteration;
        trans = result.transformation;
        finalResult = result;
    }

    return finalResult;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper function to call Open3D based implementation for color variant of plane-to-plane ICP (Generalized-ICP).
// This algorithm is inspired from "PCL v1.13.0 /registration/src/gicp6d.cpp ". The hyperparameters have been
// modified after performing experiments on a set of chosen datasets. This implementation is based on Open3D's
// template for kd tree which can suffice the need of six dimenstional kd-tree.
//
//  M. Korn, M. Holzkothen, and J. Pauli. Color supported Generalized-ICP. In VISAPP, 2014.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult registerUsingPlaneToPlaneWithColorICP(o3dGeometry::PointCloud moving,
                                                                          o3dGeometry::PointCloud fixed,                                                                          
                                                                          const int maxIteration,
                                                                          Eigen::Matrix4d initTform,
                                                                          o3dRegistration::ICPConvergenceCriteria convergenceCriteria,
                                                                          const boolean_T useInlierRatio, const boolean_T verbose,
                                                                          const double relativeRotation,
                                                                          const double relativeTranslation,
                                                                          const double metric){

    int prevMaxIteration = 0;
    bool useTransformationTolerance = true;
    auto estimation = o3dRegistration::TransformationEstimationForGeneralizedICP();

    return o3dRegistration::RegistrationGeneralizedICP(moving, fixed, metric,
    initTform, estimation, o3dRegistration::ICPConvergenceCriteria(useTransformationTolerance, 
    relativeTranslation, relativeRotation, maxIteration, prevMaxIteration, true), useInlierRatio, verbose);
}

//////////////////////////////////////////////////////////////////////////////
// Helper function to process ICP algorithm.
//////////////////////////////////////////////////////////////////////////////
o3dRegistration::RegistrationResult processICPImpl(o3dGeometry::PointCloud moving,
                                                   o3dGeometry::PointCloud fixed,
                                                   const double metric, Eigen::Matrix4d initTform,
                                                   const int maxIteration, const double relativeRotation,
                                                   const double relativeTranslation,
                                                   const boolean_T useInlierRatio,
                                                   const std::string method,
                                                   const std::vector<double> voxelSizes,
                                                   const std::vector<int> iterations){

    o3dRegistration::ICPConvergenceCriteria convergenceCriteria =
        readConvergenceCriteria(maxIteration, relativeRotation, relativeTranslation);
    
    // Verbose is not supported for code generation
    const boolean_T verbose = false;

    if(method == "pointToPoint") {
        return registerUsingPointToPointICP(moving, fixed, metric,
                                            initTform, convergenceCriteria, useInlierRatio, verbose);
    }else if(method == "pointToPlane") {
        return registerUsingPointToPlaneICP(moving, fixed, metric,
                                            initTform, convergenceCriteria, useInlierRatio, verbose);
   }else if(method=="planeToPlane") {
        return registerUsingPlaneToPlaneICP(moving, fixed,
                                            metric, initTform, convergenceCriteria,
                                            useInlierRatio, verbose);
    }else if(method=="pointToPlaneWithColor") {
        return registerUsingPointToPlaneWithColorICP(moving, fixed,
                                                     voxelSizes, iterations,
                                                     initTform,
                                                     convergenceCriteria,
                                                     useInlierRatio, verbose,
                                                     relativeRotation,
                                                     relativeTranslation,
                                                     metric);
    }else {
        return registerUsingPlaneToPlaneWithColorICP(moving, fixed,
                                                     maxIteration,
                                                     initTform,
                                                     convergenceCriteria,
                                                     useInlierRatio, verbose,
                                                     relativeRotation,
                                                     relativeTranslation,
                                                     metric);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Package outputs to send back to MATLAB.
//////////////////////////////////////////////////////////////////////////////
double packageOutputs(o3dRegistration::RegistrationResult regResult,
                      double* tform){

    // Package transformation output.
    Eigen::Matrix4d_u T = regResult.transformation;

    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++){
            tform[i+4*j] = T(j,i); // (j,i) instead of (i,j) to transpose
        }
    }

    // Package rmse output.
    double rmse = static_cast<double>(regResult.inlierRMSE);
    return rmse;
}

#endif