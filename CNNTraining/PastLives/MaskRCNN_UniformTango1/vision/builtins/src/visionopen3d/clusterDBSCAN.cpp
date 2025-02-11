///////////////////////////////////////////////////////////////////////////
// Copyright 2023-2024 The MathWorks, Inc.
///////////////////////////////////////////////////////////////////////////
// This file contains the built-in function used in cluster connected components
//
// The MATLAB API is
//  [labels] = lidarClusterDBSCAN(vertices, faces);
//
//  Outputs:
//  triangleClusters           :   vector containing the cluster index per triangle (int32)
//  clusterNTriangles          :   vector containing the number of triangles per cluster (single or double)
//  clusterArea                :   vector containing the surface area per cluster (double)
//
//  Inputs:
//  pointCloud (pointCloud.Location)   :   M-by-3 matrix (single or double). vertices has been validated to
//                                         be non-empty.
//  epsilon                            :   positive scalar (single or double). epsilon denotes the
//                                         radius for neighborhood search.
//  MinimumPoints                      :   positive scalar (int32). MinimumPoints denotes the minimum number
//                                         of points required for a cluster.

// For mxArray, matrix related functionality
// and sending data back to MATLAB
#include <mex.h>
#include "matrix/unique_mxarray_ptr.hpp"
#include "matrix.h"

#include <Eigen/Dense>
#include "open3d/geometry/PointCloud.h"
#include "open3d/geometry/TriangleMesh.h"

// For raising exceptions
#include <fl/except/MsgIDException.hpp>
///////////////////////////////////////////////////////////////////////////
// @mwBuiltinFunction       lidarClusterDBSCAN
// @mwDoesNotCallMATLAB     true
// @mwDoesNotRaiseWarnings  true
// @mwNargin                3
// @mwNargout               1
// @mwToolboxLocation vision/vision
BUILTIN_IMPLEMENTATION void lidarClusterDBSCAN(int nlhs,
                                          matrix::unique_mxarray_ptr plhs[],
                                          int nrhs,
                                          const mxArray* prhs[]) {
    using namespace open3d;

    // Read a point cloud locations from MATLAB
    const mxArray* location = prhs[0];
    size_t count = mxGetM(prhs[0]);
    double eps = mxGetScalar(prhs[1]);
    const mxArray* minPointsMxPtr = prhs[2];
    size_t* minPointsPtr = static_cast<size_t*>(mxGetData(minPointsMxPtr));
    size_t minPoints = minPointsPtr[0];

    // Create a point cloud in Open3D
    open3d::geometry::PointCloud o3dPtCloud;
    const size_t xOffset(0);
    const size_t yOffset(xOffset + count);
    const size_t zOffset(yOffset + count);
    if (mxGetClassID(prhs[0]) == mxSINGLE_CLASS) {
        real32_T* locationPoints = static_cast<real32_T*>(mxGetData(location));
        for (size_t i = 0; i < count; i++) {
            o3dPtCloud.points_.push_back(Eigen::Vector3d(locationPoints[i + xOffset],
                                                         locationPoints[i + yOffset],
                                                         locationPoints[i + zOffset]));
        }
    } else {
        real64_T* locationPoints = static_cast<real64_T*>(mxGetData(location));
        for (size_t i = 0; i < count; i++) {
            o3dPtCloud.points_.push_back(Eigen::Vector3d(locationPoints[i + xOffset],
                                                         locationPoints[i + yOffset],
                                                         locationPoints[i + zOffset]));
        }
    }

    std::vector<int> clusterLabels;
    clusterLabels = o3dPtCloud.ClusterDBSCAN(eps,minPoints);

    // Send the results back to MATLAB
    matrix::unique_mxarray_ptr mxClusterLabels;
    mxClusterLabels = matrix::create(clusterLabels.size(), 1, mxUINT64_CLASS, mxREAL);

    size_t* labels = (size_t*)mxGetData(mxClusterLabels.get());

    for (size_t i = 0; i < clusterLabels.size(); i++) {
        labels[i] = clusterLabels[i] + 1;
    }

    plhs[0] = std::move(mxClusterLabels);
}