/* Copyright 2017-2020 The MathWorks, Inc. */
/////////////////////////////////////////////////////////////////////////////////////
// This file contains the built-in function used in visionLabelRangeData
//
// The MATLAB API is
//  [labels, numLabels] = visionLabelRangeData(xyzData, rangeData, distThreshold, angleThreshold, minClusterPoints, maxClusterPoints)
//
//  xyzData    : M-by-N-by-3 matrix, each entry gives x, y, z coordinates of a point
//  rangeData  : M-by-N-by-3 matrix, each entry gives range, pitch, yaw coordinates of a point (spherical coordinates)
//  distThreshold   : distance threshold. Two pixels belong to the same cluster if distance is below the threshold
//  angleThreshold  : angle threshold. Two pixels belong to the same cluster if beta angle is above the threshold
//  minClusterPoints  : minimum number of points in a valid segmented cluster. Points that are not part of a valid cluster have a 0 label
//  minClusterPoints  : maximum number of points in a valid segmented cluster. Points that are not part of a valid cluster have a 0 label
//
//  labels	   : M-by-N unit32 matrix, each entry gives an integer label. zero is reserved for bad (NaN) pixel.
//  numLabels  : scalar, number of clusters (label 0 is not included)
//
//  Data must be single or double
//
//  Reference:
//  Bogoslavskyi and Stachniss, Fast range image-based segmentation of sparse 3D Laser scans for online operation, 2017

#include "visionLabelRangeDataCore_api.hpp"
#include "visionLabelRangeData.hpp"


void visionLabelRangeData_single(float* xyz, float* range, int nRows, int nCols, float threshold, float angle_threshold, uint32_T minClusterPoints, uint32_T maxClusterPoints, uint32_T* labels, float* numClusters)
{
    labelClusters<float>(xyz, range, nRows, nCols, labels, numClusters, threshold, angle_threshold, minClusterPoints, maxClusterPoints);
}

void visionLabelRangeData_double(double* xyz, double* range, int nRows, int nCols, double threshold, double angle_threshold, uint32_T minClusterPoints, uint32_T maxClusterPoints, uint32_T* labels, double* numClusters)
{
    labelClusters<double>(xyz, range, nRows, nCols, labels, numClusters, threshold, angle_threshold, minClusterPoints, maxClusterPoints);
}

