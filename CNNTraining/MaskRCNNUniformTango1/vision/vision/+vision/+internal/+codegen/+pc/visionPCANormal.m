function normals = visionPCANormal(xyzPoints, indices, valid)
%   Compute normal vector with PCA for the portable code generation path.
%   simulation and shared library code generation version is found in
%   visionPCANormal.cpp
%
%   Inputs:
%   -------
%    xyzPoints  : input Mx3 matrix (single or double)
%    indices    : [NUM of Neighbors]xM uint32 matrix, 
%                   indices of neighbor points in xyzPoints.
%    valid      : Mx1 vector indicating the actual number of neighbors 
%                   in indices.
%   Outputs:
%   -------
%    normals        : Mx3 matrix (single or double), normals for each points
%
%   indices are 1-based. indices and valid are generated from multi-query in knnSearch
%   Note:
%    1. This function fills NaN if the actual number of neighbors (valid) is less than numNeighbors
%    2. An internal eigen solver is used for simulation and shared library code path,
%       which may produce different results from matlab builtin routine (LAPACK)

% Copyright 2021 The MathWorks, Inc.
%#codegen

    M = uint32(size(xyzPoints, 1));
    
    numNeighbors = uint32(size(indices, 1));
    
    normals = vision.internal.codegen.pc.PCANormalImpl(xyzPoints, indices, valid, M, numNeighbors);
end