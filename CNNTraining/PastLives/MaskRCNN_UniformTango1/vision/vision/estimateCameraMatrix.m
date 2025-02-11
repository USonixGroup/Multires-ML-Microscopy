function [camMatrix, reprojectionErrors] = estimateCameraMatrix(imagePoints, worldPoints)
% estimateCameraMatrix estimate camera projection matrix
%
%---------------------------------------------------------------------------------
%   estimateCameraMatrix is not recommended. Use estimateCameraProjection instead.
%---------------------------------------------------------------------------------
%
%   camMatrix = estimateCameraMatrix(imagePoints, worldPoints) returns the
%   projection matrix of an uncalibrated camera from corresponding points
%   using Direct Linear Transform (DLT) algorithm. imagePoints is an M-by-2
%   array of [x,y] intrinsic image coordinates of keypoints. M is the
%   number of key points, where M >= 6. worldPoints is an M-by-3 array of
%   [x,y,z] world coordinates of key points that are not coplanar.
%   camMatrix is a 4-by-3 matrix that projects a 3-D world point in
%   homogeneous coordinates into the image.
%
%   [..., reprojectionErrors] =  estimateCameraMatrix(...) additionally
%   returns reprojection errors for the world points. reprojectionErrors is
%   an M-by-1 vector containing the reprojection error for each world
%   point.
%
%   Class Support
%   -------------
%   worldPoints and imagePoints must be double or single.
%
%   Notes 
%   ----- 
%   The estimation process assumes the camera has negligible lens
%   distortion. If not, calibrate the camera and use
%   estimateWorldCameraPose and cameraMatrix functions instead.
%
%   Example: Estimate camera matrix for a depth camera
%   --------------------------------------------------
%  % Load a point cloud captured by Kinect
%  ld = load('object3d.mat');
%  ptCloud = ld.ptCloud;
%
%  % Sample 1% of points
%  stepSize = 100;
%  indices = 1:stepSize:ptCloud.Count;
%  tempPtCloud = select(ptCloud, indices);
%  [Y, X] = ind2sub([size(ptCloud.Location,1),...
%                       size(ptCloud.Location,2)], indices);
%
%  % Remove the invalid points and set the coordinates of 3D-2D
%  % corresponding points
%  [tempPtCloud, validIndices] = removeInvalidPoints(tempPtCloud);
%  worldPoints = tempPtCloud.Location;
%  imagePoints = [X(validIndices)' Y(validIndices)'];
%
%  % Estimate the camera projection matrix
%  camMatrix = estimateCameraMatrix(imagePoints, worldPoints);
%
%  % Define a query point and number of neighbors
%  point = [0.4 0.3 0.2];
%  K = 10;
%
%  % Get the indices and distances of 10 nearest points
%  [indices, dists] = findNearestNeighbors(ptCloud, point, K, camMatrix);
%
%   See also cameraProjection, estimateCameraParameters, estworldpose,
%       estimateEssentialMatrix, estimateFundamentalMatrix.

%   Copyright 2018-2022 MathWorks, Inc.

% References:
%
% Hartley, Richard, and Andrew Zisserman. Multiple View Geometry in
% Computer Vision. Vol. 2. Cambridge, 2000.

[camMatrix, reprojectionErrors] = vision.internal.calibration.estimateCameraMatrixImpl( ...
    imagePoints, worldPoints, mfilename);

% Transpose camMatrix to output 4x3 matrix.
camMatrix = camMatrix';