function [tform, inlierIndex, status] ...
    = estimateGeometricTransform3D(matchedPoints1, matchedPoints2, ...
    transformType, varargin)
% estimateGeometricTransform3D Estimate 3-D geometric transformation from matching point pairs
%
%------------------------------------------------------------------------------
%   estimateGeometricTransform3D is not recommended. Use estgeotform3d instead.
%------------------------------------------------------------------------------
%
%   tform = estimateGeometricTransform3D(matchedPoints1, matchedPoints2,...
%   transformType) returns a 3-D geometric transformation which maps the
%   inliers in matchedPoints1 to the inliers in matchedPoints2.
%   matchedPoints1 and matchedPoints2 are M-by-3 matrices of [x,y,z]
%   coordinates. transformType can be 'rigid' or 'similarity'. Returned
%   tform types are as follows:
%
%   ------------------------------------
%   transformType    |  tform object
%   -----------------|------------------
%   'rigid'          |  rigid3d
%   'similarity'     |  affine3d
%   ------------------------------------
%
%   [tform, inlierIndex] = estimateGeometricTransform3D(...) additionally
%   returns inlierIndex, an M-by-1 logical vector containing 1 for inliers
%   and 0 for outliers. M is the total number of 3-D point pairs.
%
%   [tform, inlierIndex, status] = estimateGeometricTransform3D(...)
%   additionally returns a status code with the following possible values:
%
%   0: No error.
%   1: matchedPoints1 and matchedPoints2 do not contain enough points.
%   2: Not enough inliers have been found.
%
%   When the status output is not requested, the function will throw an
%   error if matchedPoints1 and matchedPoints2 do not contain enough points
%   or if not enough inliers have been found.
%
%   [...] = estimateGeometricTransform3D(matchedPoints1, matchedPoints2,...
%   transformType, Name, Value) specifies additional name-value pair
%   arguments described below:
%
%   'MaxNumTrials'        A positive integer scalar specifying the maximum
%                         number of random trials for finding the inliers.
%                         Increasing this value will improve the robustness
%                         of the output at the expense of additional
%                         computation.
%
%                         Default: 1000
%
%   'Confidence'          A numeric scalar, C, 0 < C < 100, specifying the
%                         desired confidence (in percentage) for finding
%                         the maximum number of inliers. Increasing this
%                         value will improve the robustness of the output
%                         at the expense of additional computation.
%
%                         Default: 99
%
%   'MaxDistance'         A positive numeric scalar specifying the maximum
%                         distance that a point can differ from the
%                         projected location of its associated point to be
%                         considered an inlier.
%
%                         Default: 1
%
%   Notes
%   -----
%   Outliers in matchedPoints1 and matchedPoints2 are excluded by using the
%   M-estimator SAmple Consensus (MSAC) algorithm. The MSAC algorithm is a
%   variant of the Random Sample Consensus (RANSAC) algorithm.
%
%   Class Support
%   -------------
%   matchedPoints1 and matchedPoints2 must be numeric.
%
%   Example: Estimate the transformation between 2 point clouds
%   -----------------------------------------------------------
%   ptCloud1 = pcread('teapot.ply');
%   ptCloud1 = pcdownsample(ptCloud1, 'random', 0.25);
%
%   % Create a transformation object
%   theta = 30; % degrees
%   rot = [cosd(theta)  sind(theta) 0; ...
%         -sind(theta)  cosd(theta) 0; ...
%               0           0       1];
%   trans = [0 0 0];
%   tform = rigid3d(rot, trans);
%
%   % Transform the point cloud
%   ptCloud2 = pctransform(ptCloud1, tform);
%
%   % Add noisy points in the range -2 to 2
%   noise1 = rescale(rand(1000,3), -2, 2);
%   ptCloud1 = pointCloud([ptCloud1.Location; noise1]);
%   noise2 = rescale(rand(1000,3), -2, 2);
%   ptCloud2 = pointCloud([ptCloud2.Location; noise2]);
%
%   % Visualize the noisy point clouds
%   figure
%   pcshowpair(ptCloud1, ptCloud2)
%   title('Point clouds with noisy points')
%
%   matchedPoints1 = ptCloud1.Location;
%   matchedPoints2 = ptCloud2.Location;
%
%   % Estimate the rigid transformation between the point clouds
%   [tformEst, inlierIndex] = estimateGeometricTransform3D(...
%       matchedPoints1, matchedPoints2, 'rigid');
%
%   % Get the inlier points
%   inliersPtCloud1 = transformPointsForward(tformEst,...
%       matchedPoints1(inlierIndex, :));
%   inliersPtCloud2 = matchedPoints2(inlierIndex, :);
%
%   % Visualize the inliers of the aligned point clouds
%   figure
%   pcshowpair(pointCloud(inliersPtCloud1), pointCloud(inliersPtCloud2))
%   title('Aligned point clouds')
%
% See also estgeotform3d, estgeotform2d, pcregistericp, rigidtform3d,
%          affinetform3d, ransac.

% Copyright 2020-2022 The MathWorks, Inc.

% References:
% [1] R. Hartley, A. Zisserman, "Multiple View Geometry in Computer
%     Vision," Cambridge University Press, 2003.
% [2] P. H. S. Torr and A. Zisserman, "MLESAC: A New Robust Estimator
%     with Application to Estimating Image Geometry," Computer Vision
%     and Image Understanding, 2000.

%#codegen

reportError = (nargout ~= 3);
is2D = false;

[tform, inlierIndex, status] = ...
    vision.internal.geotrans.algEstimateGeometricTransform(...
    matchedPoints1, matchedPoints2, transformType, reportError,...
    'estimateGeometricTransform3D', is2D, varargin{:});

end