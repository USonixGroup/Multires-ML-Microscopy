function [tform, inlierPoints1, inlierPoints2, status] ...
    = estimateGeometricTransform(matchedPoints1, matchedPoints2, ...
    transformType, varargin)
%estimateGeometricTransform Estimate geometric transformation from matching point pairs.
%
%   -----------------------------------------------------------------------
%   Use estimateGeometricTransform2D, which offers more functionality and
%   is recommended instead of estimateGeometricTransform.
%   -----------------------------------------------------------------------
%
%   tform = estimateGeometricTransform(matchedPoints1,matchedPoints2,
%   transformType) returns a 2-D geometric transform which maps the inliers
%   in matchedPoints1 to the inliers in matchedPoints2. matchedPoints1 and
%   matchedPoints2 can be M-by-2 matrices of [x,y] coordinates or objects
%   of any of the point feature types (cornerPoints, BRISKPoints,
%   SIFTPoints, SURFPoints, ORBPoints, KAZEPoints, MSERRegions).
%   transformType can be 'similarity', 'affine', or 'projective'. Outliers
%   in matchedPoints1 and matchedPoints2 are excluded by using the
%   M-estimator SAmple Consensus (MSAC) algorithm. The MSAC algorithm is a
%   variant of the Random Sample Consensus (RANSAC) algorithm. Returned
%   tform types are as follows:
%
%   -------------------------------------
%   transformType    |  tform object
%   -----------------|-------------------
%   similarity       |  affine2d
%   affine           |  affine2d
%   projective       |  projective2d
%   -------------------------------------
%
%   [tform,inlierPoints1,inlierPoints2] = estimateGeometricTransform(...)
%   additionally returns the corresponding inlier points in inlierPoints1
%   and inlierPoints2.
%
%   [tform,inlierPoints1,inlierPoints2,status] =
%   estimateGeometricTransform(...) additionally returns a status code with
%   the following possible values:
%
%     0: No error.
%     1: matchedPoints1 and matchedPoints2 do not contain enough points.
%     2: Not enough inliers have been found.
%
%   When the STATUS output is not given, the function will throw an error
%   if matchedPoints1 and matchedPoints2 do not contain enough points or
%   if not enough inliers have been found.
%
%   [...] = estimateGeometricTransform(matchedPoints1,matchedPoints2,
%   transformType,Name,Value) specifies additional
%   name-value pair arguments described below:
%
%   'MaxNumTrials'        A positive integer scalar specifying the maximum
%                         number of random trials for finding the inliers.
%                         Increasing this value will improve the robustness
%                         of the output at the expense of additional
%                         computation.
%
%                         Default value: 1000
%
%   'Confidence'          A numeric scalar, C, 0 < C < 100, specifying the
%                         desired confidence (in percentage) for finding
%                         the maximum number of inliers. Increasing this
%                         value will improve the robustness of the output
%                         at the expense of additional computation.
%
%                         Default value: 99
%
%   'MaxDistance'         A positive numeric scalar specifying the maximum
%                         distance in pixels that a point can differ from
%                         the projected location of its associated point.
%
%                         Default value: 1.5
%
%   Class Support
%   -------------
%   matchedPoints1 and matchedPoints2 can be double or single when
%   specified as an M-by-2 matrix.
%
%   EXAMPLE: Recover a transformed image using SURF feature points
%   ----------------------------------------------------------------
%   Iin  = imread('cameraman.tif');
%   imshow(Iin); title('Base image');
%   Iout = imresize(Iin, 0.7);
%   Iout = imrotate(Iout, 31);
%   figure;
%   imshow(Iout); title('Transformed image');
%
%   % Detect and extract features from both images
%   ptsIn  = detectSURFFeatures(Iin);
%   ptsOut = detectSURFFeatures(Iout);
%   [featuresIn,   validPtsIn] = extractFeatures(Iin,  ptsIn);
%   [featuresOut, validPtsOut] = extractFeatures(Iout, ptsOut);
%
%   % Match feature vectors
%   indexPairs = matchFeatures(featuresIn, featuresOut);
%   matchedPtsIn  = validPtsIn(indexPairs(:,1));
%   matchedPtsOut = validPtsOut(indexPairs(:,2));
%   figure;
%   showMatchedFeatures(Iin,Iout,matchedPtsIn,matchedPtsOut);
%   title('Matched SURF points, including outliers');
%
%   % Exclude the outliers and compute the transformation matrix
%   [tform,inlierPtsOut,inlierPtsIn] = estimateGeometricTransform(...
%        matchedPtsOut,matchedPtsIn,'similarity');
%   figure;
%   showMatchedFeatures(Iin,Iout,inlierPtsIn,inlierPtsOut);
%   title('Matched inlier points');
%
%   % Recover the original image Iin from Iout
%   outputView = imref2d(size(Iin));
%   Ir = imwarp(Iout, tform, 'OutputView', outputView);
%   figure;
%   imshow(Ir); title('Recovered image');
%
% See also estgeotform2d.

% References:
% [1] R. Hartley, A. Zisserman, "Multiple View Geometry in Computer
%     Vision," Cambridge University Press, 2003.
% [2] P. H. S. Torr and A. Zisserman, "MLESAC: A New Robust Estimator
%     with Application to Estimating Image Geometry," Computer Vision
%     and Image Understanding, 2000.

% Copyright 2013-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

reportError = (nargout ~= 4);
is2D = true;

[tform, inlierIdx, status] = ...
    vision.internal.geotrans.algEstimateGeometricTransform(...
    matchedPoints1, matchedPoints2, transformType, reportError,...
    'estimateGeometricTransform', is2D, varargin{:});

% Extract inlier points
if status == int32(0)
    inlierPoints1 = matchedPoints1(inlierIdx, :);
    inlierPoints2 = matchedPoints2(inlierIdx, :);
else
    inlierPoints1 = matchedPoints1([]);
    inlierPoints2 = matchedPoints2([]);
end

end