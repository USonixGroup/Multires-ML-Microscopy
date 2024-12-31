%TRIANGULATE Find 3-D locations of matching points in stereo images.
%
%   worldPoints = TRIANGULATE(matchedPoints1, matchedPoints2, stereoParams)
%   returns 3-D locations of matching pairs of undistorted image points
%   in stereo images. worldPoints is an M-by-3 matrix containing 3-D
%   coordinates relative to the optical center of camera 1. matchedPoints1
%   and matchedPoints2 can be M-by-2 matrices of [x,y] coordinates,
%   cornerPoints objects, SIFTPoints objects, SURFPoints objects, 
%   MSERRegions objects, or BRISKPoints objects. stereoParams is a 
%   stereoParameters object.
%
%   worldPoints = TRIANGULATE(matchedPoints1, matchedPoints2,
%     cameraMatrix1, cameraMatrix2) returns the 3-D locations in a world
%   coordinate system defined by 4-by-3 camera projection matrices:
%   cameraMatrix1 and cameraMatrix2, which map a 3-D world point in
%   homogeneous coordinates onto the corresponding image points.
%
%   [worldPoints, reprojectionErrors] = TRIANGULATE(...) additionally
%   returns reprojection errors for the world points. reprojectionErrors is
%   an M-by-1 vector containing the average reprojection error for each
%   world point.
%
%   [worldPoints, reprojectionErrors, validIndex] = TRIANGULATE(...)
%   additionally returns the indices of valid world points that are located
%   in front of both cameras. validIndex is an M-by-1 logical array denoting
%   the validity of each world point.
%
%   Notes
%   -----
%   - The function does not account for lens distortion. You can either
%     undistort the images using the undistortImage function before
%     detecting the points, or you can undistort the points themselves
%     using the undistortPoints function.
%
%   - If you pass a stereoParameters object to the function, the world
%     coordinate system has the origin at the optical center of camera 1,
%     with the X-axis pointing to the right, the Y-axis pointing down, and
%     the Z-axis pointing away from the camera.
%
%   - If you pass camera matrices to the function, the world coordinate
%     system is defined by those matrices. In both cases the function uses
%     a right-handed coordinate system.
%
%   - The validity of a world point with respect to a camera is determined
%     by projecting the world point onto the image using the camera matrix
%     and homogeneous coordinates. The world point is valid if the resulting
%     scale factor is positive.
%
%   Class Support
%   -------------
%   matchedPoints1 and matchedPoints2 can be double, single, or any of the
%   <a href="matlab:helpview('vision','pointfeaturetypes')">point feature types</a>. stereoParams must be a stereoParameters object.
%   cameraMatrix1 and cameraMatrix2 must be real and non-sparse numeric matrices.
%   worldPoints is double if matchedPoints1 and matchedPoints2 are double.
%   Otherwise worldPoints is of class single.
%
%   Example - Measure distance to a face
%   ------------------------------------
%   % Load stereoParams
%   load('webcamsSceneReconstruction.mat');
%
%   % Read in the stereo pair of images.
%   I1 = imread('sceneReconstructionLeft.jpg');
%   I2 = imread('sceneReconstructionRight.jpg');
%
%   % Undistort the images
%   I1 = undistortImage(I1, stereoParams.CameraParameters1);
%   I2 = undistortImage(I2, stereoParams.CameraParameters2);
%
%   % Detect a face in both images
%   faceDetector = vision.CascadeObjectDetector;
%   face1 = step(faceDetector, I1);
%   face2 = step(faceDetector, I2);
%
%   % Find the center of the face
%   center1 = face1(1:2) + face1(3:4) / 2;
%   center2 = face2(1:2) + face2(3:4) / 2;
%
%   % Compute the distance from camera 1 to the face
%   point3d = triangulate(center1, center2, stereoParams);
%   distanceInMeters = norm(point3d) / 1000;
%
%   % Display detected face and distance
%   distanceAsString = sprintf('%0.2f meters', distanceInMeters);
%   I1 = insertObjectAnnotation(I1, 'rectangle', face1, distanceAsString, ...
%       'FontSize', 18);
%   I2 = insertObjectAnnotation(I2, 'rectangle', face2, distanceAsString, ...
%       'FontSize', 18);
%   I1 = insertShape(I1, 'FilledRectangle', face1);
%   I2 = insertShape(I2, 'FilledRectangle', face2);
%
%   imshowpair(I1, I2, 'montage');
%
%   See also triangulateMultiview, estimateCameraParameters, cameraCalibrator,
%     stereoCameraCalibrator, cameraProjection, estrelpose, estworldpose,
%     undistortImage, undistortPoints, reconstructScene, cameraParameters,
%     stereoParameters.

% Copyright 2014-2022 MathWorks, Inc.

% References:
%
% Hartley, Richard, and Andrew Zisserman. Multiple View Geometry in
% Computer Vision. Second Edition. Cambridge, 2000. p. 312

%#codegen

function [xyzPoints, reprojectionErrors, validIndex] = triangulate(matchedPoints1, ...
    matchedPoints2, varargin)

[points1, points2, camMatrix1, camMatrix2] = ...
    parseInputs(matchedPoints1, matchedPoints2, varargin{:});

numPoints = size(points1, 2);

% Vectorizing the computation for all points.
% inputPoints (in groups of 4 rows) should be equivalent to A defined for
% each point as follows:
%A = zeros(4, 4);
%A(1:2,:) = points1(:,idx) * camMatrix1(3,:) - camMatrix1(1:2,:);
%A(3:4,:) = point2(:,idx) * camMatrix2(3,:) - camMatrix2(1:2,:);
if isempty(coder.target)
    xyzPoints = vision.internal.triangulatePoints(points1, points2, camMatrix1, camMatrix2, numPoints);
else
    xyzPoints = zeros(numPoints, 3, 'like', points1);
    
    scaleMatrix = [repmat(camMatrix1(3,:), 2, 1); repmat(camMatrix2(3,:), 2, 1)];
    subtractMatrix = [camMatrix1(1:2,:); camMatrix2(1:2,:)];
    
    inputPoints = [points1; points2];
    inputPoints = repmat(inputPoints(:), 1, 4);
    inputPoints = (inputPoints .* repmat(scaleMatrix, numPoints, 1)) ...
        - repmat(subtractMatrix, numPoints, 1);
    
    for i = 1:numPoints
        xyzPoints(i,:) = compute3dPoint(inputPoints((i-1)*4 + 1:i*4, :));
    end
end

if nargout > 1
    % Compute reprojection errors
    [points1proj, isInFrontOfCam1] = projectPoints(xyzPoints, camMatrix1);
    [points2proj, isInFrontOfCam2] = projectPoints(xyzPoints, camMatrix2);
    errors1 = hypot(points1(1,:)-points1proj(1,:), ...
        points1(2,:) - points1proj(2,:));
    errors2 = hypot(points2(1,:)-points2proj(1,:), ...
        points2(2,:) - points2proj(2,:));
    
    reprojectionErrors = mean([errors1; errors2])';
    validIndex = isInFrontOfCam1 & isInFrontOfCam2;
end

%--------------------------------------------------------------------------
function [points1, points2, camMatrix1, camMatrix2] = ...
    parseInputs(matchedPoints1, matchedPoints2, varargin)

narginchk(3, 4);
[points1, points2] = parsePoints(matchedPoints1, matchedPoints2);
[P1, P2] = parseCameraMatrices(varargin{:});
camMatrix1 = cast(P1, 'like', points1);
camMatrix2 = cast(P2, 'like', points2);

%--------------------------------------------------------------------------
function [pts1, pts2] = parsePoints(matchedPoints1, matchedPoints2)

[points1, points2] =  ...
    vision.internal.inputValidation.checkAndConvertMatchedPoints(...
    matchedPoints1, matchedPoints2, mfilename, 'matchedPoints1', ...
    'matchedPoints2');

if isa(points1, 'double')
    pts1 = points1';
    pts2 = points2';
else
    pts1 = single(points1)';
    pts2 = single(points2)';
end

%--------------------------------------------------------------------------
function [cameraMatrix1, cameraMatrix2] = parseCameraMatrices(varargin)
if nargin == 1
    stereoParams = varargin{1};
    validateattributes(stereoParams, {'stereoParameters'}, {}, ...
        mfilename, 'stereoParams');
    cameraMatrix1 = vision.internal.constructCameraMatrix( ...
        eye(3), ...
        [0 0 0], ...
        stereoParams.CameraParameters1.K);
    cameraMatrix2 = vision.internal.constructCameraMatrix( ...
        stereoParams.RotationOfCamera2', ...
        stereoParams.TranslationOfCamera2, ...
        stereoParams.CameraParameters2.K);
else
    narginchk(2, 2);
    % Check cameraMatrix1's format and pass to corresponding validation
    if size(varargin{1},1) == 3
        cameraMatrix1 = varargin{1};
        validateCameraMatrix(cameraMatrix1, 'cameraMatrix1');
    else
        validateCameraMatrixPostMultiply(varargin{1}, 'cameraMatrix1');
        % Make cameraMatrix1 a 3-by-4 matrix
        cameraMatrix1 = varargin{1}';
    end
    % Check cameraMatrix2's format and pass to corresponding validation
    if size(varargin{2},1) == 3
        cameraMatrix2 = varargin{2};
        validateCameraMatrix(cameraMatrix2, 'cameraMatrix2');
    else
        validateCameraMatrixPostMultiply(varargin{2}, 'cameraMatrix2');
        % Make cameraMatrix2 a 3-by-4 matrix
        cameraMatrix2 = varargin{2}';
    end
end

%--------------------------------------------------------------------------
function validateCameraMatrix(M, varName)
validateattributes(M, {'numeric'}, ...
    {'2d', 'size', [3, 4], 'finite', 'real', 'nonsparse'},...
    mfilename, varName);

%--------------------------------------------------------------------------
function validateCameraMatrixPostMultiply(M, varName)
validateattributes(M, {'numeric'}, ...
    {'2d', 'size', [4, 3], 'finite', 'real', 'nonsparse'},...
    mfilename, varName);

%--------------------------------------------------------------------------
function point3d = compute3dPoint(A)
[~,~,V] = svd(A);
X = V(:, end);
X = X/X(end);

point3d = X(1:3)';

%--------------------------------------------------------------------------
function [points2d, isInFrontOfCamera] = projectPoints(points3d, P)
points3dHomog = [points3d, ones(size(points3d, 1), 1, 'like', points3d)]';
points2dHomog = P * points3dHomog;
isInFrontOfCamera = points2dHomog(3, :)' > 0;
points2d = bsxfun(@rdivide, points2dHomog(1:2, :), points2dHomog(3, :));
