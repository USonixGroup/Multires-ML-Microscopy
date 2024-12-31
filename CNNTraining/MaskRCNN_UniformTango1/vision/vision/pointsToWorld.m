function worldPoints = pointsToWorld(intrinsics, varargin)
%pointsToWorld Determine world coordinates of image points
%
%--------------------------------------------------------------------------
%   pointsToWorld is not recommended. Use img2world2d instead.
%--------------------------------------------------------------------------
%
%   worldPoints = pointsToWorld(intrinsics, tform, imagePoints) maps
%   undistorted image points, imagePoints onto points on the X-Y plane in
%   world coordinates, worldPoints. intrinsics is a cameraIntrinsics or
%   fisheyeIntrinsics object. tform is a rigid3d object specifying the
%   transformation of the camera in world coordinates. imagePoints is an
%   M-by-2 matrix of [x,y] coordinates or an M-element feature point array,
%   where M is the number of points. worldPoints is an M-by-2 matrix of
%   [X,Y] world coordinates. The Z-coordinate for all world points is 0.
%
%   worldPoints = pointsToWorld(intrinsics, rotationMatrix,
%   translationVector, imagePoints) specifies the rotation and translation
%   of the camera in world coordinates. rotationMatrix is a 3-by-3 matrix
%   and translationVector is a 3-element vector.
%
%   Notes
%   -----
%   The function does not account for lens distortion. imagePoints must
%   either be detected in the undistorted image, or they must be undistorted
%   using the undistortPoints function. If imagePoints are detected in
%   undistorted image, their coordinates must be translated to the coordinate
%   system of the original image.
%
%   Class Support
%   -------------
%   intrinsics is a cameraIntrinsics or fisheyeIntrinsics object. tform is
%   a rigid3d object. rotationMatrix, translationVector, and imagePoints
%   must be real and nonsparse numeric arrays. worldPoints is of class
%   double if imagePoints are double. Otherwise worldPoints is of class
%   single.
%
%   Example 1: Map Image Points of Fisheye Image to World Coordinates
%   -----------------------------------------------------------------
%   % Create a set of checkerboard calibration images.
%   images = imageDatastore(fullfile(toolboxdir('vision'),'visiondata' ,...
%     'calibration','gopro'));
%
%   % Detect the checkerboard corners in the images. Leave the last image for testing.
%   [imagePoints,boardSize] = detectCheckerboardPoints(images.Files(1:end-1));
% 
%   % Generate the world coordinates of the checkerboard corners in the pattern-centric
%   % coordinate system, with the upper-left corner at (0,0).
%   squareSize = 29; % millimeters
%   worldPoints = generateCheckerboardPoints(boardSize,squareSize);
% 
%   % Estimate the fisheye camera parameters from the image and world points. 
%   % Use the first image to get image size.
%   I = imread(images.Files{end}); 
%   imageSize = [size(I,1) size(I,2)];
%   fisheyeParams = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);
%   intrinsics = fisheyeParams.Intrinsics;
% 
%   % Find the reference object in the new image.
%   imagePoints = detectCheckerboardPoints(I, 'PartialDetections', false);
% 
%   % Compute new extrinsics.
%   [R,t] = extrinsics(imagePoints,worldPoints,intrinsics);
% 
%   % Map image points to world coordinates in the X-Y plane.
%   newWorldPoints = pointsToWorld(intrinsics,R,t,imagePoints);
% 
%   % Compare estimated world points to the ground truth points.
%   plot(worldPoints(:,1),worldPoints(:,2),'gx');
%   hold on
%   plot(newWorldPoints(:,1),newWorldPoints(:,2),'ro');
%   legend('Ground Truth','Estimates');
%   hold off
%   
%   See also world2img, estimateCameraParameters, cameraCalibrator,
%       estimateExtrinsics, undistortImage, cameraIntrinsics, fisheyeIntrinsics
%       reconstructScene, triangulate, triangulateMultiview, rigidtform3d.

% Copyright 2019-2023 The MathWorks, Inc.

%#codegen

narginchk(3,4)

validateattributes(intrinsics, {'cameraIntrinsics', 'fisheyeIntrinsics', ...
    'cameraParameters'}, {'scalar'}, mfilename, 'intrinsics');

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = true;

if nargin == 4
    rotationMatrix = varargin{1};
    translationVector = varargin{2};
    imagePoints = varargin{3};

    % Validate rotationMatrix and translationVector.
    vision.internal.inputValidation.validateRotationMatrix(...
        rotationMatrix, mfilename, 'rotationMatrix');
    vision.internal.inputValidation.validateTranslationVector(...
        translationVector, mfilename, 'translationVector');
else
    rigidTform = varargin{1};
    imagePoints = varargin{2};
    
    validateattributes(rigidTform, {'rigid3d', 'rigidtform3d'}, {'scalar'}, mfilename, 'tform')
    
    rotationMatrix = rigidTform.Rotation;
    translationVector = rigidTform.Translation;
end

imagePoints = vision.internal.inputValidation.checkAndConvertPoints(...
    imagePoints, mfilename, 'imagePoints');

% Map to X-Y world plane
worldPoints = vision.internal.calibration.mapXYWorldPlane(intrinsics, ...
    rotationMatrix', translationVector, imagePoints, isCameraParamsSupported);