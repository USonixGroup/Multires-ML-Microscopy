function varargout = worldToImage(intrinsics, varargin)
% worldToImage Project world points into the image
%
%--------------------------------------------------------------------------
%   worldToImage is not recommended. Use world2img instead.
%--------------------------------------------------------------------------
%
%   imagePoints = worldToImage(intrinsics, tform, worldPoints) projects 3-D
%   world points, worldPoints into points on the image, imagePoints.
%   intrinsics is a cameraIntrinsics or fisheyeIntrinsics object. tform is
%   a rigid3d object specifying the transformation of the camera in world
%   coordinates. worldPoints is an M-by-3 matrix containing [X,Y,Z]
%   coordinates of the world points. The coordinates must be in the same
%   units as the Translation property of the tform rigid3d object.
%   imagePoints is an M-by-2 matrix containing [x,y] coordinates of image
%   points in pixels.
%
%   imagePoints = worldToImage(intrinsics, rotationMatrix,
%   translationVector, worldPoints) specifies the rotation and translation
%   of the camera in world coordinates. rotationMatrix is a 3-by-3 matrix
%   and translationVector is a 3-element vector. The [X,Y,Z] coordinates of
%   worldPoints must be in the same units as translationVector.
%
%   [imagePoints, validIndex] = worldToImage(...) additionally returns a 
%   M-by-1 logical array indicating the indices of valid imagePoints that 
%   are within image bounds. The corresponding world points are inside the
%   field of view of the camera. This syntax is supported for nonfisheye 
%   camera intrinsics.
%
%   [...] = worldToImage(..., 'ApplyDistortion', Value) optionally applies 
%   lens distortion to imagePoints. Set Value to true to apply distortion.
%   By default Value is false. This syntax is supported for nonfisheye 
%   camera intrinsics.
%
%   Class Support
%   -------------
%   intrinsics is a cameraIntrinsics or fisheyeIntrinsics object. tform is
%   a rigid3d object. rotationMatrix, translationVector, and worldPoints
%   can be of class double or single. imagePoints are the same class as
%   worldPoints.
%
%   Example
%   -------
%   % Create a set of calibration images.
%   images = imageDatastore(fullfile(toolboxdir('vision'), 'visiondata', ...
%         'calibration', 'slr'));
%
%   % Detect the checkerboard corners in the images.
%   [imagePoints, boardSize] = detectCheckerboardPoints(images.Files);
%
%   % Generate the world coordinates of the checkerboard corners in the
%   % pattern-centric coordinate system, with the upper-left corner at (0,0).
%   squareSize = 29; % in millimeters
%   worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%
%   % Calibrate the camera.
%   I = readimage(images,1);
%   imageSize = [size(I, 1), size(I, 2)];
%   cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
%                                     'ImageSize', imageSize);
%   
%   % Get the camera intrinsics
%   intrinsics = cameraParams.Intrinsics;
%
%   % Load image at new location.
%   imOrig = imread(fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', ...
%           'calibration', 'slr', 'image9.jpg'));
%   figure
%   imshow(imOrig, 'InitialMagnification', 30);
%
%   % Undistort image.
%   imUndistorted = undistortImage(imOrig, intrinsics);
%
%   % Find reference object in new image.
%   [imagePoints, boardSize] = detectCheckerboardPoints(imUndistorted);
%
%   % Compute new extrinsics.
%   [R, t] = extrinsics(imagePoints, worldPoints, cameraParams);
%
%   % Add a z-coordinate to the world points
%   worldPoints = [worldPoints, zeros(size(worldPoints, 1), 1)];
%
%   % Project world points back into original image
%   projectedPoints = worldToImage(intrinsics, R, t, worldPoints);
%   hold on
%   plot(projectedPoints(:,1), projectedPoints(:,2), 'g*-');
%   legend('Projected points');
%
%   See also img2world2d, cameraIntrinsics, fisheyeIntrinsics,
%       cameraCalibrator, estimateCameraParameters, estworldpose,
%		estimateExtrinsics, rigidtform3d.

% Copyright 2019-2022 The MathWorks, Inc.

%#codegen

narginchk(3,6)

validateattributes(intrinsics, {'cameraIntrinsics', 'fisheyeIntrinsics', ...
    'cameraParameters'}, {'scalar'}, mfilename, 'intrinsics')

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = true;

if ~isa(varargin{1}, 'rigid3d') && ~isa(varargin{1}, 'rigidtform3d')
    % Validate rotationMatrix and translationVector.    
    rotationMatrix = varargin{1};
    vision.internal.inputValidation.validateRotationMatrix(...
        rotationMatrix, mfilename, 'rotationMatrix')
    translationVector = varargin{2};
    vision.internal.inputValidation.validateTranslationVector(...
        translationVector, mfilename, 'translationVector')
    worldPoints = varargin{3};
    idx = 4;
else
    tform = varargin{1};
    validateattributes(tform, {'rigid3d', 'rigidtform3d'}, {'scalar'}, mfilename, 'tform')
    rotationMatrix = tform.Rotation;
    translationVector = tform.Translation;
    worldPoints = varargin{2};
    idx = 3;
end

validateattributes(worldPoints, {'double', 'single'}, ...
    {'real', 'nonsparse', 'nonempty', '2d', 'ncols', 3}, mfilename, 'worldPoints')

[varargout{1:nargout}] = vision.internal.calibration.worldToImageTransform( ...
    intrinsics, rotationMatrix', translationVector, idx, worldPoints, ...
    isCameraParamsSupported, varargin{:});

end