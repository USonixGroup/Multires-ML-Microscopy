function camMatrix = cameraMatrix(cameraParams, varargin)
%cameraMatrix Compute camera projection matrix.
%
%--------------------------------------------------------------------------
%  cameraMatrix is not recommended. Use cameraProjection instead.
%--------------------------------------------------------------------------
%
%  camMatrix = cameraMatrix(cameraParams, tform) returns a 4-by-3 camera
%  projection matrix, which projects a 3-D world point in homogeneous
%  coordinates into the image. cameraParams is a cameraParameters or
%  cameraIntrinsics object. tform is a rigid3d object describing the
%  transformation from the world coordinates to the camera coordinates. You
%  can obtain the rotation and translation to create the tform object using
%  the extrinsics function.
%
%  camMatrix = cameraMatrix(cameraParams, rotationMatrix, translationVector)
%  specifies the rotation and translation from the world coordinates to the
%  camera coordinates. rotationMatrix is a 3-by-3 rotation matrix and
%  translationVector is a 3-element translation vector. You can obtain
%  rotationMatrix and translationVector using the extrinsics function.
%
%  Notes
%  -----
%  The camera matrix is computed as follows:
%  camMatrix = [tform.Rotation; tform.Translation] * K
%  where K is the intrinsic matrix.
%  When the rotationMatrix and translationVector inputs are provided, it is
%  computed as follows:
%  camMatrix = [rotationMatrix; translationVector] * K
%
%  Class Support
%  -------------
%  cameraParams must be a cameraParameters or cameraIntrinsics object.
%  tform must be a rigid3d object. rotationMatrix and translationVector
%  must be numeric arrays of the same class, and must be real and
%  nonsparse. camMatrix is of class double if rotationMatrix and
%  translationVector are double, or if the Rotation and Translation
%  properties of tform are double. Otherwise camMatrix is of class single.
%
%   Example: Compute camera matrix
%   ------------------------------
%   % Create a set of calibration images.
%   images = imageDatastore(fullfile(toolboxdir('vision'), 'visiondata', ...
%       'calibration', 'slr'));
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
%                                          'ImageSize', imageSize);
%  
%   % Load image at new location.
%   imOrig = imread(fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', ...
%         'calibration', 'slr', 'image9.jpg'));
%   figure; imshow(imOrig);
%   title('Input Image');
%  
%   % Undistort image.
%   im = undistortImage(imOrig, cameraParams);
%  
%   % Find reference object in new image.
%   [imagePoints, boardSize] = detectCheckerboardPoints(im);
%
%   % Compute new extrinsics.
%   [rotationMatrix, translationVector] = extrinsics(...
%       imagePoints, worldPoints, cameraParams);
%
%   % Calculate camera matrix
%   P = cameraMatrix(cameraParams, rotationMatrix, translationVector)
%
%  See also estimateExtrinsics, triangulate, cameraCalibrator, estimateCameraParameters
%    estrelpose, estworldpose, pose2extr, rigidtform3d.

% Copyright 2018-2022 MathWorks, Inc.

%#codegen

narginchk(2,3)

% True if cameraParameters is a supported class for cameraParams
isCameraParamsSupported = true;

camMatrix = vision.internal.calibration.cameraMatrixImpl(cameraParams, ...
    isCameraParamsSupported, mfilename, varargin{:})';