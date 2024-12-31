%  EXTRINSICS Compute camera extrinsics from a planar calibration pattern
%
%--------------------------------------------------------------------------
%     EXTRINSICS is not recommended. Use estimateExtrinsics instead.
%--------------------------------------------------------------------------
%
%     [rotationMatrix, translationVector] = EXTRINSICS(imagePoints,
%     worldPoints, cameraParams) returns the translation and rotation from
%     the world coordinate system defined by worldPoints into the camera-based
%     coordinate system.
%     
%     Inputs            Description
%     ------            -----------
%     imagePoints       M-by-2 array of undistorted [x,y] coordinates of 
%                       image points
%  
%     worldPoints       Coordinates of co-planar world points corresponding to 
%                       imagePoints, specified as an M-by-2 matrix of [x,y] coordinates 
%                       with z assumed to be 0 and M >= 4.
% 
%     cameraParams      cameraParameters, cameraIntrinsics or fisheyeIntrinsics object
%  
%     Outputs           Description
%     -------           -----------
%     rotationMatrix    3-by-3 matrix describing rotation from the world coordinates
%                       to the camera-based coordinates
%     translationVector 1-by-3 vector describing translation from the world coordinates
%                       to the camera-based coordinates
%  
%     Notes
%     -----
%     If cameraParams is a cameraParameters or cameraIntrinsics object, the
%     lens distortion is not taken into account. You can either undistort
%     the image using the undistortImage function before detecting the
%     points, or you can undistort the points using the undistortPoints
%     function.
%  
%     Class Support
%     -------------
%     imagePoints and worldPoints must both be double or both be single.
%     cameraParams must be a cameraParameters, cameraIntrinsics or
%     fisheyeIntrinsics object.
%  
%     If imagePoints and worldPoints are of class double, then rotationMatrix
%     and translationVector are also double. Otherwise, they are single.
%  
%     Example: Compute camera extrinsics
%     -----------------------------------
%     % Create a set of calibration images.
%     images = imageDatastore(fullfile(toolboxdir('vision'), 'visiondata', ...
%         'calibration', 'slr'));
%  
%     % Detect the checkerboard corners in the images.
%     [imagePoints, boardSize] = detectCheckerboardPoints(images.Files);
%  
%     % Generate the world coordinates of the checkerboard corners in the
%     % pattern-centric coordinate system, with the upper-left corner at (0,0).
%     squareSize = 29; % in millimeters
%     worldPoints = generateCheckerboardPoints(boardSize, squareSize);
%  
%     % Calibrate the camera.
%     I = readimage(images,1); 
%     imageSize = [size(I, 1), size(I, 2)];
%     cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
%                                     'ImageSize', imageSize);
%  
%     % Load image at new location.
%     imOrig = imread(fullfile(matlabroot, 'toolbox', 'vision', 'visiondata', ...
%           'calibration', 'slr', 'image9.jpg'));
%     figure 
%     imshow(imOrig);
%     title('Input Image');
%  
%     % Undistort image.
%     [im, newCameraParams] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
%  
%     % Find reference object in new image.
%     [imagePoints, boardSize] = detectCheckerboardPoints(im);
%
%     % Compensate for image coordinate system shift.
%     newOrigin = cameraParams.PrincipalPoint - newCameraParams.PrincipalPoint;
%     imagePoints = [imagePoints(:,1) + newOrigin(1), ...
%                    imagePoints(:,2) + newOrigin(2)];
%  
%     % Compute new extrinsics.
%     [rotationMatrix, translationVector] = EXTRINSICS(...
%       imagePoints, worldPoints, cameraParams);
%
%     % Compute camera pose.
%     [orientation, location] = extrinsicsToCameraPose(rotationMatrix, ...
%         translationVector);
%     figure
%     plotCamera('Location', location, 'Orientation', orientation, 'Size', 20);
%     hold on
%     pcshow([worldPoints, zeros(size(worldPoints,1), 1)], ...
%         'VerticalAxisDir', 'down', 'MarkerSize', 40);
%  
%     See also estimateCameraParameters, cameraCalibrator, cameraProjection,
%         cameraParameters, undistortImage, undistortPoints, plotCamera, 
%         extr2pose, pose2extr, world2img, img2world2d, cameraIntrinsics,
%         fisheyeIntrinsics.

% Copyright 2013-2023 MathWorks, Inc

% References:
% -----------
% [1] Z. Zhang. A flexible new technique for camera calibration. IEEE
% Transactions on Pattern Analysis and Machine Intelligence,
% 22(11):1330-1334, 2000.
%
% [2] Hartley, Richard, and Andrew Zisserman. Multiple View Geometry in
% Computer Vision. Vol. 2. Cambridge, 2000.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

function [rotationMatrix, translationVector] = extrinsics(...
    imagePoints, worldPoints, cameraParams)

if ~(coder.target('MATLAB') || coder.target('MEX')) && (isempty(imagePoints) || isempty(worldPoints))
    rotationMatrix = [];
    translationVector = [];
    return;
end

% True if cameraParameters is a supported class for cameraParams
isCameraParamsSupported = true;

[rotationMatrix, translationVector] = vision.internal.calibration.estimateExtrinsicsImpl( ...
    imagePoints, worldPoints, cameraParams, mfilename, isCameraParamsSupported);

% Transpose rotationMatrix to the transpose of the regular definition
rotationMatrix = rotationMatrix';