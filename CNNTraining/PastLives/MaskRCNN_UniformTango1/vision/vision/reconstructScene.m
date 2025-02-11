% reconstructScene Reconstructs a 3-D scene from a disparity map.
%   xyzPoints = reconstructScene(disparityMap, reprojectionMatrix) returns
%   an M-by-N-by-3 array of [X,Y,Z] coordinates of world points corresponding
%   to pixels in disparityMap, an M-by-N array of disparity values.
%   reprojectionMatrix is a 4-by-4 matrix which can be calculated from a
%   pair of stereo images using the rectifyStereoImages function. The 3-D
%   world coordinates are relative to the optical center of camera 1 in a
%   stereo system. 
%
%   Notes
%   -----
%   - reprojectionMatrix is represented as a 4-by-4 matrix:
%       [1    0   0     -cx
%        0    1   0     -cy
%        0    0   0     f
%        0    0   1/b   0],
%     where f and [cx, cy] are the focal length and principal point of
%     rectified camera 1, respectively. b is the baseline of the virtual 
%     rectified stereo camera.
%   - disparity function uses -realmax('single') to mark pixels for which
%     disparity estimate is unreliable. For such pixels reconstructScene
%     sets the world coordinates to NaN. For pixels with zero disparity,
%     the world coordinates are set to Inf.
%
%   Class Support
%   -------------
%   disparityMap and reprojectionMatrix can be double or single. xyzPoints 
%   is double if disparityImage is double, otherwise it is single.
%
%   Example:
%   --------
%   % Load stereoParams
%   load('webcamsSceneReconstruction.mat');
%
%   % Read in the stereo pair of images.
%   I1 = imread('sceneReconstructionLeft.jpg');
%   I2 = imread('sceneReconstructionRight.jpg');
%
%   % Rectify the images.
%   [J1, J2, reprojectionMatrix] = rectifyStereoImages(I1, I2, stereoParams);
%
%   % Display the images after rectification.
%   figure
%   imshow(stereoAnaglyph(J1, J2), 'InitialMagnification', 50);
%
%   % Compute disparity
%   disparityMap = disparitySGM(rgb2gray(J1), rgb2gray(J2), ...
%   'DisparityRange', [0 64]);
%   figure
%   imshow(disparityMap, [0, 64], 'InitialMagnification', 50);
%
%   % Reconstruct the 3-D world coordinates of points corresponding to each
%   % pixel from the disparity map.
%   xyzPoints = reconstructScene(disparityMap, reprojectionMatrix);
%
%   % Segment out a person located between 3.2 and 3.7 meters away from the
%   % camera.
%   Z = xyzPoints(:, :, 3);
%   mask = repmat(Z > 3200 & Z < 3700, [1, 1, 3]);
%   J1(~mask) = 0;
%   figure
%   imshow(J1, 'InitialMagnification', 50);
%
%   See also estimateCameraParameters, rectifyStereoImages, disparityBM,
%            disparitySGM, stereoParameters

% Copyright 2013-2021 MathWorks, Inc.

% References:
%
% G. Bradski and A. Kaehler, "Learning OpenCV : Computer Vision with
% the OpenCV Library," O'Reilly, Sebastopol, CA, 2008.

%#codegen

function xyzPoints = reconstructScene(disparityMap, stereoParamsOrReprojMat)

validateattributes(disparityMap, {'double', 'single'}, ...
    {'2d', 'real', 'nonsparse'}, mfilename, 'disparityImage');

if isnumeric(stereoParamsOrReprojMat) % Since R2022a: Reprojection matrix
    validateReprojectionMatrix(stereoParamsOrReprojMat);

    % Output is of the same class as disparity map
    reprojectionMatrix = cast(stereoParamsOrReprojMat, 'like', disparityMap);
    
    % Transpose the reprojection matrix to compute 3-D points as
    % [x y d 1] * reprojectionMatrix = [X Y Z W]
    if isempty(coder.target)
        xyzPoints = visionReconstructScene(disparityMap, reprojectionMatrix');
    else
        xyzPoints = vision.internal.calibration.reconstructFromDisparity(...
            disparityMap, reprojectionMatrix');
    end
else % Before R2022a: stereoParameters object
    validateattributes(stereoParamsOrReprojMat, {'stereoParameters'}, {'scalar'}, ...
        mfilename, 'stereoParams');

    xyzPoints = reconstructSceneImpl(stereoParamsOrReprojMat, disparityMap);
end
end

function validateReprojectionMatrix(reprojMatrix)
validateattributes(reprojMatrix, {'double', 'single'}, ...
    {'size', [4, 4], 'nonempty', 'real', 'nonsparse'}, mfilename, 'reprojectionMatrix');
end