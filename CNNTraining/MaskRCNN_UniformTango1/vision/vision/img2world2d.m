function worldPoints = img2world2d(imagePoints, rigidTform, intrinsics)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

% Validate imagePoints
imagePoints = vision.internal.inputValidation.checkAndConvertPoints(...
    imagePoints, mfilename, 'imagePoints');

% Validate rigidTform and pull the rotation matrix and translation vector from rigidTform
validateattributes(rigidTform, {'rigidtform3d'}, {'scalar'}, mfilename, 'tform')
rotationMatrix = rigidTform.R;
translationVector = rigidTform.Translation;

% Validate intrinsics
validateattributes(intrinsics, {'cameraIntrinsics', 'fisheyeIntrinsics', 'cameraIntrinsicsKB'}, ...
    {'scalar'}, mfilename, 'intrinsics');

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = false;

% Map to X-Y world plane
worldPoints = vision.internal.calibration.mapXYWorldPlane(intrinsics, ...
    rotationMatrix, translationVector, imagePoints, isCameraParamsSupported);