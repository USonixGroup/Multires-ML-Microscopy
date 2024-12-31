function camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, intrinsics)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = false;

[rotationMatrix, translationVector] = vision.internal.calibration.estimateExtrinsicsImpl( ...
    imagePoints, worldPoints, intrinsics, mfilename, isCameraParamsSupported);

camExtrinsics = rigidtform3d(rotationMatrix, translationVector);