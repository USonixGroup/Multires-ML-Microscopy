function camMatrix = cameraProjection(intrinsics, tform)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = false;

camMatrix = vision.internal.calibration.cameraMatrixImpl(intrinsics, ...
    isCameraParamsSupported, mfilename, tform);