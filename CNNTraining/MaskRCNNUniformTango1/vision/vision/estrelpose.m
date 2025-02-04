function [relPose, validPointsFraction] = estrelpose(M, varargin)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

% True if cameraParameters is a supported class for intrinsics
isCameraParamsSupported = false;

[relPose, validPointsFraction] = vision.internal.calibration.estRelPoseImpl( ...
    M, isCameraParamsSupported, mfilename, varargin{:});

end