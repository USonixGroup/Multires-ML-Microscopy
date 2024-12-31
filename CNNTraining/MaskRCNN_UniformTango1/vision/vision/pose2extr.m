function extrinsics = pose2extr(camPose)

% Copyright 2022-2023 The MathWorks, Inc.

%#codegen

% Validate camPose input.
validateattributes(camPose, {'rigidtform3d'}, {'scalar'}, mfilename, 'camPose')

% Get extrinsics output.
extrinsics = camPose.invert;
end
